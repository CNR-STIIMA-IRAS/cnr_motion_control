#pragma once // workaround qtcreator clang-tidy

#ifndef CNR_FOLLOW_JOINT_TRAJECTORY_CONTROLLER_CNR_FOLLOW_JOINT_TRAJECTORY_CONTROLLER_IMPL_H
#define CNR_FOLLOW_JOINT_TRAJECTORY_CONTROLLER_CNR_FOLLOW_JOINT_TRAJECTORY_CONTROLLER_IMPL_H

#include <memory>
#include <thread>
#include <controller_interface/controller_base.h>
#include <name_sorting/sort_trajectories.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <cnr_follow_joint_trajectory_controller/cnr_follow_joint_trajectory_controller.h>
#include <cnr_interpolator_interface/cnr_interpolator_inputs.h>
#include <cnr_interpolator_interface/cnr_interpolator_outputs.h>
#include <cnr_interpolator_interface/cnr_interpolator_point.h>
#include <cnr_interpolator_interface/cnr_interpolator_trajectory.h>

#include <cnr_regulator_interface/cnr_regulator_state.h>
#include <cnr_regulator_interface/cnr_regulator_params.h>
#include <cnr_regulator_interface/cnr_regulator_references.h>
#include <cnr_regulator_interface/cnr_regulator_feedback.h>
#include <cnr_regulator_interface/cnr_regulator_control_commands.h>
#include <rosparam_utilities/rosparam_utilities.h>
namespace eu = eigen_utils;

namespace cnr
{
namespace control
{

template<class H, class T>
FollowJointTrajectoryController<H,T>::~FollowJointTrajectoryController()
{
  CNR_DEBUG(this->logger(), "Destroying Thor Prefilter Controller");
  joinActionServerThread();
  m_as.reset();
  CNR_DEBUG(this->logger(), "Destroyed Thor Prefilter Controller");
}

template<class H, class T>
FollowJointTrajectoryController<H,T>::FollowJointTrajectoryController()
  : m_interpolator_loader("cnr_interpolator_interface", "cnr::control::InterpolatorBase")
  , m_regulator_loader("cnr_regulator_interface", "cnr::control::BaseRegulator")
{
  m_is_in_tolerance = false;
  m_preempted = false;
  m_interpolator.reset();
}

template<class H, class T>
bool FollowJointTrajectoryController<H,T>::doInit()
{
  CNR_TRACE_START(this->logger());

  std::string interpolator;
  if(!this->getControllerNh().getParam("interpolator", interpolator))
  {
    CNR_RETURN_FALSE(this->logger(),"The param '"+this->getControllerNamespace()+"/interpolator' is missing. Abort.")
  }
  try
  {
    auto interp = m_interpolator_loader.createInstance(interpolator);
    m_interpolator = cnr::control::to_std_ptr(interp);
    if(!m_interpolator->initialize(this->logger(), this->getControllerNh()))
    {
      CNR_RETURN_FALSE(this->logger(), "The interpolator init failed. Abort.")
    }
  }
  catch(pluginlib::PluginlibException& ex)
  {
    CNR_ERROR(this->logger(), "Failed in loading the interpolator plugin: '" + std::string(ex.what())+"'");
    CNR_RETURN_FALSE(this->logger(), "Failed in loading the interpolator plugin.");
  }


  std::string regulator;
  if(!this->getControllerNh().getParam("regulator", regulator))
  {
    CNR_RETURN_FALSE(this->logger(), "The param '"+this->getControllerNamespace()+"/regulator' is missing. Abort.")
  }
  try
  {
    auto reg = m_regulator_loader.createInstance(regulator);
    m_regulator = cnr::control::to_std_ptr(reg);

    JointRegulatorParamsPtr opts(new JointRegulatorParams());
    opts->logger    = this->logger();
    opts->period    = ros::Duration(this->m_sampling_period);
    opts->dim       = this->nAx();
    opts->resources_names = this->chain().getActiveJointsName();
    opts->interpolator = m_interpolator;
    if(!m_regulator->initialize(this->getRootNh(), this->getControllerNh(), opts))
    {
      CNR_RETURN_FALSE(this->logger(), "The regulator init failed. Abort.")
    }
    m_r.reset(new JointRegulatorReference());
    m_r->set_dimension(this->chain().getActiveJointsNumber());

    m_u.reset(new JointRegulatorControlCommand());
    m_u->set_dimension(this->chain().getActiveJointsNumber());

  }
  catch(pluginlib::PluginlibException& ex)
  {
    CNR_RETURN_FALSE(this->logger(), "The plugin failed to load for some reason. Error: " + std::string(ex.what()));
  }

  std::string what;
  // ^^^^^^
  rosdyn::VectorXd goal_tolerance;  //it may be a double or a eigen::vector
  eu::setConstant(goal_tolerance, 0.001);
  if(!rosparam_utilities::getParam(this->getControllerNh(), "goal_tolerance", goal_tolerance, what, &goal_tolerance))
  {
    CNR_ERROR(this->logger(), "Error in getting the goal tolerance: " << what);
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_WARN_COND(this->logger(), what.size()>0, what);
  eu::copy(m_r->goal_tolerance, goal_tolerance);
  // ^^^^^^

  rosdyn::VectorXd path_tolerance;  //it may be a double or a eigen::vector
  eu::setConstant(path_tolerance, 0.001);
  if(!rosparam_utilities::getParam(this->getControllerNh(), "path_tolerance", path_tolerance, what, &path_tolerance))
  {
    CNR_ERROR(this->logger(), "Error in getting the path tolerance: " << what);
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_WARN_COND(this->logger(), what.size()>0, what);
  eu::copy(m_r->path_tolerance, path_tolerance);

  m_as.reset(new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>(
                    this->getControllerNh(),
                    "follow_joint_trajectory",
                    boost::bind(&FollowJointTrajectoryController<H,T>::actionGoalCallback, this, _1),
                    boost::bind(&FollowJointTrajectoryController<H,T>::actionCancelCallback, this, _1),
                    false));

  CNR_RETURN_TRUE(this->logger());
}

template<class H, class T>
bool FollowJointTrajectoryController<H,T>::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());

  trajectory_msgs::JointTrajectoryPoint pnt;
  pnt.time_from_start = ros::Duration(0.0);
  pnt.positions   = std::vector<double>(this->getPosition().data(), this->getPosition().data() + this->nAx());
  pnt.velocities  = std::vector<double>(this->getVelocity().data(), this->getVelocity().data() + this->nAx());
  pnt.accelerations.resize(this->nAx(), 0);
  pnt.effort.resize(this->nAx(), 0);

  JointTrajectoryPtr trj(new JointTrajectory({pnt}));
  m_interpolator->setTrajectory(trj);

  m_x0.reset(new JointRegulatorState());
  m_x0->robotState().copy(this->chainState(),this->chainState().FULL_STATE);

  m_regulator->starting(m_x0, ros::Time::now());
  m_r->target_override = 1.0;

  m_as->start();
  CNR_RETURN_TRUE(this->logger());
}

template<class H, class T>
bool FollowJointTrajectoryController<H,T>::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  try
  {
    m_r->target_override = this->getTargetOverride();
    m_r->period = period;

    //CNR_DEBUG_THROTTLE(this->logger(),  1, "ovr: " << m_r->target_override << " period: " << m_r->period);
    m_regulator->update(m_r, m_u);  // the regulator call the interpolator!

    m_is_in_tolerance = m_u->in_goal_tolerance;
    //CNR_DEBUG_THROTTLE(this->logger(), 1,  "is in tolerance? " << m_is_in_tolerance);

    //CNR_DEBUG_THROTTLE(this->logger(), 1, "u: " << eu::to_string(m_u->x));

    this->setCommandPosition     (m_u->x);
    this->setCommandVelocity     (m_u->xd);
    this->setCommandAcceleration (m_u->xdd);
    this->setCommandEffort       (m_u->eff);
  }
  catch (std::exception& e)
  {
    CNR_ERROR(this->logger(), "Got and exception: '" << e.what()
               << "'(function input: Time: " << time.toSec() << " Duration: " << period.toSec() << ")");
    //CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

template<class H, class T>
bool FollowJointTrajectoryController<H,T>::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());
  if (m_gh)
  {
    m_gh->setCanceled();
  }
  joinActionServerThread();
  m_gh.reset();
  CNR_RETURN_TRUE(this->logger());
}

template<class H, class T>
bool FollowJointTrajectoryController<H,T>::joinActionServerThread()
{
  m_preempted = true;
  if (m_as_thread.joinable())
  {
    m_as_thread.join();
  }
  m_preempted = false;
  return true;
}

template<class H, class T>
void FollowJointTrajectoryController<H,T>::actionServerThread()
{
  std::stringstream report;
  CNR_DEBUG(this->logger(), "START ACTION GOAL LOOPING");
  ros::WallRate lp(100);
  while (ros::ok())
  {
    lp.sleep();
    if (!m_gh)
    {
      CNR_ERROR(this->logger(), "Goal handle is not initialized");
      break;
    }

    if ((m_preempted)
    || (m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::RECALLED)
    || (m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTED))
    {
      CNR_WARN(this->logger(), "Action Server Thread Preempted");
      CNR_DEBUG(this->logger(), "(m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTED)  = "
                << (int)(m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTED));
      CNR_DEBUG(this->logger(), "m_preempted  = %d" << (int)m_preempted);
      CNR_DEBUG(this->logger(), "(m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::RECALLED)  = "
                << (int)(m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::RECALLED));

      this->addDiagnosticsMessage("OK", (m_preempted ? "preempted" : "cancelled"), {{"Interpolator", "Cancelled"}}, &report);
      CNR_INFO(this->logger(), report.str());

      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = 0;
      result.error_string = "preempted";
      m_gh->setRejected(result);
      CNR_DEBUG(this->logger(), "Preempted old goal DONE");

      break;
    }

    if(m_is_finished == 1 || m_is_in_tolerance)
    {
      this->addDiagnosticsMessage("OK", "£", {{"INTERPOLATOR", "Goal tolerance achieved!"}} , &report);
      CNR_INFO(this->logger(), report.str());

      control_msgs::FollowJointTrajectoryResult result;
      m_gh->setSucceeded(result);

      break;
    }
    else if (m_is_finished == -2)
    {
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = -4;
      result.error_string = "Some problem occurs";

      this->addDiagnosticsMessage("ERROR", "£", {{"INTERPOLATOR", "Some problem occurs"}}, &report);
      CNR_ERROR(this->logger(), report.str());

      m_gh->setAborted(result);
      break;
    }
  }
  m_gh.reset();
  CNR_DEBUG(this->logger(), "START ACTION GOAL END");
}

template<class H, class T>
void FollowJointTrajectoryController<H,T>::actionGoalCallback(
                                    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh)
{
  CNR_TRACE_START(this->logger());
  CNR_INFO(this->logger(), "Received a goal");
  auto goal = gh.getGoal();

  boost::shared_ptr<actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle> current_gh;

  if (m_gh)
  {
    // PREEMPTED OLD GOAL
    CNR_INFO(this->logger(), "preempting old goal");
    m_gh->setAborted();
    joinActionServerThread();

    CNR_INFO(this->logger(), "Goal Stopped");
  }
  else
  {
    CNR_INFO(this->logger(), "No goals running yet");
  }

  current_gh.reset(new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle(gh));
  m_gh = current_gh;
  unsigned  int nPnt = goal->trajectory.points.size();

  if (nPnt == 0)
  {
    CNR_DEBUG(this->logger(),"TRAJECTORY WITH NO POINT");
    control_msgs::FollowJointTrajectoryResult result;
    m_gh->setAccepted();
    current_gh->setSucceeded(result);
    m_gh.reset();
    return;
  }
  CNR_DEBUG(this->logger(),"Trajectory composed of " << nPnt << " points");
  CNR_DEBUG(this->logger(),"Sort the names");
  JointTrajectoryPtr interp_trj(new JointTrajectory());
  if (!trajectory_processing::sort_trajectory(this->jointNames(), goal->trajectory, *interp_trj->trj))
  {
    CNR_ERROR(this->logger(), "Names are different");
    m_gh->setAborted();
    joinActionServerThread();
    return;
  }
  CNR_DEBUG(this->logger(),"[Check] Trajectory composed of " << nPnt << " points");

  try
  {
    std::lock_guard<std::mutex> lock(m_mtx);
    m_interpolator->setTrajectory(interp_trj);
    m_regulator->starting(m_x0, ros::Time::now());
    CNR_DEBUG(this->logger(), "Starting managing new goal");

    auto str = [](const std::vector<double>& q) -> std::string
    {
      std::string ret = "[ ";
      for(size_t i=0;i<q.size()-1;i++)
      {
        ret += std::to_string(q[i]) + ", ";
      }
      return ret + std::to_string(q.back()) + "]";;
    };

    CNR_DEBUG(this->logger(), "First Point of the trajectory:\n q : " <<
                str(interp_trj->trj->points.front().positions) << "\n" << " qd:"
                  << str(interp_trj->trj->points.front().velocities) );

    CNR_DEBUG(this->logger(), "Last Point of the trajectory:\n q : " <<
                str(interp_trj->trj->points.back().positions) << "\n" << " qd:"
                  << str(interp_trj->trj->points.back().velocities) );

    m_is_finished=0;
  }
  catch(std::exception& e)
  {
     CNR_ERROR(this->logger(), "Set Trajectory Failed");
  }

  m_gh->setAccepted();

  joinActionServerThread();

  m_as_thread = std::thread(&FollowJointTrajectoryController<H,T>::actionServerThread, this);

  CNR_RETURN_OK(this->logger(), void());
}

template<class H, class T>
void FollowJointTrajectoryController<H,T>::actionCancelCallback(
                          actionlib::ActionServer< control_msgs::FollowJointTrajectoryAction >::GoalHandle /*gh*/)
{
  CNR_TRACE_START(this->logger());
  CNR_DEBUG(this->logger(), "Cancel active goal Callback");
  if (m_gh)
  {
    m_gh->setCanceled();
    joinActionServerThread();
    m_gh.reset();

    try
    {
      std::lock_guard<std::mutex> lock(m_mtx);
      JointTrajectoryPtr interp_trj(new JointTrajectory());
      interp_trj->append(m_interpolator->getLastInterpolatedPoint());
      m_interpolator->setTrajectory(interp_trj);
      m_regulator->setRegulatorTime(ros::Duration(0));
    }
    catch(std::exception& e)
    {
      CNR_ERROR(this->logger(), "Set Trajectory Failed. Exception:" << std::string(e.what()));
    }
  }
  else
  {
    CNR_WARN(this->logger(), "No goal to cancel");
  }
  CNR_RETURN_OK(this->logger(), void());
}

}  // namespace control
}  // namespace cnr


#endif  // CNR_FOLLOW_JOINT_TRAJECTORY_CONTROLLER_CNR_FOLLOW_JOINT_TRAJECTORY_CONTROLLER_IMPL_H
