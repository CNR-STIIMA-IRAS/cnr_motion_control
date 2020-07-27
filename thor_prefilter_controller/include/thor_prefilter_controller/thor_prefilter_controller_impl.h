#ifndef THOR_PREFILTER_CONTROLLER_THOR_PREFILTER_CONTROLLER_IMPL_H
#define THOR_PREFILTER_CONTROLLER_THOR_PREFILTER_CONTROLLER_IMPL_H

#include <name_sorting/sort_trajectories.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <thor_prefilter_controller/thor_prefilter_controller.h>

namespace thor
{

template< class T>
PrefilterController<T>::~PrefilterController()
{
  CNR_DEBUG(*m_joint_ctrl->logger(), "Destroying Thor Prefilter Controller");
  joinActionServerThread();
  m_as.reset();
  CNR_DEBUG(*m_joint_ctrl->logger(), "Destroyed Thor Prefilter Controller");
}

template< class T>
bool PrefilterController<T>::joinActionServerThread()
{
  m_preempted = true;
  if (m_as_thread.joinable())
  {
    m_as_thread.join();
  }
  m_preempted = false;
  return true;
}

template< class T>
PrefilterController<T>::PrefilterController(cnr_controller_interface::JointController<T>* joint_ctrl) : m_joint_ctrl(joint_ctrl)
{
  m_override = 1;
  m_safe_override_1 = 1;
  m_safe_override_2 = 1;
  m_is_in_tolerance = true;
  m_goal_tolerance = 0.00001;
  m_path_tolerance = 0.01;
  m_preempted = false;

  assert(joint_ctrl);

  //------------------------------------ NP -----------
  m_joint_ctrl->add_subscriber("speed_ovr"  , "/speed_ovr" , 1, &thor::PrefilterController<T>::overrideCallback, this);
  m_joint_ctrl->add_subscriber("safe_ovr_1" , "/safe_ovr_1", 1, &thor::PrefilterController<T>::safeOverrideCallback_1, this);
  m_joint_ctrl->add_subscriber("safe_ovr_2" , "/safe_ovr_2", 1, &thor::PrefilterController<T>::safeOverrideCallback_2, this);
  m_joint_ctrl->template add_publisher < std_msgs::Float64 > ("scaled_time", "/scaled_time", 1);
  m_joint_ctrl->template add_publisher < sensor_msgs::JointState >("target"     , "/target/joint_states", 1);
  //---------------------------------------------------

}

template< class T>
void PrefilterController<T>::starting(const ros::Time& time, const std::vector< double >& qini, const std::vector< double >& Dqini)
{
  assert(m_joint_ctrl->getJointNames().size() == qini.size());
  assert(m_joint_ctrl->getJointNames().size() == Dqini.size());

  m_pnt.positions   = qini;
  m_pnt.velocities  = Dqini;
  m_pnt.accelerations.resize(m_joint_ctrl->getJointNames().size(), 0);
  m_pnt.effort.resize(m_joint_ctrl->getJointNames().size(), 0);
  m_pnt.time_from_start = ros::Duration(0);
  trajectory_msgs::JointTrajectoryPtr trj(new trajectory_msgs::JointTrajectory());
  trj->points.push_back(m_pnt);
  m_thor_prefilter.reset(new thor::ThorPrefilter());
  m_thor_prefilter->setTrajectory(trj);
  m_scaled_time = ros::Duration(0);;
  m_time = ros::Duration(0);
  int spline_order = 1;
  if (!m_joint_ctrl->getControllerNh().getParam("spline_order", spline_order))
  {
    CNR_WARN(*m_joint_ctrl->logger(), "spline_order is not set, set equal to 1");
    spline_order = 1;
  }
  if (spline_order < 0)
    spline_order = 0;
  m_thor_prefilter->setSplineOrder(spline_order);

  m_as.reset(new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>(m_joint_ctrl->getControllerNh(), "follow_joint_trajectory",
             boost::bind(&thor::PrefilterController<T>::actionGoalCallback,    this,  _1),
             boost::bind(&thor::PrefilterController<T>::actionCancelCallback,  this,  _1),
             false));
  m_as->start();
}

template< class T>
bool PrefilterController<T>::update(const ros::Time& time, const ros::Duration& period)
{
  try
  {
    if (!m_thor_prefilter->interpolate(m_scaled_time, m_pnt, m_override * m_safe_override_1 * m_safe_override_2))
    {
      CNR_ERROR(*m_joint_ctrl->logger(), "something wrong in interpolation.");
      return false;
    }
    m_scaled_time += period * m_override * m_safe_override_1 * m_safe_override_2;
    m_time        += period;
    std_msgs::Float64Ptr msg(new std_msgs::Float64());
    msg->data = m_scaled_time.toSec();
    m_joint_ctrl->publish("scaled_time_pub", msg);

    sensor_msgs::JointStatePtr js_msg(new sensor_msgs::JointState());
    js_msg->name = m_joint_ctrl->getJointNames();
    js_msg->position.resize(m_joint_ctrl->getJointNames().size());
    js_msg->velocity.resize(m_joint_ctrl->getJointNames().size());
    js_msg->effort.resize(m_joint_ctrl->getJointNames().size(), 0);
    js_msg->position      = getTrajectoryPoint().positions;
    js_msg->velocity      = getTrajectoryPoint().velocities;
    js_msg->header.stamp  = ros::Time::now();
    m_joint_ctrl->publish("target_pub", js_msg);
  }
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM("something wrong: " << e.what()
                     << "(function input: Time: " << time.toSec() << " Duration: " << period.toSec() << ")");
    return false;
  }
  return true;
}

template< class T>
void PrefilterController<T>::stopping(const ros::Time& time)
{
  CNR_TRACE_START(*m_joint_ctrl->logger(), "Stopping thor");
  if (m_gh)
  {
    m_gh->setCanceled();
  }
  joinActionServerThread();
  m_gh.reset();
  CNR_TRACE(*m_joint_ctrl->logger(), "Stopped thor successfully!");
}

template< class T>
void PrefilterController<T>::overrideCallback(const std_msgs::Int64ConstPtr& msg)
{
  double ovr;
  if (msg->data > 100)
    ovr = 1;
  else if (msg->data < 0)
    ovr = 0;
  else
    ovr = msg->data * 0.01;
  m_override = ovr;
}

template< class T>
void PrefilterController<T>::safeOverrideCallback_1(const std_msgs::Int64ConstPtr& msg)
{
  double ovr;
  if (msg->data > 100)
    ovr = 1;
  else if (msg->data < 0)
    ovr = 0;
  else
    ovr = msg->data * 0.01;
  m_safe_override_1 = ovr;
}

template< class T>
void PrefilterController<T>::safeOverrideCallback_2(const std_msgs::Int64ConstPtr& msg)
{
  double ovr;
  if (msg->data > 100)
    ovr = 1;
  else if (msg->data < 0)
    ovr = 0;
  else
    ovr = msg->data * 0.01;
  m_safe_override_2 = ovr;
}

template< class T>
void PrefilterController<T>::actionServerThread()
{
  CNR_DEBUG(*m_joint_ctrl->logger(), "START ACTION GOAL LOOPING");
  ros::WallRate lp(100);
  while (ros::ok())
  {
    lp.sleep();
    if (!m_gh)
    {
      CNR_ERROR(*m_joint_ctrl->logger(), "Goal handle is not initialized");
      break;
    }

    if ((m_preempted) 
    || (m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::RECALLED) 
    || (m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTED))
    {
      CNR_WARN(*m_joint_ctrl->logger(), "Action Server Thread Preempted");
      CNR_DEBUG(*m_joint_ctrl->logger(), "(m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTED)  = " << (int)(m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTED));
      CNR_DEBUG(*m_joint_ctrl->logger(), "m_preempted  = %d" << (int)m_preempted);
      CNR_DEBUG(*m_joint_ctrl->logger(), "(m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::RECALLED)  = " << (int)(m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::RECALLED));
      m_joint_ctrl->add_diagnostic_message((m_preempted ? "preempted" : "cancelled"), "INTERPOLATOR", "OK", true);

      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = 0;
      result.error_string = "preempted";
      m_gh->setRejected(result);
      CNR_DEBUG(*m_joint_ctrl->logger(), "Preempted old goal DONE");

      break;
    }

    if ((m_is_finished == 1) || (((m_scaled_time - m_thor_prefilter->trjTime()).toSec() > 0) && m_is_in_tolerance))
    {
      m_joint_ctrl->add_diagnostic_message("all is ok", "INTERPOLATOR", "OK", true);

      control_msgs::FollowJointTrajectoryResult result;
      m_gh->setSucceeded(result);

      break;
    }
    else if (m_is_finished == -2)
    {
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = -4;
      result.error_string = "Some problem occurs";

      m_joint_ctrl->add_diagnostic_message("Some problem occurs", "INTERPOLATOR", "ERROR", true);
      m_gh->setAborted(result);
      break;
    }

  }
  m_gh.reset();
  CNR_DEBUG(*m_joint_ctrl->logger(), "START ACTION GOAL END");

}

template< class T>
void PrefilterController<T>::actionGoalCallback(actionlib::ActionServer< control_msgs::FollowJointTrajectoryAction >::GoalHandle gh)
{
  CNR_TRACE_START(*m_joint_ctrl->logger());
  CNR_INFO(*m_joint_ctrl->logger(), "Received a goal");
  auto goal = gh.getGoal();

  boost::shared_ptr<actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle> current_gh;

  if (m_gh)
  {
    // PREEMPTED OLD GOAL
    CNR_INFO(*m_joint_ctrl->logger(), "preempting old goal");
    m_gh->setAborted();
    joinActionServerThread();

    CNR_INFO(*m_joint_ctrl->logger(), "Goal Stopped");

  }
  else
  {
    CNR_INFO(*m_joint_ctrl->logger(), "No goals running yet");
  }

  current_gh.reset(new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle(gh));
  m_gh = current_gh;
  unsigned  int nPnt = goal->trajectory.points.size();

  if (nPnt == 0)
  {
    ROS_DEBUG("TRAJECTORY WITH NO POINT");
    control_msgs::FollowJointTrajectoryResult result;
    m_gh->setAccepted();
    current_gh->setSucceeded(result);
    m_gh.reset();
    return;
  }

  trajectory_msgs::JointTrajectoryPtr trj(new trajectory_msgs::JointTrajectory());

  if (!trajectory_processing::sort_trajectory(m_joint_ctrl->getJointNames(), goal->trajectory, *trj))
  {
    CNR_ERROR(*m_joint_ctrl->logger(), "Names are different");
    m_gh->setAborted();
    joinActionServerThread();
    return;
  }

  m_mtx.lock();
  m_thor_prefilter->setTrajectory(trj);
  m_scaled_time = ros::Duration(0);
  m_time = ros::Duration(0);

  CNR_INFO(*m_joint_ctrl->logger(), "Starting managing new goal");
  m_is_finished = 0;
  m_mtx.unlock();
  m_gh->setAccepted();

  joinActionServerThread();

  m_as_thread = std::thread(&PrefilterController<T>::actionServerThread, this);

  CNR_RETURN_OK(*m_joint_ctrl->logger(), void() );
}

template< class T>
void PrefilterController<T>::actionCancelCallback(actionlib::ActionServer< control_msgs::FollowJointTrajectoryAction >::GoalHandle gh)
{
  CNR_TRACE_START(*m_joint_ctrl->logger());
  CNR_DEBUG(*m_joint_ctrl->logger(), "Cancel active goal Callback");
  if (m_gh)
  {
    m_gh->setCanceled();
    joinActionServerThread();
    m_gh.reset();

    m_mtx.lock();

    trajectory_msgs::JointTrajectoryPtr trj(new trajectory_msgs::JointTrajectory());
    trj->points.push_back(m_pnt);
    m_thor_prefilter->setTrajectory(trj);
    m_scaled_time = ros::Duration(0);
    m_time = ros::Duration(0);
    m_mtx.unlock();

  }
  else
  {
    CNR_WARN(*m_joint_ctrl->logger(), "No goal to cancel");
  }
  CNR_RETURN_OK(*m_joint_ctrl->logger(), void());
}


}

#endif  // THOR_PREFILTER_CONTROLLER_THOR_PREFILTER_CONTROLLER_IMPL_H
