/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef THOR_PREFILTER_CONTROLLER_THOR_PREFILTER_CONTROLLER_H
#define THOR_PREFILTER_CONTROLLER_THOR_PREFILTER_CONTROLLER_H


#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <thor_prefilter/thor_prefilter.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <thread>
#include <mutex>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <cnr_controller_interface/cnr_joint_controller_interface.h>

namespace thor
{

template< class T >
class PrefilterController
{
public:
  PrefilterController(cnr_controller_interface::JointController<T>* joint_ctrl);
  ~PrefilterController();

  virtual bool update(const ros::Time& time, const ros::Duration& period);
  virtual void starting(const ros::Time& time, const std::vector<double>& qini, const std::vector<double>& Dqini);
  virtual void stopping(const ros::Time& time);

  trajectory_msgs::JointTrajectoryPoint& getTrajectoryPoint()
  {
    return m_pnt;
  }

  ros::Duration getTime()
  {
    return m_time;
  }
  ros::Duration getScaledTime()
  {
    return m_scaled_time;
  }

protected:
  void actionServerThread();
  void actionGoalCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh);
  void actionCancelCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh);
  void overrideCallback(const std_msgs::Int64ConstPtr& msg);
  void safeOverrideCallback_1(const std_msgs::Int64ConstPtr& msg);
  void safeOverrideCallback_2(const std_msgs::Int64ConstPtr& msg);

  cnr_controller_interface::JointController<T>* m_joint_ctrl;
  
  bool   joinActionServerThread();
  double m_override;
  double m_safe_override_1;
  double m_safe_override_2;

  boost::shared_ptr<thor::ThorPrefilter> m_thor_prefilter;
  
  int m_is_finished;
  std::mutex m_mtx;

  boost::shared_ptr<actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>> m_as;
  boost::shared_ptr<actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle> m_gh;
  std::thread m_as_thread;
  bool m_preempted;
  
  trajectory_msgs::JointTrajectoryPoint m_pnt;

  ros::Duration m_scaled_time;
  ros::Duration m_time;
  bool m_is_in_tolerance;
  bool m_is_in_path_tolerance;
  double m_goal_tolerance;
  double m_path_tolerance;
};




class PrefilterPosVelEffController : public cnr_controller_interface::JointController<hardware_interface::PosVelEffJointInterface>
{
public:
  virtual bool doInit();
  virtual bool doUpdate(const ros::Time& time, const ros::Duration& period);
  virtual bool doStarting(const ros::Time& time);
  virtual bool doStopping(const ros::Time& time);

protected:

  std::shared_ptr< PrefilterController< hardware_interface::PosVelEffJointInterface > > m_ctrl;

};

class PrefilterPosController : public cnr_controller_interface::JointController<hardware_interface::PositionJointInterface>
{
public:
  virtual bool doInit();
  virtual bool doUpdate(const ros::Time& time, const ros::Duration& period);
  virtual bool doStarting(const ros::Time& time);
  virtual bool doStopping(const ros::Time& time);

protected:

  std::shared_ptr<PrefilterController< hardware_interface::PositionJointInterface > > m_ctrl;

};
}

#include <thor_prefilter_controller/thor_prefilter_controller_impl.h>





#endif  // THOR_PREFILTER_CONTROLLER_THOR_PREFILTER_CONTROLLER_H
