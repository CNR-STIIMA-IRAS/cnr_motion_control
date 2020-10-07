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
#ifndef CNR_FOLLOW_JOINT_TRAJECTORY_CONTROLLER_CNR_FOLLOW_JOINT_TRAJECTORY_POS_CONTROLLER__H
#define CNR_FOLLOW_JOINT_TRAJECTORY_CONTROLLER_CNR_FOLLOW_JOINT_TRAJECTORY_POS_CONTROLLER__H


#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <thread>
#include <mutex>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <hardware_interface/joint_command_interface.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <cnr_follow_joint_trajectory_controller/cnr_follow_joint_trajectory_controller.h>

namespace cnr
{
namespace control
{

class FollowJointTrajectoryPosController :
    public FollowJointTrajectoryController<hardware_interface::JointHandle, hardware_interface::PositionJointInterface>
{
public:

  bool doInit()
  {
    CNR_TRACE_START(*m_logger);
    this->setPriority(Q_PRIORITY);
    bool ret = FollowJointTrajectoryController<hardware_interface::JointHandle, 
                                               hardware_interface::PositionJointInterface>::doInit();
    CNR_RETURN_BOOL(*m_logger, ret);
  }
};

}  // namespace control
}  // namespace cnr

#endif  // CNR_FOLLOW_JOINT_TRAJECTORY_CONTROLLER_CNR_FOLLOW_JOINT_TRAJECTORY_POS_CONTROLLER__H
