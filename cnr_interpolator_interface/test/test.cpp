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

#include <iostream>
#include <ros/ros.h>
#include <cnr_logger/cnr_logger.h>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <rosdyn_core/primitives.h>
#include <cnr_interpolator_interface/cnr_interpolator_interface.h>

std::shared_ptr<cnr_logger::TraceLogger> logger;

rosdyn::ChainPtr kin;

std::shared_ptr<ros::NodeHandle> root_nh;
std::shared_ptr<ros::NodeHandle> robot_nh;
std::shared_ptr<ros::NodeHandle> joint_interp_nh;
std::shared_ptr<cnr::control::JointInterpolatorInterface> joint_interp;
std::shared_ptr<cnr::control::CartesianInterpolatorInterface> cart_interp;

cnr::control::CartesianTrajectoryPtr trj;
cnr::control::CartesianInterpolatorPointPtr pnt;

// Declare a test
TEST(TestSuite, loggerInit)
{
  std::string path1 = "/logger";

  EXPECT_NO_FATAL_FAILURE(logger.reset(new cnr_logger::TraceLogger("log1",path1 )));
  EXPECT_FALSE(logger->init("log1", path1, false, false));
}

TEST(TestSuite, chainInit)
{
  ASSERT_NO_THROW(kin.reset(new rosdyn::Chain()););

  urdf::Model model;
  model.initParam("robot_description");

  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;

  rosdyn::Chain chain;
  std::string error;
  rosdyn::LinkPtr root_link(new rosdyn::Link());
  root_link->fromUrdf(model.root_link_);

  std::stringstream report;
  int ret = -1;
  ASSERT_NO_THROW(kin->init(error, root_link, "base_link","tool0") ;);

  if(ret==-1)
    std::cerr << "ERROR: " << report.str() << std::endl;

  if(ret==0)
    std::cerr << "WARNING: " << report.str() << std::endl;
}

TEST(TestSuite, params)
{
  EXPECT_NO_FATAL_FAILURE(trj.reset(new cnr::control::CartesianTrajectory() ));
  EXPECT_NO_FATAL_FAILURE(pnt.reset(new cnr::control::CartesianInterpolatorPoint()) );
  pnt->time_from_start = ros::Duration(0.0);
  pnt->twist  = Eigen::Vector6d::Zero();
  pnt->twistd = Eigen::Vector6d::Zero();
  pnt->x      = Eigen::Affine3d::Identity();
  ASSERT_TRUE(trj->append( pnt ));
}



// Declare a test
TEST(TestSuite, jointConstructor)
{
  EXPECT_NO_FATAL_FAILURE(joint_interp.reset(new cnr::control::JointInterpolatorInterface()));
  EXPECT_TRUE(joint_interp->initialize(logger, *joint_interp_nh));
}


TEST(TestSuite, cartConstructor)
{
  EXPECT_NO_FATAL_FAILURE(cart_interp.reset(new cnr::control::CartesianInterpolatorInterface()));
  EXPECT_TRUE(cart_interp->initialize(logger, *joint_interp_nh));
}

TEST(TestSuite, initParams)
{
  ASSERT_TRUE(cart_interp->setTrajectory(trj));
}
TEST(TestSuite, Desctructor)
{
  EXPECT_NO_FATAL_FAILURE(joint_interp.reset());
  EXPECT_NO_FATAL_FAILURE(cart_interp.reset());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "cnr_logger_tester");
  root_nh  .reset(new ros::NodeHandle("/"));
  robot_nh .reset(new ros::NodeHandle("/ur10_hw"));
  joint_interp_nh  .reset(new ros::NodeHandle("/ur10_hw/fake_controller"));
  return RUN_ALL_TESTS();
}
