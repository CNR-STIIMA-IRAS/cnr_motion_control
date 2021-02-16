cmake_minimum_required(VERSION 3.1)
project(cnr_follow_joint_trajectory_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  actionlib
  cnr_regulator_interface
  cnr_interpolator_interface
  cnr_controller_interface
  cnr_hardware_interface
  control_msgs
  name_sorting
  roscpp
  sensor_msgs
  trajectory_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cnr_follow_joint_trajectory_controller
  CATKIN_DEPENDS pluginlib actionlib cnr_regulator_interface cnr_interpolator_interface cnr_controller_interface cnr_hardware_interface
  control_msgs name_sorting roscpp sensor_msgs trajectory_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include ${catkin_INCLUDE_DIRS} )

add_library(${PROJECT_NAME}   src/${PROJECT_NAME}/cnr_follow_joint_trajectory_pos_controller.cpp
                              src/${PROJECT_NAME}/cnr_follow_joint_trajectory_posveleff_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -faligned-new
        $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)

if(${CMAKE_VERSION} VERSION_GREATER "3.16.0")
target_precompile_headers(${PROJECT_NAME} PUBLIC <mutex>
                                                <memory>
                                                <boost/shared_ptr.hpp>
                                                <ros/ros.h>
                                                <actionlib/server/action_server.h>
                                                <pluginlib/class_loader.h>
                                                <pluginlib/class_list_macros.h>
                                                <control_msgs/FollowJointTrajectoryAction.h>
                                                <controller_interface/controller_base.h>
                                                <name_sorting/sort_trajectories.h>
                                                <std_msgs/Int64.h>
                                                <std_msgs/Float64.h>
                                                <sensor_msgs/JointState.h>)
endif()

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cnr_follow_joint_trajectory_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
