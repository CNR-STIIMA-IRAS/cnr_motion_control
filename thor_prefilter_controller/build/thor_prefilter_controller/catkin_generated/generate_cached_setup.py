# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/melodic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/melodic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/wsl/ctrl_ws/install/ur_identification;/home/wsl/ctrl_ws/install/ur_hardware_interface;/home/wsl/ctrl_ws/install/dimostratore_moveit_config;/home/wsl/ctrl_ws/install/dimostratore_descriptions;/home/wsl/ctrl_ws/install/ur_description;/home/wsl/ctrl_ws/install/ur_app;/home/wsl/ctrl_ws/install/dimostratore_configurations;/home/wsl/ctrl_ws/install/thor_prefilter_controller;/home/wsl/ctrl_ws/install/thor_prefilter;/home/wsl/ctrl_ws/install/tf_helper;/home/wsl/ctrl_ws/install/robots_teleop_gui;/home/wsl/ctrl_ws/install/joint_test_suite;/home/wsl/ctrl_ws/install/cnr_velocity_to_torque_controller;/home/wsl/ctrl_ws/install/cnr_topics_hardware_interface;/home/wsl/ctrl_ws/install/cnr_topic_hardware_interface;/home/wsl/ctrl_ws/install/cnr_position_to_velocity_controller;/home/wsl/ctrl_ws/install/cnr_open_loop_position_controller;/home/wsl/ctrl_ws/install/cnr_joint_teleop_controller;/home/wsl/ctrl_ws/install/cnr_joint_state_publisher;/home/wsl/ctrl_ws/install/cnr_configuration_manager;/home/wsl/ctrl_ws/install/subscription_notifier;/home/wsl/ctrl_ws/install/stiima_lab_descriptions;/home/wsl/ctrl_ws/install/si_utils;/home/wsl/ctrl_ws/install/rosdyn_gui;/home/wsl/ctrl_ws/install/moveit_planning_helper;/home/wsl/ctrl_ws/install/eigen_state_space_systems;/home/wsl/ctrl_ws/install/eigen_matrix_utils;/home/wsl/ctrl_ws/install/rosparam_utilities;/home/wsl/ctrl_ws/install/rosdyn_identification_msgs;/home/wsl/ctrl_ws/install/robotiq_modbus_tcp;/home/wsl/ctrl_ws/install/robotiq_modbus_rtu;/home/wsl/ctrl_ws/install/robotiq_ft_sensor;/home/wsl/ctrl_ws/install/robotiq_ethercat;/home/wsl/ctrl_ws/install/robotiq_85_msgs;/home/wsl/ctrl_ws/install/robotiq_85_moveit_config;/home/wsl/ctrl_ws/install/robotiq_85_gazebo;/home/wsl/ctrl_ws/install/robotiq_85_description;/home/wsl/ctrl_ws/install/robotiq_3f_gripper_articulated_gazebo;/home/wsl/ctrl_ws/install/robotiq_3f_gripper_visualization;/home/wsl/ctrl_ws/install/robotiq_3f_gripper_articulated_gazebo_plugins;/home/wsl/ctrl_ws/install/robotiq_3f_gripper_articulated_msgs;/home/wsl/ctrl_ws/install/robotiq_2f_c2_gripper_visualization;/home/wsl/ctrl_ws/install/robotiq_2f_85_gripper_visualization;/home/wsl/ctrl_ws/install/robotiq_2f_140_gripper_visualization;/home/wsl/ctrl_ws/install/roboticsgroup_gazebo_plugins;/home/wsl/ctrl_ws/install/cnr_fake_hardware_interface;/home/wsl/ctrl_ws/install/cnr_hardware_nodelet_interface;/home/wsl/ctrl_ws/install/cnr_hardware_interface;/home/wsl/ctrl_ws/install/cnr_controller_manager_interface;/home/wsl/ctrl_ws/install/cnr_controller_interface;/home/wsl/ctrl_ws/install/realtime_utilities;/home/wsl/ctrl_ws/install/name_sorting;/home/wsl/ctrl_ws/install/dimostratore_sensors;/home/wsl/ctrl_ws/install/ddynamic_reconfigure_python;/home/wsl/ctrl_ws/install/ddynamic_reconfigure;/home/wsl/ctrl_ws/install/configuration_gui;/home/wsl/ctrl_ws/install/configuration_msgs;/home/wsl/ctrl_ws/install/cnr_logger;/home/wsl/ctrl_ws/install/binary_logger;/home/wsl/ctrl_ws/install/additional_sensor_msgs;/opt/ros/melodic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/wsl/ctrl_ws/src/thor_prefilter_controller/build/thor_prefilter_controller/devel/env.sh')

output_filename = '/home/wsl/ctrl_ws/src/thor_prefilter_controller/build/thor_prefilter_controller/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
