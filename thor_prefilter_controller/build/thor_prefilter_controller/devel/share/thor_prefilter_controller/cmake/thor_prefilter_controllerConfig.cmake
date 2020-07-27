# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(thor_prefilter_controller_CONFIG_INCLUDED)
  return()
endif()
set(thor_prefilter_controller_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(thor_prefilter_controller_SOURCE_PREFIX /home/wsl/ctrl_ws/src/thor_prefilter_controller)
  set(thor_prefilter_controller_DEVEL_PREFIX /home/wsl/ctrl_ws/src/thor_prefilter_controller/build/thor_prefilter_controller/devel)
  set(thor_prefilter_controller_INSTALL_PREFIX "")
  set(thor_prefilter_controller_PREFIX ${thor_prefilter_controller_DEVEL_PREFIX})
else()
  set(thor_prefilter_controller_SOURCE_PREFIX "")
  set(thor_prefilter_controller_DEVEL_PREFIX "")
  set(thor_prefilter_controller_INSTALL_PREFIX /home/wsl/ctrl_ws/src/thor_prefilter_controller/install/thor_prefilter_controller)
  set(thor_prefilter_controller_PREFIX ${thor_prefilter_controller_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'thor_prefilter_controller' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(thor_prefilter_controller_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/home/wsl/ctrl_ws/src/thor_prefilter_controller/include " STREQUAL " ")
  set(thor_prefilter_controller_INCLUDE_DIRS "")
  set(_include_dirs "/home/wsl/ctrl_ws/src/thor_prefilter_controller/include")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'Manuel Beschi <manuel.beschi@stiima.cnr.it>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${thor_prefilter_controller_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'thor_prefilter_controller' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'thor_prefilter_controller' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/wsl/ctrl_ws/src/thor_prefilter_controller/${idir}'.  ${_report}")
    endif()
    _list_append_unique(thor_prefilter_controller_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "thor_prefilter_controller")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND thor_prefilter_controller_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND thor_prefilter_controller_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT thor_prefilter_controller_NUM_DUMMY_TARGETS)
      set(thor_prefilter_controller_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::thor_prefilter_controller::wrapped-linker-option${thor_prefilter_controller_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR thor_prefilter_controller_NUM_DUMMY_TARGETS "${thor_prefilter_controller_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::thor_prefilter_controller::wrapped-linker-option${thor_prefilter_controller_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND thor_prefilter_controller_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND thor_prefilter_controller_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND thor_prefilter_controller_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/wsl/ctrl_ws/src/thor_prefilter_controller/build/thor_prefilter_controller/devel/lib;/home/wsl/ctrl_ws/install/ur_identification/lib;/home/wsl/ctrl_ws/install/ur_hardware_interface/lib;/home/wsl/ctrl_ws/install/dimostratore_moveit_config/lib;/home/wsl/ctrl_ws/install/dimostratore_descriptions/lib;/home/wsl/ctrl_ws/install/ur_description/lib;/home/wsl/ctrl_ws/install/ur_app/lib;/home/wsl/ctrl_ws/install/dimostratore_configurations/lib;/home/wsl/ctrl_ws/install/thor_prefilter_controller/lib;/home/wsl/ctrl_ws/install/thor_prefilter/lib;/home/wsl/ctrl_ws/install/tf_helper/lib;/home/wsl/ctrl_ws/install/robots_teleop_gui/lib;/home/wsl/ctrl_ws/install/joint_test_suite/lib;/home/wsl/ctrl_ws/install/cnr_velocity_to_torque_controller/lib;/home/wsl/ctrl_ws/install/cnr_topics_hardware_interface/lib;/home/wsl/ctrl_ws/install/cnr_topic_hardware_interface/lib;/home/wsl/ctrl_ws/install/cnr_position_to_velocity_controller/lib;/home/wsl/ctrl_ws/install/cnr_open_loop_position_controller/lib;/home/wsl/ctrl_ws/install/cnr_joint_teleop_controller/lib;/home/wsl/ctrl_ws/install/cnr_joint_state_publisher/lib;/home/wsl/ctrl_ws/install/cnr_configuration_manager/lib;/home/wsl/ctrl_ws/install/subscription_notifier/lib;/home/wsl/ctrl_ws/install/stiima_lab_descriptions/lib;/home/wsl/ctrl_ws/install/si_utils/lib;/home/wsl/ctrl_ws/install/rosdyn_gui/lib;/home/wsl/ctrl_ws/install/moveit_planning_helper/lib;/home/wsl/ctrl_ws/install/eigen_state_space_systems/lib;/home/wsl/ctrl_ws/install/eigen_matrix_utils/lib;/home/wsl/ctrl_ws/install/rosparam_utilities/lib;/home/wsl/ctrl_ws/install/rosdyn_identification_msgs/lib;/home/wsl/ctrl_ws/install/robotiq_modbus_tcp/lib;/home/wsl/ctrl_ws/install/robotiq_modbus_rtu/lib;/home/wsl/ctrl_ws/install/robotiq_ft_sensor/lib;/home/wsl/ctrl_ws/install/robotiq_ethercat/lib;/home/wsl/ctrl_ws/install/robotiq_85_msgs/lib;/home/wsl/ctrl_ws/install/robotiq_85_moveit_config/lib;/home/wsl/ctrl_ws/install/robotiq_85_gazebo/lib;/home/wsl/ctrl_ws/install/robotiq_85_description/lib;/home/wsl/ctrl_ws/install/robotiq_3f_gripper_articulated_gazebo/lib;/home/wsl/ctrl_ws/install/robotiq_3f_gripper_visualization/lib;/home/wsl/ctrl_ws/install/robotiq_3f_gripper_articulated_gazebo_plugins/lib;/home/wsl/ctrl_ws/install/robotiq_3f_gripper_articulated_msgs/lib;/home/wsl/ctrl_ws/install/robotiq_2f_c2_gripper_visualization/lib;/home/wsl/ctrl_ws/install/robotiq_2f_85_gripper_visualization/lib;/home/wsl/ctrl_ws/install/robotiq_2f_140_gripper_visualization/lib;/home/wsl/ctrl_ws/install/roboticsgroup_gazebo_plugins/lib;/home/wsl/ctrl_ws/install/cnr_fake_hardware_interface/lib;/home/wsl/ctrl_ws/install/cnr_hardware_nodelet_interface/lib;/home/wsl/ctrl_ws/install/cnr_hardware_interface/lib;/home/wsl/ctrl_ws/install/cnr_controller_manager_interface/lib;/home/wsl/ctrl_ws/install/cnr_controller_interface/lib;/home/wsl/ctrl_ws/install/realtime_utilities/lib;/home/wsl/ctrl_ws/install/name_sorting/lib;/home/wsl/ctrl_ws/install/dimostratore_sensors/lib;/home/wsl/ctrl_ws/install/ddynamic_reconfigure_python/lib;/home/wsl/ctrl_ws/install/ddynamic_reconfigure/lib;/home/wsl/ctrl_ws/install/configuration_gui/lib;/home/wsl/ctrl_ws/install/configuration_msgs/lib;/home/wsl/ctrl_ws/install/cnr_logger/lib;/home/wsl/ctrl_ws/install/binary_logger/lib;/home/wsl/ctrl_ws/install/additional_sensor_msgs/lib;/opt/ros/melodic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(thor_prefilter_controller_LIBRARY_DIRS ${lib_path})
      list(APPEND thor_prefilter_controller_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'thor_prefilter_controller'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND thor_prefilter_controller_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(thor_prefilter_controller_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${thor_prefilter_controller_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "cnr_controller_interface;roscpp;thor_prefilter;hardware_interface;actionlib;trajectory_msgs;control_msgs;sensor_msgs;moveit_planning_helper")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 thor_prefilter_controller_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${thor_prefilter_controller_dep}_FOUND)
      find_package(${thor_prefilter_controller_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${thor_prefilter_controller_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(thor_prefilter_controller_INCLUDE_DIRS ${${thor_prefilter_controller_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(thor_prefilter_controller_LIBRARIES ${thor_prefilter_controller_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${thor_prefilter_controller_dep}_LIBRARIES})
  _list_append_deduplicate(thor_prefilter_controller_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(thor_prefilter_controller_LIBRARIES ${thor_prefilter_controller_LIBRARIES})

  _list_append_unique(thor_prefilter_controller_LIBRARY_DIRS ${${thor_prefilter_controller_dep}_LIBRARY_DIRS})
  list(APPEND thor_prefilter_controller_EXPORTED_TARGETS ${${thor_prefilter_controller_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${thor_prefilter_controller_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
