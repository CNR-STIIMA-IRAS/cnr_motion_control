cmake_minimum_required(VERSION 3.1)

project(thor_prefilter_controller)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  cnr_controller_interface roscpp thor_prefilter  cnr_hardware_interface name_sorting actionlib trajectory_msgs control_msgs sensor_msgs moveit_planning_helper
)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES thor_prefilter_controller
  CATKIN_DEPENDS cnr_controller_interface name_sorting roscpp thor_prefilter  hardware_interface actionlib trajectory_msgs control_msgs sensor_msgs moveit_planning_helper
  DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}  src/${PROJECT_NAME}/thor_prefilter_controller.cpp )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)

# add_executable(${PROJECT_NAME}_node src/thor_prefilter_controller_node.cpp)
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME})
