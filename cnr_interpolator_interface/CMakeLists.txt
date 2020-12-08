cmake_minimum_required(VERSION 3.0.2)
project(cnr_interpolator_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  cnr_logger
  roscpp
  trajectory_msgs
  rosdyn_core
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES cnr_interpolator_interface
 CATKIN_DEPENDS cnr_logger roscpp trajectory_msgs rosdyn_core
 DEPENDS Eigen3
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} )

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/internal/cnr_interpolator_base.cpp )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -faligned-new $<$<CONFIG:RELEASE>:-Ofast>)
#target_compile_definitions(${PROJECT_NAME} PUBLIC -DEIGEN_DONT_VECTORIZE=1)
target_precompile_headers(${PROJECT_NAME} PUBLIC <Eigen/Dense>
                                                  <trajectory_msgs/JointTrajectory.h>
                                                  <ros/node_handle.h>)

#############
## Install ##
#############

# Mark libraries for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cnr_interpolator_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
