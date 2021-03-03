cmake_minimum_required(VERSION 3.0.2)
project(lookahead_prefilter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  rosdyn_core
  cnr_interpolator_interface
  cnr_logger
  roscpp
)

find_package(Eigen3 REQUIRED)

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rosdyn_core cnr_interpolator_interface cnr_logger roscpp
  DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}   src/${PROJECT_NAME}/cartesian_lookahead_prefilter.cpp )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -faligned-new
        $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)

if(${CMAKE_VERSION} VERSION_GREATER "3.16.0")
target_precompile_headers(${PROJECT_NAME} PUBLIC  <Eigen/Dense>
                                                  <vector>
                                                  <Eigen/StdVector>
                                                  <ros/node_handle.h>
                                                  <trajectory_msgs/JointTrajectory.h>
                                                  <rosdyn_core/spacevect_algebra.h>)
endif()

#############
## Install ##
#############

# Mark libraries for installation
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

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(cnr_fake_hardware_interface REQUIRED)
  find_package(cnr_regulator_interface REQUIRED)

  add_rostest_gtest(${PROJECT_NAME}_test test/test.launch test/test.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
                                ${cnr_regulator_interface_INCLUDE_DIRS} ${cnr_fake_hardware_interface_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME}_test
                            ${catkin_LIBRARIES} ${cnr_fake_hardware_interface_LIBRARIES}
                              ${cnr_regulator_interface_LIBRARIES} ${rostest_LIBRARIES} )
  target_compile_options(${PROJECT_NAME}_test PUBLIC $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)
  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage_report
      DEPENDENCIES ${PROJECT_NAME}_test
    )
  endif()
endif()

