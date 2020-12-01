cmake_minimum_required(VERSION 3.0.2)
project(safe_stop)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  carma_utils
  carma_wm
  cav_msgs
  cav_srvs
  roscpp
  trajectory_utils
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES safe_stop
   CATKIN_DEPENDS carma_utils cav_msgs cav_srvs roscpp carma_wm trajectory_utils
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

file(GLOB_RECURSE headers */*.hpp */*.h)

add_executable( ${PROJECT_NAME}
  ${headers}
  src/smoothing/CubicSpline.cpp
  src/main.cpp
  src/safe_stop_node.cpp
  src/safe_stop_arbitrator.cpp
  src/route_end.cpp)

add_library(safe_stop_library src/safe_stop_node.cpp src/route_end.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} safe_stop_library)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(safe_stop_library ${catkin_EXPORTED_TARGETS})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_node
  src/safe_stop_node.cpp
)


## Install C++
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Other Resources
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
catkin_add_gmock(${PROJECT_NAME}-test
  test/test_route_end.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(${PROJECT_NAME}-test safe_stop_library ${catkin_LIBRARIES})

