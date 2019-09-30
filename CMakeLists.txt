cmake_minimum_required(VERSION 2.8.3)
project(atypical_planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

include_directories(${PROJECT_SOURCE_DIR}/include)
set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)


#ECBS
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/ecbs)
include_directories(${PROJECT_SOURCE_DIR}/third_party/ecbs/include)

#EIGEN
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

#OCTOMAP
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
add_definitions(-DOCTOMAP_NODEBUGOUT)

#ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  nav_msgs
  geometry_msgs
  octomap_ros
  octomap_msgs
)
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp roslib
)


#BUILD
add_executable(corridor_generator src/corridor_generator.cpp)
target_link_libraries(corridor_generator
  ${catkin_LIBRARIES}
  ${OCTOMAP_INCLUDE_DIRS}
)