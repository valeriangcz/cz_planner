cmake_minimum_required(VERSION 2.8.3)

project(gcopter)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
# find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  pcl_conversions
  pcl_ros

  plan_env
  cv_bridge

  message_generation
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    # ${PCL_INCLUDE_DIRS}
    include
)
add_message_files(
  FILES
  traj.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp pcl_ros pcl_conversions plan_env message_runtime
)


add_executable(global_planning src/global_planning.cpp)

target_link_libraries(global_planning
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(for_test src/for_test.cpp)
target_link_libraries(for_test
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES})


  add_executable(cz_planner src/cz_planner.cpp)
  target_link_libraries(cz_planner
    ${OMPL_LIBRARIES}
    ${catkin_LIBRARIES})
  


  add_executable(trajectory_server src/trajectory_server.cpp)
  target_link_libraries(trajectory_server
    ${OMPL_LIBRARIES}
    ${catkin_LIBRARIES})