cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance

project(demo_mpc_vesc_mocap)
add_definitions(-std=c++11 -O3)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf2
  tf
)

catkin_package()

include_directories(/usr/local/include)
include_directories(include/Eigen-3.3)
include_directories(
  include
  
  ${catkin_INCLUDE_DIRS}
)

link_directories(/usr/local/lib)
add_executable(${PROJECT_NAME}_test_node src/main.cpp src/MPC.cpp)

target_link_libraries(${PROJECT_NAME}_test_node
   ${catkin_LIBRARIES}
    ipopt z ssl 
#    /usr/local/lib/libipopt.so.1
  #  /usr/local/lib/libcoinlapack.a
)

