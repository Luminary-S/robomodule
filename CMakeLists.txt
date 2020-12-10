cmake_minimum_required(VERSION 3.0.2)
project(robomodule)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  control_msgs
  tf
  serial
)

find_package(Threads)
find_package(yaml-cpp REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sensor_startup
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(include lib ${catkin_INCLUDE_DIRS})
link_directories(lib)

# define the library used in sensor communication
add_library(robo_can_app
  # src/imu_reader.cpp
  # src/motor_reader.cpp
  src/kinco/can_application.cpp
  # src/kinco/driver_controller.cpp
  # src/kinco/driver_controller_ros.cpp
)
target_link_libraries(robo_can_app
  yaml-cpp
  # controlcan
  usbcan
  ${catkin_LIBRARIES}
)


add_executable(can_test 
  src/test_can.cpp
  src/robomoduleCan.cpp)
target_link_libraries(can_test
 robo_can_app 
 usbcan 
 ${catkin_LIBRARIES}
)

# add_executable(robomodule_can src/robomoduleCan.cpp)
# target_link_libraries(robomodule_can  
#   robo_can_app 
#   # controlcan
#   usbcan  
#   )

# add_executable(robomodule_node 
#   src/robomodule_node.cpp
#   src/robomoduleCan.cpp)
# target_link_libraries(robomodule_node  
#   robo_can_app 
#   usbcan  
# )

add_executable(dcmotor_node 
  src/canbus_motor_node.cpp
  src/robomoduleCan.cpp)
target_link_libraries(dcmotor_node  
  robo_can_app 
  usbcan  
)