cmake_minimum_required(VERSION 2.8.3)
project(odrive_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  hardware_interface
  controller_manager
  dynamic_reconfigure
  nav_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(
  config/Odrive.cfg
)

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
set(usb-1.0_LIBRARIES usb-1.0)
set(jsoncpp_LIBRARIES jsoncpp)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs
  DEPENDS ${rt_LIBRARIES} ${pthread_LIBRARIES} ${usb-1.0_LIBRARIES} ${jsoncpp_LIBRARIES}
)

include_directories(
 include
 include/odrive_driver
 include/ros_driver
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
  src/odrive_endpoint.cpp
  src/odrive_utils.cpp
  src/odrive_diff.cpp
  src/main.cpp
)
target_link_libraries(
  ${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
  ${usb-1.0_LIBRARIES} 
  ${jsoncpp_LIBRARIES}
)

add_dependencies(
  ${PROJECT_NAME} 
  ${PROJECT_NAME}_gencfg
)

