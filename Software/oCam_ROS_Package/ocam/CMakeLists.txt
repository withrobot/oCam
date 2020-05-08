cmake_minimum_required(VERSION 2.8.3)
project(ocam)

add_definitions(-std=c++11)
#set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
  dynamic_reconfigure
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/cam.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    cv_bridge
    image_transport
    camera_info_manager
    sensor_msgs
    dynamic_reconfigure
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(ocam src/oCam.cpp src/withrobot_camera.cpp src/withrobot_utility.cpp)
target_link_libraries(ocam ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} -lv4l2 -ludev)
add_dependencies(ocam ${PROJECT_NAME}_gencfg)
