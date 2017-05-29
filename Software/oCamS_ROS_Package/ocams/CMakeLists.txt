cmake_minimum_required(VERSION 2.8.3)
project(ocams)

add_definitions(-std=c++11)
#set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    cv_bridge
    image_transport
    camera_info_manager
    sensor_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(ocams src/oCamS.cpp src/withrobot_camera.cpp src/withrobot_utility.cpp )
target_link_libraries(ocams ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} -lv4l2 -ludev)

