cmake_minimum_required(VERSION 3.8)
project(erl1_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(erl1_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc aruco)

add_executable(aruco_detector src/aruco_detector.cpp)
target_compile_features(aruco_detector PUBLIC cxx_std_17)
ament_target_dependencies(aruco_detector
  rclcpp sensor_msgs std_msgs cv_bridge image_transport erl1_interfaces)
target_link_libraries(aruco_detector ${OpenCV_LIBS})

install(TARGETS aruco_detector DESTINATION lib/${PROJECT_NAME})

ament_package()