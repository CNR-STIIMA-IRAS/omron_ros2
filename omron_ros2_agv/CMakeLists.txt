cmake_minimum_required(VERSION 3.8)
project(omron_ros2_agv)

add_compile_options(-std=c++17)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)

set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(omron_ros2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)


add_executable(omron_ros2_agv_node src/ros2_omron_agv_node.cpp)

target_include_directories(omron_ros2_agv_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/local/include/Aria/)

target_compile_features(omron_ros2_agv_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


target_link_libraries(omron_ros2_agv_node
  Aria
  ArNetworking
)

ament_target_dependencies(omron_ros2_agv_node 
                            ${${PROJECT_NAME}_EXPORTED_TARGETS}
                            rclcpp 
                            tf2_ros 
                            sensor_msgs 
                            omron_ros2_msgs
                            geometry_msgs 
                            angles 
                            nav_msgs
                            tf2_geometry_msgs)





install(
  TARGETS omron_ros2_agv_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

install(
  DIRECTORY launch mesh urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
