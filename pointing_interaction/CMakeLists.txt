cmake_minimum_required(VERSION 3.8)
project(pointing_interaction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(leap_gesture_interface REQUIRED)

# MoveIt & shapes
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometric_shapes REQUIRED)

# Include directories
include_directories(
  include
)

# Messages
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/PointingIntersection.msg
  DEPENDENCIES std_msgs geometry_msgs shape_msgs builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(lmc_intersection_publisher_node src/lmc_intersection_publisher_node.cpp)
ament_target_dependencies(lmc_intersection_publisher_node
  rclcpp geometry_msgs std_msgs visualization_msgs
  tf2_ros tf2_geometry_msgs leap_gesture_interface
  moveit_core moveit_ros_planning moveit_ros_planning_interface geometric_shapes builtin_interfaces
)
target_link_libraries(lmc_intersection_publisher_node ${cpp_typesupport_target})

# If your intersection library is a compiled target, link it here:
# target_link_libraries(lmc_intersection_publisher_node intersection_library_target)

install(TARGETS lmc_intersection_publisher_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
