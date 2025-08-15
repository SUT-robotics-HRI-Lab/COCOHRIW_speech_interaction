cmake_minimum_required(VERSION 3.8)
project(workspace_objects)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(resource_retriever REQUIRED)
# Optional, usually pulled via deps; uncomment if your toolchain compl

add_executable(workspace_objects_node src/workspace_objects_node.cpp)
ament_target_dependencies(workspace_objects_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  shape_msgs
  moveit_msgs
  geometric_shapes
  resource_retriever
)

target_link_libraries(workspace_objects_node
  ${YAML_CPP_LIBRARIES}
)


# If explicitly finding Eigen3 above, also:
# target_link_libraries(workspace_objects_node Eigen3::Eigen)

install(TARGETS workspace_objects_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
