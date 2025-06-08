cmake_minimum_required(VERSION 3.8)
project(task_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(task_msgs REQUIRED)
find_package(dialog_interfaces REQUIRED)
find_package(leap_gesture_interface REQUIRED)


add_executable(task_executor_node src/${PROJECT_NAME}/task_executor_node.cpp)
ament_target_dependencies(task_executor_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  task_msgs
  dialog_interfaces
  leap_gesture_interface
)

install(TARGETS
  task_executor_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
