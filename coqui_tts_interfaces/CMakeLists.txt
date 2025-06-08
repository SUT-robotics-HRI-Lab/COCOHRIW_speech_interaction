cmake_minimum_required(VERSION 3.5)
project(coqui_tts_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Speak.srv"
  "msg/SpeakingStatus.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
