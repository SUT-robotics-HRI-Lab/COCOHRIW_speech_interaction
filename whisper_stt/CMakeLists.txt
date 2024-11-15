cmake_minimum_required(VERSION 3.0.2)
project(whisper_stt)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Add these lines to use setup.py
catkin_python_setup()

## Declare Python scripts for installation
## In this case, install your transcription_node.py script
catkin_install_python(PROGRAMS
  src/transcription_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Add any additional dependencies
# Uncomment below if you need to specify additional libraries
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
