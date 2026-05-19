cmake_minimum_required(VERSION 3.8)
project(juxiedrive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointCommand.msg"
  "msg/JointStatus.msg"
  "srv/ExecuteCommand.srv"
  "srv/ReadObject.srv"
  "srv/WriteObject.srv"
  DEPENDENCIES builtin_interfaces
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  juxiedrive/juxiedrive_node.py
  examples/frame_preview_example.py
  examples/control_sequence_example.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  examples
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
