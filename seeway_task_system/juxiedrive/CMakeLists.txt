cmake_minimum_required(VERSION 3.8)
project(juxiedrive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(juxiedrive)

install(PROGRAMS
  juxiedrive/driver_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  examples/command_demo.py
  examples/custom_pdo_demo.py
  examples/pdo_mapping_demo.py
  DESTINATION lib/${PROJECT_NAME}/examples
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
