cmake_minimum_required(VERSION 3.8)
project(lz_omni_chassis_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python package
ament_python_install_package(lz_omni_chassis_driver)

# Install executable entry points
install(PROGRAMS
  lz_omni_chassis_driver/chassis_driver_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install examples
install(PROGRAMS
  examples/send_example.py
  DESTINATION lib/${PROJECT_NAME}/examples
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_chassis_driver test/test_chassis_driver.py)
endif()

ament_package()
