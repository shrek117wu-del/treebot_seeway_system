cmake_minimum_required(VERSION 3.8)
project(seeway_task_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(seeway_task_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Install Python modules
ament_python_install_package(seeway_task_manager)

# Install executable scripts
install(PROGRAMS
  seeway_task_manager/task_manager_node.py
  seeway_task_manager/xbox_controller_node.py
  seeway_task_manager/nav2_client_node.py
  seeway_task_manager/task_scheduler_node.py
  seeway_task_manager/examples/task_publisher_example.py
  seeway_task_manager/examples/joy_visualizer_example.py
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
