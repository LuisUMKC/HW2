cmake_minimum_required(VERSION 3.5)

#change name of package to this
project(unmanned_systems_ros2_pkg)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies this will allow CPP and python
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Include Cpp "include" directory
include_directories(include)
# Create Cpp executable
add_executable(cpp_node src/cpp_node.cpp)
ament_target_dependencies(cpp_node rclcpp)
# Install Cpp executables
install(TARGETS
  cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
scripts/python_node.py
scripts/turtlebot_simple.py
scripts/logger.py
scripts/turtlebot_pid.py
scripts/turtlebot_wp.py
scripts/turtlebot_spawn.py
scripts/animated_pn.py
scripts/pn.py
scripts/logger_pursuit.py
scripts/evader.py
  # scripts/problem1.py
  # scripts/problem2.py
  # scripts/problem3.py
  # scripts/problem4.py
  # scripts/problem5.py
  scripts/Prob_1HW6.py
  scripts/Prob_2HW6.py
  scripts/Prob_3HW6.py
  scripts/Prob_4HW6.py
  scripts/Prob_4PurHW6.py
  scripts/Prob_5HW6.py
  scripts/Prob_6HW6.py
  scripts/Prob_7HW6.py
  scripts/Prob_8HW6.py
  scripts/Prob_8PurHW6.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

#launch
install(DIRECTORY
  launch
  # models
  # urdfros
  # worlds
  # assets
  DESTINATION share/${PROJECT_NAME}
)