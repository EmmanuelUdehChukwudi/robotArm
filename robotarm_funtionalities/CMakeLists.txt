cmake_minimum_required(VERSION 3.8)
project(robotarm_funtionalities)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Suppress OpenGL policy warning
if(POLICY CMP0072)
  cmake_policy(SET CMP0072 OLD)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

add_executable(random_path src/execute_random_path.cpp)
add_executable(custom_path src/execute_custom_path.cpp)

ament_target_dependencies(random_path
  rclcpp
  std_msgs
  moveit_ros_planning_interface
  interactive_markers
  moveit_core
  moveit_ros_perception
)

ament_target_dependencies(custom_path
  rclcpp
  std_msgs
  moveit_ros_planning_interface
  interactive_markers
  moveit_core
  moveit_ros_perception
)

target_link_libraries(random_path
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
)

target_link_libraries(custom_path
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
)

install(TARGETS random_path custom_path
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
