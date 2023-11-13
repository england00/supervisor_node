cmake_minimum_required(VERSION 3.5)
project(supervisor_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiling options - Warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(yasmin_viewer REQUIRED)
find_package(simple_node REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)

# C++
include_directories(include)
include_directories(src)

set(LIB ${CMAKE_PROJECT_NAME}_lib)
set(DEPENDENCIES
  rclcpp
  std_msgs
  yasmin
  yasmin_ros
  yasmin_viewer
  simple_node
  nav_msgs
  example_interfaces
  action_tutorials_interfaces
)

# supervisor node
add_executable(supervisor_node src/supervisor_node.cpp)
set(SOURCES src/supervisor_node.cpp)
ament_target_dependencies(supervisor_node ${DEPENDENCIES}) # works only for packages found with 'find_package', links files included inside the script
install(TARGETS supervisor_node DESTINATION lib/${PROJECT_NAME})

# state selector node
add_executable(state_selector src/state_selector.cpp)
set(SOURCES src/state_selector.cpp)
ament_target_dependencies(state_selector ${DEPENDENCIES}) # works only for packages found with 'find_package', links files included inside the script
install(TARGETS state_selector DESTINATION lib/${PROJECT_NAME})

# state selector node
add_executable(failure_simulator src/failure_simulator.cpp)
set(SOURCES src/failure_simulator.cpp)
ament_target_dependencies(failure_simulator ${DEPENDENCIES}) # works only for packages found with 'find_package', links files included inside the script
install(TARGETS failure_simulator DESTINATION lib/${PROJECT_NAME})

# install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_export_include_directories(include)
ament_export_libraries(${LIB})

ament_package()
