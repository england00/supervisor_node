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
add_executable(supervisor_node src/supervisor_node/supervisor_node.cpp
        src/supervisor_node/header/supervisor_node_header.h
        src/supervisor_node/methods/methods.cpp
        src/supervisor_node/states/idle.h
        src/supervisor_node/states/manual.h
        src/supervisor_node/states/active.h
        src/supervisor_node/states/emergency_takeover.h
        src/supervisor_node/states/emergency_stop.h)
set(SOURCES src/supervisor_node/supervisor_node.cpp)
ament_target_dependencies(supervisor_node ${DEPENDENCIES}) # works only for packages found with 'find_package', links files included inside the script
install(TARGETS supervisor_node DESTINATION lib/${PROJECT_NAME})

# state selector node
add_executable(external_state_selector_node src/external_state_selector_node.cpp)
set(SOURCES src/external_state_selector_node.cpp)
ament_target_dependencies(external_state_selector_node ${DEPENDENCIES}) # works only for packages found with 'find_package', links files included inside the script
install(TARGETS external_state_selector_node DESTINATION lib/${PROJECT_NAME})

# state selector node
add_executable(pub_sub_simulator_node src/pub_sub_simulator_node.cpp)
set(SOURCES src/pub_sub_simulator_node.cpp)
ament_target_dependencies(pub_sub_simulator_node ${DEPENDENCIES}) # works only for packages found with 'find_package', links files included inside the script
install(TARGETS pub_sub_simulator_node DESTINATION lib/${PROJECT_NAME})

# install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_export_include_directories(include)
ament_export_libraries(${LIB})

ament_package()
