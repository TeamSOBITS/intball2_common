cmake_minimum_required(VERSION 3.8)
project(ib2_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Collect all interface files (msg/srv/action) from current source dir.
file(GLOB MSG_FILES    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")
file(GLOB SRV_FILES    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "srv/*.srv")
file(GLOB ACTION_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "action/*.action")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  ${ACTION_FILES}
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
    action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()