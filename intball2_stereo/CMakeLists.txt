cmake_minimum_required(VERSION 3.16)
project(intball2_stereo VERSION 0.1.0 LANGUAGES CXX)

option(INTBALL2_STEREO_BUILD_ROS "Build the ROS adapters and stereo node with ament" ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
include(CTest)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)

add_library(intball2_stereo_core SHARED
  src/core/stereo_config.cpp
  src/core/camera_model.cpp
  src/core/image_preprocessor.cpp
  src/core/sgbm_matcher.cpp
  src/core/depth_converter.cpp
  src/core/depth_refiner.cpp
  src/core/stereo_processor.cpp
  src/core/pointcloud_generator.cpp)
target_include_directories(intball2_stereo_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(intball2_stereo_core PUBLIC ${OpenCV_LIBS})
target_compile_features(intball2_stereo_core PUBLIC cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(intball2_stereo_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

set(package_targets intball2_stereo_core)
if(INTBALL2_STEREO_BUILD_ROS)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rcl_interfaces REQUIRED)
  find_package(rmw REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(Threads REQUIRED)
  add_library(intball2_stereo_ros SHARED
    src/ros/stereo_parameters.cpp src/ros/stereo_messages.cpp
    src/ros/stereo_input.cpp src/ros/stereo_worker.cpp src/ros/stereo_node.cpp
    src/ros/stereo_diagnostics.cpp src/ros/stereo_timing.cpp)
  target_include_directories(intball2_stereo_ros PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(intball2_stereo_ros PUBLIC intball2_stereo_core Threads::Threads)
  ament_target_dependencies(intball2_stereo_ros PUBLIC rclcpp rcl_interfaces rmw sensor_msgs
    diagnostic_msgs)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(intball2_stereo_ros PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  list(APPEND package_targets intball2_stereo_ros)
  add_executable(stereo_node src/ros/stereo_node_main.cpp)
  target_link_libraries(stereo_node PRIVATE intball2_stereo_ros)
  install(TARGETS stereo_node DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS ${package_targets} EXPORT intball2_stereo_targets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/intball2_stereo/core DESTINATION include/intball2_stereo)
if(INTBALL2_STEREO_BUILD_ROS)
  install(DIRECTORY include/intball2_stereo/ros DESTINATION include/intball2_stereo)
endif()
install(DIRECTORY config images DESTINATION share/${PROJECT_NAME})
if(INTBALL2_STEREO_BUILD_ROS)
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()
install(FILES README.md README.en.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(replay_core test/manual/replay_core.cpp)
  target_link_libraries(replay_core PRIVATE intball2_stereo_core ${CMAKE_DL_LIBS})
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(replay_core PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME depth_comparison COMMAND ${Python3_EXECUTABLE} -m unittest discover
    -s ${CMAKE_CURRENT_SOURCE_DIR}/test -p test_depth_comparison.py)
  add_test(NAME compare_sim_depth COMMAND ${Python3_EXECUTABLE} -m unittest discover
    -s ${CMAKE_CURRENT_SOURCE_DIR}/test/manual -p test_compare_sim_depth.py)
  add_test(NAME input_capture COMMAND ${Python3_EXECUTABLE} -m unittest discover
    -s ${CMAKE_CURRENT_SOURCE_DIR}/test/manual -p test_input_capture.py)
  add_test(NAME replay_capture COMMAND ${Python3_EXECUTABLE} -m unittest discover
    -s ${CMAKE_CURRENT_SOURCE_DIR}/test/manual -p test_replay_capture.py)
  add_test(NAME replay_precision COMMAND ${Python3_EXECUTABLE} -m unittest discover
    -s ${CMAKE_CURRENT_SOURCE_DIR}/test/manual -p test_compare_replay_precision.py)
  add_test(NAME refine_replay_precision COMMAND ${Python3_EXECUTABLE} -m unittest discover
    -s ${CMAKE_CURRENT_SOURCE_DIR}/test/manual -p test_refine_replay_precision.py)
  add_test(NAME mesh_reference COMMAND ${Python3_EXECUTABLE} -m unittest discover
    -s ${CMAKE_CURRENT_SOURCE_DIR}/test/manual -p test_mesh_reference.py)
  add_test(NAME bridge_transport_probe COMMAND ${Python3_EXECUTABLE} -m unittest discover
    -s ${CMAKE_CURRENT_SOURCE_DIR}/test/manual -p test_bridge_transport_probe.py)
  add_test(NAME analyze_depth_edges COMMAND ${Python3_EXECUTABLE} -m unittest discover
    -s ${CMAKE_CURRENT_SOURCE_DIR}/test/manual -p test_analyze_depth_edges.py)
  add_test(NAME stereo_image_proc_depth_adapter COMMAND ${Python3_EXECUTABLE} -m unittest discover
    -s ${CMAKE_CURRENT_SOURCE_DIR}/test -p test_stereo_image_proc_depth_adapter.py)
  add_test(NAME summarize_stereo_image_proc_comparison COMMAND ${Python3_EXECUTABLE} -m unittest discover
    -s ${CMAKE_CURRENT_SOURCE_DIR}/test/manual -p test_summarize_stereo_image_proc_comparison.py)
  set_tests_properties(depth_comparison compare_sim_depth input_capture replay_capture
    replay_precision refine_replay_precision mesh_reference bridge_transport_probe analyze_depth_edges stereo_image_proc_depth_adapter summarize_stereo_image_proc_comparison PROPERTIES
    ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
  add_executable(test_core_contracts test/core/test_core_contracts.cpp)
  target_link_libraries(test_core_contracts PRIVATE intball2_stereo_core)
  add_test(NAME core_contracts COMMAND test_core_contracts)
  find_package(GTest REQUIRED)
  foreach(test_name IN ITEMS calibration_depth stereo_processing pointcloud depth_refiner matching_roi)
    add_executable(test_${test_name} test/core/test_${test_name}.cpp test/core/stereo_test_main.cpp)
    target_link_libraries(test_${test_name} PRIVATE intball2_stereo_core GTest::gtest)
    add_test(NAME ${test_name} COMMAND test_${test_name})
  endforeach()
  if(INTBALL2_STEREO_BUILD_ROS)
    find_package(stereo_image_proc QUIET)
    if(stereo_image_proc_FOUND)
      add_executable(replay_stereo_image_proc test/manual/replay_stereo_image_proc.cpp)
      target_link_libraries(replay_stereo_image_proc PRIVATE ${OpenCV_LIBS} ${CMAKE_DL_LIBS})
      ament_target_dependencies(replay_stereo_image_proc PUBLIC stereo_image_proc)
      target_compile_options(replay_stereo_image_proc PRIVATE -Wall -Wextra -Wpedantic)
      add_test(NAME stereo_image_proc_replay_contract COMMAND replay_stereo_image_proc --self-test)
    endif()
    find_package(rcl_yaml_param_parser REQUIRED)
    find_package(rcutils REQUIRED)
    add_executable(test_parameter_contracts test/ros/test_parameter_contracts.cpp)
    target_link_libraries(test_parameter_contracts PRIVATE intball2_stereo_ros)
    target_compile_definitions(test_parameter_contracts PRIVATE
      STEREO_PARAMS_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/stereo_params.yaml")
    ament_target_dependencies(test_parameter_contracts PUBLIC rclcpp rcl_yaml_param_parser rcutils)
    add_test(NAME parameter_contracts COMMAND test_parameter_contracts)
    add_executable(test_ros_pipeline test/ros/test_ros_pipeline.cpp test/core/stereo_test_main.cpp)
    target_link_libraries(test_ros_pipeline PRIVATE intball2_stereo_ros GTest::gtest)
    add_test(NAME ros_pipeline COMMAND test_ros_pipeline)
    set_tests_properties(ros_pipeline PROPERTIES TIMEOUT 30)
    foreach(test_name IN ITEMS pointcloud_messages parameter_updates diagnostics timing)
      add_executable(test_${test_name} test/ros/test_${test_name}.cpp test/core/stereo_test_main.cpp)
      target_link_libraries(test_${test_name} PRIVATE intball2_stereo_ros GTest::gtest)
      add_test(NAME ${test_name} COMMAND test_${test_name})
      set_tests_properties(${test_name} PROPERTIES TIMEOUT 30)
    endforeach()
  endif()
endif()

if(INTBALL2_STEREO_BUILD_ROS)
  ament_export_targets(intball2_stereo_targets HAS_LIBRARY_TARGET)
  ament_export_dependencies(OpenCV rclcpp rcl_interfaces rmw sensor_msgs diagnostic_msgs Threads)
  ament_package()
else()
  include(CMakePackageConfigHelpers)
  configure_package_config_file(cmake/intball2_stereo_config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/intball2_stereoConfig.cmake
    INSTALL_DESTINATION lib/cmake/intball2_stereo)
  write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/intball2_stereoConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
  install(EXPORT intball2_stereo_targets NAMESPACE intball2_stereo::
    DESTINATION lib/cmake/intball2_stereo)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/intball2_stereoConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/intball2_stereoConfigVersion.cmake
    DESTINATION lib/cmake/intball2_stereo)
endif()
