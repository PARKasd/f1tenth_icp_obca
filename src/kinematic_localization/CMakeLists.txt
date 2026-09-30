cmake_minimum_required(VERSION 3.16...3.26)
project(kinematic_localization LANGUAGES CXX)

# Vendored kinematic-icp core (third_party/kinematic-icp). Sophus and
# tsl-robin-map are not installable system-wide (no sudo/apt), so force the
# FetchContent path of the underlying kiss-icp dependency.
set(USE_SYSTEM_SOPHUS OFF CACHE BOOL "Fetch Sophus instead of system package" FORCE)
set(USE_SYSTEM_TSL-ROBIN-MAP OFF CACHE BOOL "Fetch tsl-robin-map instead of system package" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/kinematic-icp/cpp/kinematic_icp
                 ${CMAKE_CURRENT_BINARY_DIR}/kinematic_icp_core)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
    ament_index_cpp
    diagnostic_msgs
    f110_msgs
    geometry_msgs
    laser_geometry
    nav_msgs
    rclcpp
    sensor_msgs
    std_srvs
    tf2
    tf2_msgs
    tf2_ros)

add_executable(localization_node src/localization_node.cpp)
target_compile_features(localization_node PRIVATE cxx_std_17)
target_include_directories(localization_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(localization_node kinematic_icp_pipeline)
ament_target_dependencies(localization_node ${dependencies})

add_executable(mapping_node src/mapping_node.cpp)
target_compile_features(mapping_node PRIVATE cxx_std_17)
target_include_directories(mapping_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(mapping_node kinematic_icp_pipeline)
ament_target_dependencies(mapping_node ${dependencies})
ament_target_dependencies(mapping_node rosbag2_cpp rosbag2_storage)

install(TARGETS localization_node mapping_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/pgm_to_kissmap.py scripts/compare_trajectory.py scripts/wall_rate.py
                scripts/param_sweep.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(config_guard test/test_config_guard.py)

  # §9 scan-end synchronisation decision logic (2026-08-22). Header-only and
  # ROS-free on purpose so the "never clamp / never extrapolate" contract is
  # checked without a graph.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan_odom_sync test/test_scan_odom_sync.cpp)
  if(TARGET test_scan_odom_sync)
    target_compile_features(test_scan_odom_sync PRIVATE cxx_std_17)
    target_include_directories(test_scan_odom_sync PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  endif()

  ament_add_gtest(test_tilt_compensation test/test_tilt_compensation.cpp)
  if(TARGET test_tilt_compensation)
    target_compile_features(test_tilt_compensation PRIVATE cxx_std_17)
    target_include_directories(test_tilt_compensation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(test_tilt_compensation kinematic_icp_pipeline)
    ament_target_dependencies(test_tilt_compensation ${dependencies})
  endif()
endif()

ament_package()
