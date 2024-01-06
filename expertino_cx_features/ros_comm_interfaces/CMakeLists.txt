cmake_minimum_required(VERSION 3.8)
project(ros_comm_interfaces)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set(ryml_DIR /home/userin4o/rapidyaml/install/lib/cmake/ryml)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cx_core REQUIRED)
find_package(expertino_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)

set(DEPENDENCIES ament_cmake cx_utils cx_core cx_msgs expertino_msgs pluginlib)

pkg_check_modules(CLIPSMM REQUIRED IMPORTED_TARGET clipsmm-1.0)

include_directories(include)

pluginlib_export_plugin_description_file(ros_comm_interfaces
                                         features_plugin.xml)

add_library(navgraph_compute SHARED src/ros_comm_interfaces/NavGraphCompute.cpp)
ament_target_dependencies(navgraph_compute ${DEPENDENCIES})
target_link_libraries(navgraph_compute PkgConfig::CLIPSMM)

add_library(navgraph_set_bounding_box SHARED
            src/ros_comm_interfaces/NavGraphSetBoundingBox.cpp)
ament_target_dependencies(navgraph_set_bounding_box ${DEPENDENCIES})
target_link_libraries(navgraph_set_bounding_box PkgConfig::CLIPSMM)

add_library(navgraph_generate_waitzones SHARED
            src/ros_comm_interfaces/NavGraphGenerateWaitzones.cpp)
ament_target_dependencies(navgraph_generate_waitzones ${DEPENDENCIES})
target_link_libraries(navgraph_generate_waitzones PkgConfig::CLIPSMM)

add_library(navgraph_update_station_by_tag SHARED
            src/ros_comm_interfaces/NavGraphUpdateStationByTag.cpp)
ament_target_dependencies(navgraph_update_station_by_tag ${DEPENDENCIES})
target_link_libraries(navgraph_update_station_by_tag PkgConfig::CLIPSMM)

# END OF PLUGINS REGISTRATION

install(DIRECTORY include/ DESTINATION include/)

install(FILES features_plugin.xml DESTINATION share/${PROJECT_NAME})

install(
  TARGETS navgraph_compute navgraph_set_bounding_box navgraph_generate_waitzones
          navgraph_update_station_by_tag
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto)

  if(ament_lint_auto_FOUND)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest)
    find_package(cx_clips_executive REQUIRED)

    if(ament_cmake_gtest)
      add_subdirectory(test)
    endif()
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
