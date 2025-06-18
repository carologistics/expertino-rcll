cmake_minimum_required(VERSION 3.8)
project(expertino)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcll_protobuf_msgs REQUIRED)
find_package(cx_plugin REQUIRED)
find_package(cx_ros_comm_gen REQUIRED)

# plugins for pddl planner action client and nested message of it
cx_generate_msg_bindings("pddl_msgs" "TimedPlanAction")
cx_generate_action_bindings("pddl_msgs" "PlanTemporal")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(FILES launch/refbox.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch params pddl DESTINATION share/${PROJECT_NAME})
install(DIRECTORY clips/ DESTINATION share/${PROJECT_NAME}/clips/expertino/)

ament_export_dependencies(${dependencies})

ament_package()
