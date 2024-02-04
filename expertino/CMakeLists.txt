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
execute_process(
  COMMAND
    ${CMAKE_COMMAND} -E create_symlink
    ${rcll_protobuf_msgs_DIR}/rcll-protobuf-msgs
    ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/rcll-protobuf-msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(FILES launch/refbox.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY clips launch params DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})

ament_package()
