cmake_minimum_required(VERSION 3.8)
project(rcll_protobuf_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcll_protobuf_msgs REQUIRED)
find_package(Protobuf REQUIRED)
message(${rcll_protobuf_msgs_DIR} " is the protobuf dir")
set(PROTO_FILES
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/AgentTask.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/AttentionMessage.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/BeaconSignal.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/ExplorationInfo.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/GameInfo.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/GameState.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/MachineCommands.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/MachineDescription.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/MachineInfo.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/MachineInstructions.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/MachineReport.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/NavigationChallenge.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/OrderInfo.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/Pose2D.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/ProductColor.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/RingInfo.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/RobotCommands.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/RobotInfo.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/SimTimeSync.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/Team.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/Time.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/VersionInfo.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/WorkpieceInfo.proto
    ${rcll_protobuf_msgs_DIR}/../rcll-protobuf-msgs/Zone.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
add_library(${PROJECT_NAME} SHARED ${PROTO_SRCS} ${PROTO_HDRS})
install(FILES ${PROTO_HDRS} DESTINATION include/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Protobuf)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
