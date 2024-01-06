cmake_minimum_required(VERSION 3.8)
project(expertino_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

set(MSGS "msg/NavGraphInterfaceMessage.msg"
         "msg/NavGraphWithMPSInterfaceMessage.msg")

set(SERVICES
    "srv/NavGraphInterfaceCompute.srv"
    "srv/NavGraphInterfaceGenerateWaitzones.srv"
    "srv/NavGraphInterfaceSetBoundingBox.srv"
    "srv/NavGraphInterfaceUpdateStationByTag.srv")

# set(ACTIONS

# )
rosidl_generate_interfaces(
  ${PROJECT_NAME} ${MSGS} ${SERVICES}
  # ${actions}
  DEPENDENCIES builtin_interfaces action_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
