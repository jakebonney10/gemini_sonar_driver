cmake_minimum_required(VERSION 3.8)

project(gemini_sonar_driver_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(MSG_FILES
    "msg/RawPacket.msg"
    "msg/GeminiStatus.msg"
    "msg/LoggerStatus.msg")

set(SRV_FILES
    "srv/StartSonar.srv"
    "srv/StopSonar.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
    ${MSG_FILES}
    ${SRV_FILES}
    DEPENDENCIES
    std_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
