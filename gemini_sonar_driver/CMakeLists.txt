cmake_minimum_required(VERSION 3.8)

################################
##  CONFIGURE THIS FILE HERE  ##
################################

set(ROS_DEPENDS
    ament_cmake
    rclcpp
    std_msgs
    sensor_msgs
    marine_acoustic_msgs
    gemini_sonar_driver_interfaces
    rosbag2_cpp
  )

set(EXTERNAL_DEPENDS

  )

set(EXTERNAL_DEPENDS_LIB

  )

file(GLOB GEMINI_SDK_DIRS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/.." "${CMAKE_CURRENT_SOURCE_DIR}/../GeminiSDK_v*")
list(SORT GEMINI_SDK_DIRS)
list(REVERSE GEMINI_SDK_DIRS)
list(GET GEMINI_SDK_DIRS 0 GEMINI_SDK_VERSION_DIR)
set(GEMINI_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../${GEMINI_SDK_VERSION_DIR}")
message(STATUS "Auto-selected GeminiSDK: ${GEMINI_SDK_DIR}")
set(GEMINI_LIB_DIR ${GEMINI_SDK_DIR}/bin)

get_filename_component(PACKAGE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message("Creating Package: ${PACKAGE_NAME}")
project(${PACKAGE_NAME})  ## this package name is the name of the directory this cmake file is in


add_compile_definitions(CMAKE_PACKAGE_NAME=${PACKAGE_NAME})

# Default to C++17 for ROS2 Humble and Iron
# Use C++20 for ROS2 Jazzy and newer distributions
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###
###  Add dependencies
###

foreach(DEPEND ${ROS_DEPENDS} ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)


################
## Find files ##
################

file(GLOB HDRS CONFIGURE_DEPENDS
  "include/${PROJECT_NAME}/*.h"
)
file(GLOB SRCS CONFIGURE_DEPENDS
  "src/*.cpp"
  "src/*.hpp"
)
file(GLOB NODES CONFIGURE_DEPENDS
  "nodes/*.cpp"
)

include_directories(
  include
  ${GEMINI_SDK_DIR}/include
  ${GEMINI_SDK_DIR}/include/Svs5Seq
  ${GEMINI_SDK_DIR}/include/Gemini
  ${GEMINI_SDK_DIR}/include/GenesisSerializer
)

link_directories(${GEMINI_SDK_DIR}/bin)

###########
## Build ##
###########

set(EXECUTABLES "")  # for installing later

message("building ${PROJECT_NAME} node: ")

foreach(NODE_FILE ${NODES})
  get_filename_component(NODE_NAME ${NODE_FILE} NAME_WE)
  message("  - " ${NODE_NAME})
  list(APPEND EXECUTABLES ${NODE_NAME})
  add_executable(${NODE_NAME} ${NODE_FILE} ${HDRS} ${SRCS})
  ament_target_dependencies(${NODE_NAME} ${ROS_DEPENDS} ${EXTERNAL_DEPENDS_LIB})
  target_include_directories(${NODE_NAME} PRIVATE "include/${PROJECT_NAME}/")
  
  # Link Gemini SDK libraries (Svs5SeqLib includes GeminiComms, add GenesisSerializer for GLF processing)
  target_link_libraries(${NODE_NAME} Svs5SeqLib GenesisSerializer)
endforeach(NODE_FILE)

# Get absolute path to SDK for environment hook
get_filename_component(GEMINI_LIB_DIR_ABS "${GEMINI_LIB_DIR}" ABSOLUTE)

message("  Installing: " ${EXECUTABLES})
install(TARGETS
  ${EXECUTABLES}
  DESTINATION lib/${PROJECT_NAME}/
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Create environment hook to add Gemini SDK to LD_LIBRARY_PATH
# ubuntu 24.04 uses DT_RUNPATH instead of DT_RPATH, so we need to set LD_LIBRARY_PATH
set(GEMINI_ENV_HOOK "${CMAKE_CURRENT_BINARY_DIR}/env_hook/gemini_sdk.sh")
file(WRITE "${GEMINI_ENV_HOOK}"
"# Automatically add Gemini SDK to LD_LIBRARY_PATH
export LD_LIBRARY_PATH=\"${GEMINI_LIB_DIR_ABS}:\$LD_LIBRARY_PATH\"
")
ament_environment_hooks("${GEMINI_ENV_HOOK}")

#############
## testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
