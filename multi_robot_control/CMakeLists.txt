cmake_minimum_required(VERSION 2.8.3)
project(multi_robot_control)

set(CMAKE_BUILD_TYPE "Release")
set(HALCON_ROOT /opt/halcon)
set(HALCONARCH x64-linux)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp message_generation geometry_msgs std_msgs tf2 tf2_ros moveit_core moveit_msgs moveit_ros_planning_interface visp_hand2eye_calibration)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



# OpenCV
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# GoSDK and KAPI
# Set GO_SDK header files (That is different names for Sdk5.x )
SET(GO_SDK ~/GO_SDK)
FIND_PATH(
    GOCATOR_INCLUDES
    NAMES GoSdk/GoSdk.h
    PATHS ${GO_SDK}/Gocator/GoSdk)
FIND_PATH(
    KAPI_INCLUDES
    NAMES kApi/kApi.h
    PATHS ${GO_SDK}/Platform/kApi)
INCLUDE_DIRECTORIES(${GOCATOR_INCLUDES} ${KAPI_INCLUDES} ./include)

# Set GO_SDK libs (That is the same name for both GoSDK4.x and GoSDK5.x )

# Release Version
if(${CMAKE_BUILD_TYPE} STREQUAL "Release")
message("CMAKE_BUILD_TYPE is Release")
FIND_LIBRARY(
    GOCATOR_LIBRARIES
    NAMES GoSdk
    PATHS ${GO_SDK}/lib/linux_x64/) 
FIND_LIBRARY(
    KAPI_LIBRARIES
    NAMES kApi
    PATHS ${GO_SDK}/lib/linux_x64/)
message(STATUS "Gocator Headers   found at ${GOCATOR_INCLUDES}.")
message(STATUS "Gocator Libraries found at ${GOCATOR_LIBRARIES}.")
endif()

# Debug Version
if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
FIND_LIBRARY(
    GOCATOR_LIBRARIES
    NAMES GoSdk
    PATHS ${GO_SDK}/lib/linux_x64d/) 
FIND_LIBRARY(
    KAPI_LIBRARIES
    NAMES kApi
    PATHS ${GO_SDK}/lib/linux_x64d/)
message(STATUS "Gocator Headers   found at ${GOCATOR_INCLUDES}.")
message(STATUS "Gocator Libraries found at ${GOCATOR_LIBRARIES}.")
endif()

include_directories(include)
add_subdirectory(src/tools)

# Build library
# Set source files
set(SRCS src/libs/config.cc src/libs/dbscan.cc src/libs/DefectDetect.cc src/libs/gocator3100.cc src/libs/gocator3200.cc src/libs/pointcloud_helper.cc src/libs/Utility.cc)
# Set header files 
set(HDRS include/config.h include/dbscan.h include/DefectDetect.h include/gocator3100.h include/gocator3200.h include/pointcloud_helper.h include/Utility.h)
# Add library
add_library(${PROJECT_NAME}_LIB SHARED ${SRCS})

TARGET_LINK_LIBRARIES(${PROJECT_NAME}_LIB ${GOCATOR_LIBRARIES} ${KAPI_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})


## Generate messages in the 'msg' folder
add_message_files(FILES
  pose.msg
)

## Generate services in the 'srv' folder
add_service_files(FILES
  polish.srv
  robot_control.srv
  user_input.srv
  basler_detection.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
  message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES multi_robot_control
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${HALCON_ROOT}/include
  ${HALCON_ROOT}/include/halconcpp
# include
# ${catkin_INCLUDE_DIRS}
)
link_directories(
  ${HALCON_ROOT}/lib/${HALCONARCH}
)


add_executable(frame_transformation
  src/frame_transformation.cpp
)
add_dependencies(frame_transformation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(frame_transformation
  ${catkin_LIBRARIES}
)

add_executable(main_node
  src/main_node.cpp
)
add_dependencies(main_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_LIB
)

add_executable(user_input
  src/user_input.cpp
)
add_dependencies(user_input ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(user_input
  ${catkin_LIBRARIES}
)

add_executable(capture_robot_control
  src/capture_robot_control.cpp
)
add_dependencies(capture_robot_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(capture_robot_control
  ${catkin_LIBRARIES}
)

add_executable(polish_robot_control
  src/polish_robot_control.cpp
)
add_dependencies(polish_robot_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(polish_robot_control
  ${catkin_LIBRARIES}
)

add_executable(eigen_test
  src/eigen_test.cpp
)
add_dependencies(eigen_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(eigen_test
  ${catkin_LIBRARIES}
)

add_executable(realsense_frame_transform
  src/realsense_frame_transform.cpp
)
add_dependencies(realsense_frame_transform ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(realsense_frame_transform
  ${catkin_LIBRARIES}
)

add_executable(tool_pose_pub
  src/tool_pose_pub.cpp
)
add_dependencies(tool_pose_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tool_pose_pub
  ${catkin_LIBRARIES}
)

add_executable(basler_detection
  src/basler_detection.cpp
)
add_dependencies(basler_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(basler_detection
  ${catkin_LIBRARIES} halconcpp halcon
)
