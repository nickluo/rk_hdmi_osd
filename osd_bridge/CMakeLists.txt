cmake_minimum_required(VERSION 3.16)
project(osd_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()
add_compile_options(-Wall -Wextra)

# ---- board-only package ------------------------------------------------
# The render pipeline drives the RK3576 V4L2/DRM/RGA hardware directly.
# On an x86 host, build the osd stack standalone instead:
#   cmake -B build-x86 -DOSD_WITH_HW=OFF   (osd_core + osd_layout + tests)
if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
  message(FATAL_ERROR
    "osd_bridge targets the RK3576 board (aarch64, native ROS2). "
    "Deploy this repo to the board (~/workspace/ar0234_osd_hdmi), symlink "
    "osd_bridge/ into the ROS workspace src/, and colcon build "
    "--packages-select osd_bridge there. On x86 only the portable subset "
    "builds (-DOSD_WITH_HW=OFF in the osd stack).")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# ---- the osd stack: one colcon build produces everything ---------------
# Default: this package lives INSIDE the ar0234_osd_hdmi checkout (the
# parent of this directory), so one sync serves both the libs and the node.
# The package is typically reached through a symlink from the ROS
# workspace src/: resolve THIS directory's real path first, then take its
# parent (CMake REALPATH lexical-collapses "symlink/.." before resolving,
# so a plain "${CMAKE_CURRENT_SOURCE_DIR}/.. REALPATH" lands in src/).
# Override -DOSD_STACK_ROOT=<path> only for split layouts.
get_filename_component(_pkg_real "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
get_filename_component(_osd_stack_default "${_pkg_real}" DIRECTORY)
set(OSD_STACK_ROOT "${_osd_stack_default}" CACHE PATH
    "ar0234_osd_hdmi checkout (the osd hw/osd/layout libraries)")
set(LIBRGA_ROOT "$ENV{HOME}/workspace/librga" CACHE PATH "librga SDK root")
if(NOT EXISTS "${LIBRGA_ROOT}/include/im2d.hpp")
  # running as root $HOME is /root - probe the standard board locations
  file(GLOB _librga_probe /home/*/workspace/librga)
  foreach(_c ${_librga_probe})
    if(EXISTS "${_c}/include/im2d.hpp")
      set(LIBRGA_ROOT "${_c}" CACHE PATH "librga SDK root" FORCE)
      break()
    endif()
  endforeach()
endif()
if(NOT EXISTS "${OSD_STACK_ROOT}/osd/osd.h")
  message(FATAL_ERROR
    "OSD_STACK_ROOT (${OSD_STACK_ROOT}) does not look like an "
    "ar0234_osd_hdmi checkout (osd/osd.h missing). Check the checkout or "
    "pass -DOSD_STACK_ROOT=<path>.")
endif()
set(OSD_WITH_HW ON CACHE BOOL "" FORCE)
add_subdirectory(${OSD_STACK_ROOT} osd_stack)

# ---- bridge library + node ---------------------------------------------
# osd_bridge_pipeline: ROS-free render engine (camera->RGA->HDMI loop,
# telemetry/box seams, snapshot dumps) - reusable outside this node.
add_library(osd_bridge_pipeline STATIC
  src/pipeline.cpp
)
target_include_directories(osd_bridge_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(osd_bridge_pipeline PUBLIC osd::core osd::layout osd::hw)

add_executable(osd_bridge_node
  src/bridge_node.cpp
  src/main.cpp
)
target_include_directories(osd_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
# plain signature: ament_target_dependencies (below) links with the plain
# form and CMake forbids mixing keyword/plain on one target
target_link_libraries(osd_bridge_node osd_bridge_pipeline)
ament_target_dependencies(osd_bridge_node
  rclcpp std_msgs std_srvs sensor_msgs quadrotor_msgs mavros_msgs)

# installed executable must still find the librga .so at the SDK path
set_target_properties(osd_bridge_node PROPERTIES
  INSTALL_RPATH ${LIBRGA_ROOT}/libs/Linux/gcc-aarch64)

install(TARGETS osd_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch parameters DESTINATION share/${PROJECT_NAME})

ament_package()
