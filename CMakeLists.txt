# osd_stack - portable FPV OSD overlay stack for RK3576.
#
# Standalone build (board or host):
#   cmake -B build -DLIBRGA_ROOT=$HOME/workspace/librga && cmake --build build -j
# Host build without vendor libs (osd::core + layout + tests only):
#   cmake -B build-x86 -DOSD_WITH_HW=OFF && cmake --build build-x86 -j && ctest
#
# Embedded build: add_subdirectory()'d from the osd_bridge ament package
# (OSD_STANDALONE off then) so one colcon build on the board produces
# everything. No install() rules: static libs are consumed in-tree.
cmake_minimum_required(VERSION 3.16)
project(osd_stack VERSION 0.2.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
# colcon never sets CMAKE_BUILD_TYPE and this stack is performance-critical;
# -O2 RelWithDebInfo matches the historical Makefile flags.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()
add_compile_options(-Wall -Wextra)

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  set(OSD_STANDALONE ON)
else()
  set(OSD_STANDALONE OFF)
endif()

# librga SDK location on the board (Makefile default: ~/workspace/librga)
set(LIBRGA_ROOT "$ENV{HOME}/workspace/librga" CACHE PATH "librga SDK root (v1.10.6)")
option(OSD_WITH_HW "Build osd::hw (needs libdrm + librga)" ${OSD_STANDALONE})

find_package(Threads REQUIRED)

add_subdirectory(osd)
add_subdirectory(layout)
if(OSD_WITH_HW)
  add_subdirectory(hw)
endif()

if(OSD_STANDALONE)
  enable_testing()
  add_executable(probe8 probe8.cpp)
  target_link_libraries(probe8 PRIVATE osd::core)
  add_executable(test_layout_snapshot tests/test_layout_snapshot.cpp)
  target_link_libraries(test_layout_snapshot PRIVATE osd::core osd::layout)
  add_test(NAME layout_snapshot COMMAND test_layout_snapshot ${CMAKE_BINARY_DIR})
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
    # NEON raster micro-benchmark (host-side ARM only)
    add_executable(probe10 probe10.cpp)
  endif()

  if(OSD_WITH_HW)
    add_executable(osd_demo apps/osd_demo.cpp)
    target_link_libraries(osd_demo PRIVATE osd::core osd::hw)
    set_target_properties(osd_demo PROPERTIES
      BUILD_RPATH ${LIBRGA_ROOT}/libs/Linux/gcc-aarch64)
    add_executable(probe9 probe9.cpp)
    target_link_libraries(probe9 PRIVATE osd::hw)
    set_target_properties(probe9 PROPERTIES
      BUILD_RPATH ${LIBRGA_ROOT}/libs/Linux/gcc-aarch64)
    add_executable(test_cam test_cam.cpp)
    target_link_libraries(test_cam PRIVATE osd::hw)
  endif()
endif()
