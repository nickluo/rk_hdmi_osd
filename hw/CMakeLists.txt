# osd::hw - RK3576 hardware layer: V4L2 MIPI capture, DRM/KMS HDMI output
# with overlay plane, RGA compositor. Vendor deps: libdrm (pkg-config) +
# librga SDK (LIBRGA_ROOT). compositor.cpp is the only TU that includes
# im2d.hpp; the include dir is PUBLIC so its consumers need nothing extra.
find_path(RGA_INCLUDE_DIR im2d.hpp HINTS ${LIBRGA_ROOT}/include REQUIRED)
find_library(RGA_LIBRARY NAMES rga HINTS ${LIBRGA_ROOT}/libs/Linux/gcc-aarch64 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(DRM REQUIRED IMPORTED_TARGET libdrm)

add_library(osd_hw STATIC camera.cpp sof2epoch.cpp display.cpp compositor.cpp dmabuf.cpp)
add_library(osd::hw ALIAS osd_hw)
target_include_directories(osd_hw PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${RGA_INCLUDE_DIR})
target_link_libraries(osd_hw PUBLIC PkgConfig::DRM Threads::Threads ${RGA_LIBRARY})
