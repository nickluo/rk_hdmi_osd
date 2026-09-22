# osd::core - pure-CPU rasteriser, zero vendor dependencies (portable to x86).
# Consumers do #include "osd/osd.h"; osd.h includes "osd_font.h" from its
# own directory, so the repo root is the only public include path.
add_library(osd_core STATIC osd.cpp)
add_library(osd::core ALIAS osd_core)
target_include_directories(osd_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
