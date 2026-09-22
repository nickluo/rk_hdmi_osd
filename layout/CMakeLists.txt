# osd::layout - Betaflight-style element layout engine.
# Depends only on osd::core: portable to any host, no vendor libraries.
add_library(osd_layout STATIC layout.cpp)
add_library(osd::layout ALIAS osd_layout)
target_include_directories(osd_layout PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(osd_layout PUBLIC osd::core)
