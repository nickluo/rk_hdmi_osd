# AR0234 -> RGA OSD -> HDMI (NTSC 720x480) pipeline + libosd
# Build on the RK3576 board (aarch64, Ubuntu 22.04):
#   make LIBRGA=/home/firefly/workspace/librga

LIBRGA  ?= $(HOME)/workspace/librga
CXX     ?= g++

CXXFLAGS := -O2 -std=c++17 -Wall -Wextra -pthread \
            $(shell pkg-config --cflags libdrm) \
            -I$(LIBRGA)/include -I.
LDLIBS   := $(shell pkg-config --libs libdrm) \
            $(LIBRGA)/libs/Linux/gcc-aarch64/librga.so
LDFLAGS  := -Wl,-rpath=$(LIBRGA)/libs/Linux/gcc-aarch64

all: osd_demo

# ---- libosd ----
osd/osd.o: osd/osd.cpp osd/osd.h osd_font.h
	@mkdir -p osd
	$(CXX) $(CXXFLAGS) -c osd/osd.cpp -o $@

libosd.a: osd/osd.o
	ar rcs $@ $^

# ---- hardware layer: MIPI capture + HDMI/KMS output + RGA compositor ----
hw/camera.o: hw/camera.cpp hw/camera.h hw/image.h hw/sof2epoch.h
	$(CXX) $(CXXFLAGS) -c hw/camera.cpp -o $@

hw/sof2epoch.o: hw/sof2epoch.cpp hw/sof2epoch.h
	$(CXX) $(CXXFLAGS) -c hw/sof2epoch.cpp -o $@

hw/display.o: hw/display.cpp hw/display.h hw/image.h
	$(CXX) $(CXXFLAGS) -c hw/display.cpp -o $@

hw/compositor.o: hw/compositor.cpp hw/compositor.h hw/image.h
	$(CXX) $(CXXFLAGS) -c hw/compositor.cpp -o $@

libhw.a: hw/camera.o hw/sof2epoch.o hw/display.o hw/compositor.o
	ar rcs $@ $^

osd_demo: osd_demo.cpp libosd.a libhw.a osd/osd.h hw/camera.h hw/display.h hw/compositor.h
	$(CXX) $(CXXFLAGS) osd_demo.cpp libosd.a libhw.a -o $@ $(LDFLAGS) $(LDLIBS)

# ---- probes & tests, none of them part of `all` ----
# camera-only smoke test (no DRM/HDMI needed)
test_cam: test_cam.cpp libhw.a hw/camera.h
	$(CXX) $(CXXFLAGS) test_cam.cpp libhw.a -o $@ $(LDFLAGS) $(LDLIBS)

# libosd regression; needs no camera, no DRM and no vendor library
probe8: probe8.cpp libosd.a osd/osd.h
	$(CXX) $(CXXFLAGS) probe8.cpp libosd.a -o $@

# RGA pipeline capability/optimization checks (no camera/DRM needed)
probe9: probe9.cpp
	$(CXX) $(CXXFLAGS) probe9.cpp -o $@ $(LDFLAGS) $(LDLIBS)

# raster micro-benchmark (pure CPU)
probe10: probe10.cpp
	$(CXX) $(CXXFLAGS) probe10.cpp -o $@

probes: probe8 probe9 probe10

clean:
	rm -f osd_demo probe8 probe9 probe10 libosd.a osd/osd.o \
	      libhw.a hw/camera.o hw/sof2epoch.o hw/display.o hw/compositor.o

.PHONY: all clean probes
