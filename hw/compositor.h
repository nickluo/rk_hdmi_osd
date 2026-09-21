/*
 * hw::Compositor - turns a greyscale camera frame plus an OSD overlay into
 * the finished display buffer, entirely on the Rockchip RGA 2D engine.
 *
 * This is the ONLY translation unit that knows about RGA, about NV12, and
 * about the letterbox scratch buffer. Callers deal in hw::ImageDesc.
 *
 * RGA2-E notes that shape the implementation:
 *  - direct Y400 -> RGB is broken (G stalls near 130), so luma goes through
 *    an NV12 view with a neutral chroma plane and BT.601 full-range CSC
 *  - Y400 IS valid as a destination and writes only the luma plane, so the
 *    letterbox happens in pass 1 and the chroma plane plus the black bars
 *    are written once at open() instead of on every frame
 *  - CSC and alpha blend cannot be merged: 3-channel blend demands an RGB
 *    src1 whose size equals the destination
 */
#pragma once

#include "image.h"

namespace hw {

class Compositor {
public:
    struct Config {
        int width = 720, height = 480;    /* destination size */
        Rect video{0, 14, 720, 450};      /* where the camera image lands */
    };

    Compositor() = default;
    ~Compositor();
    Compositor(const Compositor &) = delete;
    Compositor &operator=(const Compositor &) = delete;

    bool open(const Config &cfg);
    void close();
    bool valid() const { return m_scratch_fd >= 0; }

    /* blended on top of the video on every render(); pass an invalid
     * descriptor to disable. Its alpha must be 0 or 255 - RGA2-E's blend
     * math is wrong for partially transparent sources. */
    bool set_overlay(const ImageDesc &overlay);

    /* Y10 frames (SC233HGS) are unpacked to Gray8 on the CPU before the
     * RGA passes: RGA has no 10-bit luma input, and rkcif writes 'Y10 '
     * as a little-endian packed bitstream (see compositor.cpp). */
    bool render(const ImageDesc &video, const ImageDesc &dst);

private:
    struct Slot { int fd; void *handle; };

    /* dmabufs are registered with the RGA driver once and reused; the set
     * is tiny (camera buffers + 2 framebuffers + overlay + scratch) */
    void *handle_for(const ImageDesc &img);

    bool ensure_y8_scratch_(int width, int height);
    void y10_to_y8_(const ImageDesc &src);

    Config m_cfg;
    int m_scratch_fd = -1;
    void *m_scratch_va = nullptr;
    size_t m_scratch_size = 0;
    void *m_scratch_handle = nullptr;

    int m_y8_fd = -1;
    void *m_y8_va = nullptr;
    size_t m_y8_size = 0;
    int m_y8_w = 0, m_y8_h = 0, m_y8_stride = 0;

    ImageDesc m_overlay;
    Slot m_slots[16] = {};
    int m_slots_used = 0;
};

} // namespace hw
