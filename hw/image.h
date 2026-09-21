/*
 * Vendor-neutral description of a dmabuf-backed image.
 *
 * The capture, display and OSD layers all speak this type, so only the
 * compositor (hw/compositor.h) ever includes the Rockchip RGA headers.
 */
#pragma once

#include <cstddef>

namespace hw {

enum class PixelFormat {
    Gray8,      /* single luma plane (Y400), one byte per pixel */
    Y10,        /* 10-bit greyscale, LE packed bitstream (rkcif 'Y10 '),
                   1.25 B/px - never passed to RGA, down-converted first */
    NV12,       /* luma plane + interleaved chroma plane */
    BGRX8888,   /* memory order B,G,R,X - what DRM calls XRGB8888 */
    RGBA8888,   /* memory order R,G,B,A */
};

constexpr int bytes_per_pixel(PixelFormat f)
{
    return (f == PixelFormat::BGRX8888 || f == PixelFormat::RGBA8888) ? 4
         : (f == PixelFormat::Y10) ? 2 : 1;   /* Y10 value unused for RGA */
}

struct Rect { int x = 0, y = 0, w = 0, h = 0; };

struct ImageDesc {
    int    dma_fd = -1;
    void  *va = nullptr;        /* CPU mapping, may be null */
    size_t size = 0;
    int    width = 0, height = 0;
    int    stride = 0;          /* bytes per row of the first plane */
    PixelFormat format = PixelFormat::Gray8;

    bool valid() const { return dma_fd >= 0 && width > 0 && height > 0; }
};

} // namespace hw
