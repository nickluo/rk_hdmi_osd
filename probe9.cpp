/* probe9: optimization probes for the RK3576 RGA pipeline.
 *
 * A) Does op1 (Y400 -> NV12) clobber the chroma plane?  If RGA leaves UV
 *    alone, the per-frame 162KB memset(128) + 2 cache-sync ioctls in the
 *    render loop are pure waste and can move to init.
 * B) Is Y400 accepted as a *destination*?  Writing only the Y plane would
 *    make the UV preservation guaranteed rather than observed.
 * C) Can imcomposite() do CSC + letterbox scale + alpha blend in ONE op,
 *    replacing op2 + op3 (saves ~0.93ms of RGA time per frame)?
 *
 * Needs no camera and no DRM master - safe to run over ssh.
 */
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/dma-heap.h>
#include <linux/dma-buf.h>
#include "im2d.hpp"

static int g_fail = 0;

static void check(bool ok, const char *what, const char *detail)
{
    printf("%-4s %-38s %s\n", ok ? "PASS" : "FAIL", what, detail);
    if (!ok) g_fail++;
}

static double now_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

struct Buf {
    int fd = -1;
    uint8_t *va = nullptr;
    size_t size = 0;
    rga_buffer_handle_t h = 0;

    bool alloc(size_t bytes)
    {
        int heap = open("/dev/dma_heap/system", O_RDWR);
        if (heap < 0) return false;
        struct dma_heap_allocation_data ad = {};
        ad.len = bytes;
        ad.fd_flags = O_RDWR | O_CLOEXEC;
        int r = ioctl(heap, DMA_HEAP_IOCTL_ALLOC, &ad);
        close(heap);
        if (r < 0) return false;
        fd = ad.fd;
        size = bytes;
        va = (uint8_t *)mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (va == MAP_FAILED) return false;
        h = importbuffer_fd(fd, bytes);
        return h != 0;
    }
    void sync(bool start)
    {
        struct dma_buf_sync s = {};
        s.flags = DMA_BUF_SYNC_RW | (start ? DMA_BUF_SYNC_START : DMA_BUF_SYNC_END);
        ioctl(fd, DMA_BUF_IOCTL_SYNC, &s);
    }
};

static const int CW = 1920, CH = 1200;            /* camera */
static const int VW = 720,  VH = 450;             /* letterboxed video */
static const int SW = 720,  SH = 480;             /* screen */

int main()
{
    Buf ysrc, nv12, fb_ref, fb_one, hud;
    if (!ysrc.alloc((size_t)CW * CH) || !nv12.alloc((size_t)VW * VH * 3 / 2) ||
        !fb_ref.alloc((size_t)SW * SH * 4) || !fb_one.alloc((size_t)SW * SH * 4) ||
        !hud.alloc((size_t)SW * SH * 4)) {
        printf("alloc failed\n");
        return 1;
    }
    char d[200];

    /* synthetic camera frame: horizontal gradient */
    ysrc.sync(true);
    for (int y = 0; y < CH; y++)
        for (int x = 0; x < CW; x++)
            ysrc.va[(size_t)y * CW + x] = (uint8_t)(x * 255 / (CW - 1));
    ysrc.sync(false);

    rga_buffer_t cam = wrapbuffer_handle_t(ysrc.h, CW, CH, CW, CH, RK_FORMAT_YCbCr_400);
    rga_buffer_t nv_dst = wrapbuffer_handle_t(nv12.h, VW, VH, VW, VH, RK_FORMAT_YCbCr_420_SP);
    rga_buffer_t nv_src = wrapbuffer_handle_t(nv12.h, VW, VH, VW, VH, RK_FORMAT_YCbCr_420_SP);
    imsetColorSpace(&nv_src, (IM_COLOR_SPACE_MODE)IM_YUV_BT601_FULL_RANGE);
    rga_buffer_t y_dst = wrapbuffer_handle_t(nv12.h, VW, VH, VW, VH, RK_FORMAT_YCbCr_400);
    rga_buffer_t dst_ref = wrapbuffer_handle_t(fb_ref.h, SW, SH, SW, SH, RK_FORMAT_BGRX_8888);
    rga_buffer_t dst_one = wrapbuffer_handle_t(fb_one.h, SW, SH, SW, SH, RK_FORMAT_BGRX_8888);
    rga_buffer_t hud_b  = wrapbuffer_handle_t(hud.h, SW, SH, SW, SH, RK_FORMAT_RGBA_8888);

    uint8_t *uv = nv12.va + (size_t)VW * VH;
    const size_t uv_len = (size_t)VW * VH / 2;

    /* ---- A. does Y400 -> NV12 touch the chroma plane? ---- */
    nv12.sync(true);
    memset(uv, 0xAA, uv_len);
    nv12.sync(false);
    double t0 = now_ms();
    IM_STATUS s = improcess(cam, nv_dst, {}, {}, {}, {}, -1, nullptr, nullptr, IM_SYNC);
    const double t_op1 = now_ms() - t0;
    if (s != IM_STATUS_SUCCESS) { printf("op1 failed: %s\n", imStrError(s)); return 1; }
    nv12.sync(true);
    size_t kept = 0;
    for (size_t i = 0; i < uv_len; i++) if (uv[i] == 0xAA) kept++;
    nv12.sync(false);
    const bool uv_untouched = (kept == uv_len);
    snprintf(d, sizeof d, "0xAA bytes surviving: %zu/%zu", kept, uv_len);
    check(true, uv_untouched ? "op1 leaves UV untouched" : "op1 OVERWRITES UV", d);

    /* ---- B. Y400 as destination (writes the Y plane only) ---- */
    nv12.sync(true);
    memset(uv, 0x5A, uv_len);
    memset(nv12.va, 0, (size_t)VW * VH);
    nv12.sync(false);
    s = improcess(cam, y_dst, {}, {}, {}, {}, -1, nullptr, nullptr, IM_SYNC);
    const bool y400_dst_ok = (s == IM_STATUS_SUCCESS);
    if (y400_dst_ok) {
        nv12.sync(true);
        size_t uvkeep = 0;
        for (size_t i = 0; i < uv_len; i++) if (uv[i] == 0x5A) uvkeep++;
        /* the gradient must survive the 1920->720 downscale */
        int mid = nv12.va[(size_t)(VH / 2) * VW + VW / 2];
        int expect = (VW / 2) * 255 / (VW - 1);
        nv12.sync(false);
        snprintf(d, sizeof d, "UV kept %zu/%zu, Y(mid)=%d want~%d",
                 uvkeep, uv_len, mid, expect);
        check(uvkeep == uv_len && abs(mid - expect) <= 4,
              "Y400 dst: writes Y only, UV intact", d);
    } else {
        snprintf(d, sizeof d, "%s", imStrError(s));
        check(false, "Y400 accepted as destination", d);
    }

    /* neutral chroma for the colour path below */
    nv12.sync(true);
    memset(uv, 128, uv_len);
    nv12.sync(false);

    /* HUD: opaque white box + transparent elsewhere */
    hud.sync(true);
    memset(hud.va, 0, hud.size);
    for (int y = 40; y < 90; y++)
        for (int x = 60; x < 200; x++)
            ((uint32_t *)hud.va)[(size_t)y * SW + x] = 0xFFEBEBEB;   /* A,B,G,R */
    hud.sync(false);

    im_rect vrect = { 0, 14, VW, VH };

    /* ---- reference: op2 (CSC + letterbox) then op3 (blend) ---- */
    memset(fb_ref.va, 0, fb_ref.size);
    t0 = now_ms();
    s = improcess(nv_src, dst_ref, {}, {}, vrect, {}, -1, nullptr, nullptr, IM_SYNC);
    if (s != IM_STATUS_SUCCESS) { printf("op2 failed: %s\n", imStrError(s)); return 1; }
    double t_op2 = now_ms() - t0;
    t0 = now_ms();
    s = imblend(hud_b, dst_ref, IM_ALPHA_BLEND_SRC_OVER, 1);
    if (s != IM_STATUS_SUCCESS) { printf("op3 failed: %s\n", imStrError(s)); return 1; }
    double t_op3 = now_ms() - t0;

    /* ---- C. one-shot: video + hud -> fb in a single RGA job.
     * imcomposite() takes no rects so it cannot letterbox; improcess()
     * carries a pat (= srcB) channel plus drect, which can. ---- */
    memset(fb_one.va, 0, fb_one.size);
    t0 = now_ms();
    s = imcomposite(nv_src, hud_b, dst_one, IM_ALPHA_BLEND_SRC_OVER, 1);
    double t_one = now_ms() - t0;
    snprintf(d, sizeof d, "%s, %.2fms (no rect arg -> cannot letterbox)",
             s == IM_STATUS_SUCCESS ? "supported" : imStrError(s), t_one);
    check(s == IM_STATUS_SUCCESS, "imcomposite 3-channel supported", d);

    /* ---- D. proposed pipeline: letterbox in op1, so op2 is a full-frame
     * CSC + blend and satisfies "src1 w/h == dst w/h" ----
     *   op1': cam Y400 -> Y400 dst 720x480, drect {0,14,720,450}
     *         (Y only: bars stay 0 and UV stays 128 from init)
     *   op2': improcess(nv12 720x480, fb, pat=hud, BLEND)            */
    Buf nv48;
    if (!nv48.alloc((size_t)SW * SH * 3 / 2)) { printf("alloc nv48\n"); return 1; }
    rga_buffer_t y48_dst = wrapbuffer_handle_t(nv48.h, SW, SH, SW, SH, RK_FORMAT_YCbCr_400);
    rga_buffer_t nv48_src = wrapbuffer_handle_t(nv48.h, SW, SH, SW, SH, RK_FORMAT_YCbCr_420_SP);
    imsetColorSpace(&nv48_src, (IM_COLOR_SPACE_MODE)IM_YUV_BT601_FULL_RANGE);

    /* one-time init, exactly what the demo would do at startup */
    nv48.sync(true);
    memset(nv48.va, 0, (size_t)SW * SH);                       /* black bars */
    memset(nv48.va + (size_t)SW * SH, 128, (size_t)SW * SH / 2);
    nv48.sync(false);

    memset(fb_one.va, 0, fb_one.size);
    t0 = now_ms();
    s = improcess(cam, y48_dst, {}, {}, vrect, {}, -1, nullptr, nullptr, IM_SYNC);
    double t_op1b = now_ms() - t0;
    if (s != IM_STATUS_SUCCESS) {
        snprintf(d, sizeof d, "op1' Y400 dst with drect: %s", imStrError(s));
        check(false, "letterbox into a Y400 destination", d);
    } else {
        nv48.sync(true);
        const uint8_t *y = nv48.va;
        bool bars_black = true;
        for (int yy = 0; yy < 14 && bars_black; yy++)
            for (int xx = 0; xx < SW; xx++)
                if (y[(size_t)yy * SW + xx]) { bars_black = false; break; }
        size_t uvbad = 0;
        for (size_t i = 0; i < (size_t)SW * SH / 2; i++)
            if (nv48.va[(size_t)SW * SH + i] != 128) uvbad++;
        nv48.sync(false);
        snprintf(d, sizeof d, "%.2fms, bars black=%d, UV!=128 bytes=%zu",
                 t_op1b, (int)bars_black, uvbad);
        check(bars_black && uvbad == 0, "letterbox into a Y400 destination", d);
    }

    t0 = now_ms();
    s = improcess(nv48_src, dst_one, hud_b, {}, {}, {}, -1, nullptr, nullptr,
                  IM_ALPHA_BLEND_SRC_OVER | IM_SYNC);
    double t_op2b = now_ms() - t0;
    if (s != IM_STATUS_SUCCESS) {
        snprintf(d, sizeof d, "%s", imStrError(s));
        check(false, "full-frame CSC+blend in one op", d);
    } else {
        /* classify: the X byte of BGRX is ignored by the scanout engine, a
         * difference there is harmless - RGB differences are not */
        size_t x_diff = 0, rgb_diff = 0, rgb_max = 0, rgb_first = 0;
        for (size_t i = 0; i < (size_t)SW * SH * 4; i++) {
            size_t dv = fb_ref.va[i] > fb_one.va[i] ? fb_ref.va[i] - fb_one.va[i]
                                                    : fb_one.va[i] - fb_ref.va[i];
            if (!dv) continue;
            if (i % 4 == 3) { x_diff++; continue; }
            if (!rgb_diff) rgb_first = i / 4;
            rgb_diff++;
            if (dv > rgb_max) rgb_max = dv;
        }
        snprintf(d, sizeof d, "2-op %.2f+%.2f=%.2fms vs 3-op %.2f+%.2f+%.2f=%.2fms; "
                 "RGB diff %zu (max %zu, first px %zu,%zu) X-byte diff %zu",
                 t_op1b, t_op2b, t_op1b + t_op2b, t_op1, t_op2, t_op3,
                 t_op1 + t_op2 + t_op3, rgb_diff, rgb_max,
                 rgb_first % SW, rgb_first / SW, x_diff);
        check(rgb_diff == 0, "2-op pipeline == 3-op (RGB channels)", d);
    }

    /* ---- E. blend channel order: is `pat` the foreground or the background?
     * D showed the video winning inside the opaque HUD box, i.e. src is
     * composited OVER pat. Swap them: src = HUD, pat = video. ---- */
    memset(fb_one.va, 0, fb_one.size);
    t0 = now_ms();
    s = improcess(hud_b, dst_one, nv48_src, {}, {}, {}, -1, nullptr, nullptr,
                  IM_ALPHA_BLEND_SRC_OVER | IM_SYNC);
    double t_swap = now_ms() - t0;
    if (s != IM_STATUS_SUCCESS) {
        snprintf(d, sizeof d, "src=HUD pat=video: %s", imStrError(s));
        check(false, "swapped channel order", d);
    } else {
        size_t rgb_diff = 0, rgb_max = 0, rgb_first = 0, x_diff = 0;
        for (size_t i = 0; i < (size_t)SW * SH * 4; i++) {
            size_t dv = fb_ref.va[i] > fb_one.va[i] ? fb_ref.va[i] - fb_one.va[i]
                                                    : fb_one.va[i] - fb_ref.va[i];
            if (!dv) continue;
            if (i % 4 == 3) { x_diff++; continue; }
            if (!rgb_diff) rgb_first = i / 4;
            rgb_diff++;
            if (dv > rgb_max) rgb_max = dv;
        }
        snprintf(d, sizeof d, "%.2fms; RGB diff %zu (max %zu, first px %zu,%zu) X diff %zu",
                 t_swap, rgb_diff, rgb_max, rgb_first % SW, rgb_first / SW, x_diff);
        check(rgb_diff == 0, "src=HUD over pat=video == 3-op", d);
    }

    /* ---- cost of the per-frame UV memset we want to delete ---- */
    nv12.sync(true);
    t0 = now_ms();
    for (int i = 0; i < 100; i++) memset(uv, 128, uv_len);
    double t_memset = (now_ms() - t0) / 100;
    nv12.sync(false);
    t0 = now_ms();
    for (int i = 0; i < 100; i++) { nv12.sync(true); nv12.sync(false); }
    double t_sync = (now_ms() - t0) / 100;
    printf("\ninfo: UV memset %.3f ms/frame, 2x DMA_BUF_SYNC %.3f ms/frame"
           "  -> %.3f ms/frame removable\n", t_memset, t_sync, t_memset + t_sync);

    printf("\n%s (%d failures)\n", g_fail ? "PROBE9 FAILED" : "PROBE9 OK", g_fail);
    return g_fail ? 1 : 0;
}
