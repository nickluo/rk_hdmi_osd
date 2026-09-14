/* probe10: raster micro-benchmark for the OSD fill paths.
 *
 * User-space CPU in osd_demo is ~0.41 ms/frame and almost all of it is
 * osd::Layer fill/clear. Those are scalar 32-bit store loops; GCC at -O2
 * does not vectorize them. Measure what NEON (and plain wider stores) buy
 * before changing library code.
 */
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <arm_neon.h>

static double now_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static const int W = 720, H = 480;
static uint32_t canvas[W * H];

/* current libosd inner loop */
static void fill_scalar(int x, int y, int w, int h, uint32_t p)
{
    for (int j = 0; j < h; j++) {
        uint32_t *row = &canvas[(size_t)(y + j) * W + x];
        for (int i = 0; i < w; i++) row[i] = p;
    }
}

static void fill_neon(int x, int y, int w, int h, uint32_t p)
{
    const uint32x4_t v = vdupq_n_u32(p);
    for (int j = 0; j < h; j++) {
        uint32_t *row = &canvas[(size_t)(y + j) * W + x];
        int i = 0;
        for (; i + 16 <= w; i += 16) {
            vst1q_u32(row + i, v);
            vst1q_u32(row + i + 4, v);
            vst1q_u32(row + i + 8, v);
            vst1q_u32(row + i + 12, v);
        }
        for (; i + 4 <= w; i += 4) vst1q_u32(row + i, v);
        for (; i < w; i++) row[i] = p;
    }
}

/* glyph-cell composite: skip transparent, copy opaque (the common case
 * after the outline fix - an outlined glyph is strictly alpha 0 or 255) */
static void blit_scalar(uint32_t *dst, const uint32_t *src, int n)
{
    for (int i = 0; i < n; i++) {
        const uint32_t sp = src[i];
        if (sp >> 24) dst[i] = sp;
    }
}

static void blit_neon(uint32_t *dst, const uint32_t *src, int n)
{
    int i = 0;
    for (; i + 4 <= n; i += 4) {
        const uint32x4_t s = vld1q_u32(src + i);
        const uint32x4_t d = vld1q_u32(dst + i);
        /* mask = alpha != 0, per lane */
        const uint32x4_t m = vtstq_u32(s, vdupq_n_u32(0xFF000000u));
        vst1q_u32(dst + i, vbslq_u32(m, s, d));
    }
    for (; i < n; i++) { const uint32_t sp = src[i]; if (sp >> 24) dst[i] = sp; }
}

template <typename F>
static double bench(const char *name, F f, int iters, double ref = 0)
{
    f();                                     /* warm the cache */
    double t0 = now_ms();
    for (int i = 0; i < iters; i++) f();
    double ms = (now_ms() - t0) / iters;
    if (ref > 0)
        printf("  %-26s %7.3f ms  (%.2fx)\n", name, ms, ref / ms);
    else
        printf("  %-26s %7.3f ms\n", name, ms);
    return ms;
}

int main()
{
    printf("raster micro-benchmark, %dx%d canvas\n\n", W, H);

    /* a detection-box footprint clear: ~200x150 */
    printf("clear 200x150 (detect box footprint):\n");
    double s1 = bench("scalar u32 loop", [] { fill_scalar(100, 100, 200, 150, 0); }, 2000);
    bench("NEON vst1q x4", [] { fill_neon(100, 100, 200, 150, 0); }, 2000, s1);
    bench("memset per row", [] {
        for (int j = 0; j < 150; j++) memset(&canvas[(size_t)(100 + j) * W + 100], 0, 200 * 4);
    }, 2000, s1);

    printf("\nfill 150x12 (bar) x2 + 64x34 plate x2:\n");
    double s2 = bench("scalar u32 loop", [] {
        fill_scalar(20, 300, 150, 12, 0xFF5AE65A); fill_scalar(20, 340, 150, 12, 0xFF5AE65A);
        fill_scalar(176, 289, 64, 34, 0xFF000000); fill_scalar(176, 329, 64, 34, 0xFF000000);
    }, 5000);
    bench("NEON vst1q x4", [] {
        fill_neon(20, 300, 150, 12, 0xFF5AE65A); fill_neon(20, 340, 150, 12, 0xFF5AE65A);
        fill_neon(176, 289, 64, 34, 0xFF000000); fill_neon(176, 329, 64, 34, 0xFF000000);
    }, 5000, s2);

    printf("\nglyph row composite (20 px cell x 36 rows x 12 chars):\n");
    static uint32_t cell[36 * 20];
    for (int i = 0; i < 36 * 20; i++) cell[i] = (i % 3) ? 0xFFEBEBEB : 0;
    double s3 = bench("scalar branch", [] {
        for (int c = 0; c < 12; c++)
            for (int r = 0; r < 36; r++)
                blit_scalar(&canvas[(size_t)(200 + r) * W + 40 + c * 16], &cell[r * 20], 20);
    }, 5000);
    bench("NEON select", [] {
        for (int c = 0; c < 12; c++)
            for (int r = 0; r < 36; r++)
                blit_neon(&canvas[(size_t)(200 + r) * W + 40 + c * 16], &cell[r * 20], 20);
    }, 5000, s3);

    printf("\nfull-canvas clear (720x480):\n");
    double s4 = bench("scalar u32 loop", [] { fill_scalar(0, 0, W, H, 0); }, 500);
    bench("NEON vst1q x4", [] { fill_neon(0, 0, W, H, 0); }, 500, s4);
    bench("memset whole", [] { memset(canvas, 0, sizeof canvas); }, 500, s4);
    return 0;
}
