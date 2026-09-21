/*
 * test_cam - camera-only smoke test: media-graph discovery, SC233HGS Y10
 * capture, SOF(CLOCK_MONOTONIC_RAW)->Epoch timestamps.
 *
 * No DRM/HDMI needed - safe first test on a headless board.
 *
 * Build: make LIBRGA=... test_cam
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <getopt.h>

#include "hw/camera.h"

int main(int argc, char **argv)
{
    int cam_idx = 0, frames_wanted = 180;
    bool use_isp = false;
    static const struct option longopts[] = {
        {"cam", required_argument, 0, 'c'},
        {"frames", required_argument, 0, 'n'},
        {"isp", no_argument, 0, 'i'},
        {0, 0, 0, 0},
    };
    int ch;
    while ((ch = getopt_long(argc, argv, "c:n:i", longopts, NULL)) != -1) {
        switch (ch) {
        case 'c': cam_idx = atoi(optarg); break;
        case 'n': frames_wanted = atoi(optarg); break;
        case 'i': use_isp = true; break;
        default: return 1;
        }
    }

    hw::MipiCamera cam;
    hw::CameraBase::Config cfg;
    cfg.sensor = use_isp ? hw::CameraBase::Sensor::Sc233hgsIsp
                         : hw::CameraBase::Sensor::Sc233hgs;
    cfg.camera_index = cam_idx;
    if (!cam.open(cfg))
        return 1;

    uint64_t last_sof = 0;
    double dt_acc = 0, jit_acc = 0, jit_sq = 0;
    int dt_n = 0, dt_max_us = 0, dt_min_us = 1 << 30;
    int jit_n = 0, jit_min = 1 << 30, jit_max = 0, skips = 0;
    unsigned vmin = 65535, vmax = 0;

    for (int i = 0; i < frames_wanted; i++) {
        hw::CameraBase::Frame f = cam.capture(2000);
        if (!f.valid()) {
            printf("[E] capture failed at frame %d\n", i);
            return 1;
        }

        if (i == 0 && f.image().va && f.image().format == hw::PixelFormat::Y10) {
            /* sample the raw 16-bit words to characterise the layout */
            const unsigned short *p = (const unsigned short *)f.image().va;
            size_t n = f.image().size / 2;
            for (size_t k = 0; k < n; k += 97) {
                if (p[k] < vmin) vmin = p[k];
                if (p[k] > vmax) vmax = p[k];
            }
            printf("[I] frame0 u16 word range: %u..%u (stride=%d, %s layout)\n",
                   vmin, vmax, f.image().stride,
                   f.image().stride < f.image().width * 2
                       ? "packed 10-bit" : "16-bit container");

            /* LE-bitstream decode smoothness: a correct decode of a real
             * scene has low neighbour deltas; garbage decode looks random */
            if (f.image().stride < f.image().width * 2) {
                const unsigned char *row =
                    (const unsigned char *)f.image().va + f.image().stride * 600;
                const int w = f.image().width;
                unsigned char y8[2048];
                for (int g = 0; g < w / 4; g++) {
                    const unsigned char *s = row + g * 5;
                    const unsigned p0 = s[0] | ((s[1] & 0x03u) << 8);
                    const unsigned p1 = (s[1] >> 2) | ((s[2] & 0x0fu) << 6);
                    const unsigned p2 = (s[2] >> 4) | ((s[3] & 0x3fu) << 4);
                    const unsigned p3 = (s[3] >> 6) | ((unsigned)s[4] << 2);
                    y8[g * 4 + 0] = (unsigned char)(p0 >> 2);
                    y8[g * 4 + 1] = (unsigned char)(p1 >> 2);
                    y8[g * 4 + 2] = (unsigned char)(p2 >> 2);
                    y8[g * 4 + 3] = (unsigned char)(p3 >> 2);
                }
                long acc = 0;
                for (int x = 1; x < w; x++)
                    acc += abs((int)y8[x] - (int)y8[x - 1]);
                printf("[I] LE-bitstream decode row600 mean |dy| = %.2f "
                       "(scene ~<10, random ~85)\n",
                       (double)acc / (w - 1));
            }
        }

        if (f.sof_raw_ns && last_sof) {
            int dt_us = (int)((f.sof_raw_ns - last_sof) / 1000);
            dt_acc += dt_us;
            dt_n++;
            if (dt_us > dt_max_us) dt_max_us = dt_us;
            if (dt_us < dt_min_us) dt_min_us = dt_us;
            /* jitter stats over clean single-frame intervals */
            if (dt_us > 15000 && dt_us < 19000) {
                jit_acc += dt_us;
                jit_n++;
                jit_sq += (double)dt_us * dt_us;
                if (dt_us < jit_min) jit_min = dt_us;
                if (dt_us > jit_max) jit_max = dt_us;
            } else {
                skips++;
            }
        }
        last_sof = f.sof_raw_ns;

        if (i == 0) {
            struct timespec r;
            clock_gettime(CLOCK_REALTIME, &r);
            double age_ms = (r.tv_sec * 1e3 + r.tv_nsec / 1e6)
                          - (double)f.epoch_ns / 1e6;
            printf("[I] frame0: seq=%u sof_raw=%llu ns epoch=%llu ns "
                   "age=%.2f ms rate=%+.2f ppm\n",
                   f.sequence, (unsigned long long)f.sof_raw_ns,
                   (unsigned long long)f.epoch_ns, age_ms,
                   (cam.clock().rate() - 1.0) * 1e6);
        }
        f.release();
    }

    const int n = dt_n ? dt_n : 1;
    printf("[I] %d frames: mean dt=%.3f ms (%.2f fps), skips=%d\n",
           frames_wanted, dt_acc / n / 1000.0, 1e6 / (dt_acc / n), skips);
    if (jit_n > 1) {
        const double mean = jit_acc / jit_n;
        const double std = sqrt(jit_sq / jit_n - mean * mean);
        printf("[I] clean SOF intervals: n=%d mean=%.3f us (target 16666.667), "
               "std=%.1f us, min=%d, max=%d\n",
               jit_n, mean, std, jit_min, jit_max);
    }
    return 0;
}
