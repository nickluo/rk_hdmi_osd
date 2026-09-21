/*
 * hw::Sof2Epoch - maps camera SOF timestamps from CLOCK_MONOTONIC_RAW (the
 * domain the rkcif driver stamps its capture buffers in) to CLOCK_REALTIME
 * (Epoch).
 *
 * The two clocks differ by a step-prone offset (settimeofday / NTP steps /
 * RTC resync) and, while an NTP daemon slews the system clock, by a small
 * frequency error (typically < 50 ppm). The mapping is therefore tracked as
 * the linear model
 *
 *     epoch_ns = rt_anchor + rate * (raw_ns - raw_anchor)
 *
 * anchored on sandwiched clock samples (RAW / REALTIME / RAW, which cancels
 * the read-order skew) and refreshed by a background thread: the rate is
 * estimated from the elapsed distance between anchor pairs, exponential-
 * smoothed, and never updated across a detected wall-clock step (a sudden
 * rate outlier > 500 ppm means the clock was set, not that the crystal
 * drifted). Anchors live as int64 ns and only the small delta goes through
 * double, so the 1.7e18 magnitude of realtime ns never loses precision.
 *
 * Error budget: initial sampling < 0.1 us; between calibrations the residual
 * is rate_change * period, well under the V4L2 usec timestamp truncation and
 * the driver's SOF interrupt jitter.
 */
#pragma once

#include <cstdint>
#include <pthread.h>

namespace hw {

class Sof2Epoch {
public:
    Sof2Epoch() = default;
    ~Sof2Epoch();
    Sof2Epoch(const Sof2Epoch &) = delete;
    Sof2Epoch &operator=(const Sof2Epoch &) = delete;

    /* initial calibration + background recalibration thread;
     * cal_period_s is the anchor refresh interval (default 10 s) */
    bool start(unsigned cal_period_s = 10);
    void stop();
    bool valid() const { return m_rt_anchor != 0; }

    /* raw CLOCK_MONOTONIC_RAW ns -> CLOCK_REALTIME ns; 0 when not started */
    int64_t to_epoch_ns(int64_t raw_ns) const;

    /* diagnostics */
    double rate() const;                    /* measured REALTIME/RAW ratio */
    uint64_t recalibrations() const;

private:
    struct Sample { int64_t rt_ns; int64_t raw_ns; };

    static void sample_pair(Sample *s);
    void recalibrate_(const Sample &s);
    static void *thread_(void *arg);

    mutable pthread_mutex_t m_lock = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t m_wake = PTHREAD_COND_INITIALIZER;
    pthread_t m_thread = {};
    bool m_running = false;
    unsigned m_period_s = 10;
    int64_t m_rt_anchor = 0;
    int64_t m_raw_anchor = 0;
    double m_rate = 1.0;
    uint64_t m_recal = 0;
};

} // namespace hw
