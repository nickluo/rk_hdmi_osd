/*
 * hw::Sof2Epoch implementation - see hw/sof2epoch.h for the model.
 */
#include "sof2epoch.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <ctime>
#include <sys/timex.h>

namespace hw {
namespace {

#define S2E(...) do { fprintf(stderr, "[E] sof2epoch: " __VA_ARGS__); fflush(stderr); } while (0)

int64_t ts_ns(const struct timespec *t)
{
    return (int64_t)t->tv_sec * 1000000000 + t->tv_nsec;
}

} // namespace

Sof2Epoch::~Sof2Epoch()
{
    stop();
}

void Sof2Epoch::sample_pair(Sample *s)
{
    /* sandwich: the realtime read sits between two raw reads, the raw
     * midpoint cancels the skew of reading two clocks in sequence */
    struct timespec m1, r, m2;
    clock_gettime(CLOCK_MONOTONIC_RAW, &m1);
    clock_gettime(CLOCK_REALTIME,      &r);
    clock_gettime(CLOCK_MONOTONIC_RAW, &m2);
    s->raw_ns = (ts_ns(&m1) + ts_ns(&m2)) / 2;
    s->rt_ns  = ts_ns(&r);
}

bool Sof2Epoch::start(unsigned cal_period_s)
{
    stop();
    if (cal_period_s == 0)
        cal_period_s = 10;
    m_period_s = cal_period_s;

    Sample s;
    sample_pair(&s);

    pthread_mutex_lock(&m_lock);
    m_rt_anchor = s.rt_ns;
    m_raw_anchor = s.raw_ns;
    m_recal = 0;
    /* the kernel's current NTP frequency correction is the best rate prior;
     * it is exactly 1.0 (no slew active) when no daemon is disciplining */
    m_rate = 1.0;
    struct timex tx = {};
    if (adjtimex(&tx) >= 0)
        m_rate = 1.0 + (double)tx.freq / 65536.0 / 1e6;
    pthread_mutex_unlock(&m_lock);

    if (pthread_create(&m_thread, nullptr, thread_, this) != 0) {
        /* conversion still works with the static initial anchors */
        S2E("calibration thread create: %s (static model only)\n",
            strerror(errno));
        return true;
    }
    m_running = true;
    return true;
}

void Sof2Epoch::stop()
{
    if (!m_thread)
        return;
    pthread_mutex_lock(&m_lock);
    m_running = false;
    pthread_cond_broadcast(&m_wake);
    pthread_mutex_unlock(&m_lock);
    pthread_join(m_thread, nullptr);
    m_thread = pthread_t{};
}

void *Sof2Epoch::thread_(void *arg)
{
    Sof2Epoch *self = static_cast<Sof2Epoch *>(arg);

    for (;;) {
        pthread_mutex_lock(&self->m_lock);
        if (self->m_running) {
            struct timespec until;
            clock_gettime(CLOCK_REALTIME, &until);
            until.tv_sec += self->m_period_s;
            int rc = 0;
            while (rc != ETIMEDOUT && self->m_running)
                rc = pthread_cond_timedwait(&self->m_wake, &self->m_lock,
                                            &until);
        }
        const bool run = self->m_running;
        pthread_mutex_unlock(&self->m_lock);
        if (!run)
            break;

        Sample s;
        sample_pair(&s);
        self->recalibrate_(s);
    }
    return nullptr;
}

void Sof2Epoch::recalibrate_(const Sample &s)
{
    pthread_mutex_lock(&m_lock);
    const int64_t d_raw = s.raw_ns - m_raw_anchor;
    const int64_t d_rt  = s.rt_ns  - m_rt_anchor;
    if (d_raw > 1000000000LL) {    /* need >= 1 s of elapsed raw time */
        const double measured = (double)d_rt / (double)d_raw;
        /* > 500 ppm outlier = wall clock step: swap anchors, keep rate */
        if (std::fabs(measured - m_rate) < 500e-6)
            m_rate += 0.3 * (measured - m_rate);
        m_rt_anchor = s.rt_ns;
        m_raw_anchor = s.raw_ns;
        m_recal++;
    }
    pthread_mutex_unlock(&m_lock);
}

int64_t Sof2Epoch::to_epoch_ns(int64_t raw_ns) const
{
    pthread_mutex_lock(&m_lock);
    const int64_t rt = m_rt_anchor;
    const int64_t ra = m_raw_anchor;
    const double k = m_rate;
    pthread_mutex_unlock(&m_lock);

    if (rt == 0)
        return 0;    /* not started */
    /* big numbers stay int64; only the (possibly negative) delta is double */
    return rt + (int64_t)llround(k * (double)(raw_ns - ra));
}

double Sof2Epoch::rate() const
{
    pthread_mutex_lock(&m_lock);
    const double r = m_rate;
    pthread_mutex_unlock(&m_lock);
    return r;
}

uint64_t Sof2Epoch::recalibrations() const
{
    pthread_mutex_lock(&m_lock);
    const uint64_t n = m_recal;
    pthread_mutex_unlock(&m_lock);
    return n;
}

} // namespace hw
