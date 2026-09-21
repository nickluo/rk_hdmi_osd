/*
 * hw::HdmiDisplay implementation - see hw/display.h for the API contract.
 */
#include "display.h"

#include <cstdio>
#include <cstring>
#include <cerrno>
#include <ctime>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <sys/mman.h>
#include <xf86drm.h>
#include <drm_fourcc.h>

namespace hw {
namespace {

#define DISPE(...) do { fprintf(stderr, "[E] display: " __VA_ARGS__); fflush(stderr); } while (0)

void page_flip_cb(int, unsigned, unsigned, unsigned, void *user)
{
    *(bool *)user = false;
}

} // namespace

HdmiDisplay::~HdmiDisplay()
{
    close();
}

/* ------------------------------------------------------------------ */
/* setup                                                               */
/* ------------------------------------------------------------------ */
bool HdmiDisplay::open(const Config &cfg)
{
    close();
    m_w = cfg.width;
    m_h = cfg.height;

    m_fd = ::open(cfg.dev, O_RDWR | O_CLOEXEC | O_NONBLOCK);
    if (m_fd < 0) {
        DISPE("open %s: %s\n", cfg.dev, strerror(errno));
        return false;
    }
    drmSetMaster(m_fd);

    if (!pick_connector_and_mode()) {
        close();
        return false;
    }
    for (int i = 0; i < 2; i++) {
        /* DRM XRGB8888 memory is B,G,R,X == PixelFormat::BGRX8888 */
        if (!create_fb(m_fb[i], DRM_FORMAT_XRGB8888)) {
            close();
            return false;
        }
        memset(m_fb[i].map, 0, m_fb[i].map_size);      /* letterbox bars */
    }
    if (drmModeSetCrtc(m_fd, m_crtc_id, m_fb[0].fb_id, 0, 0,
                       &m_conn_id, 1, &m_mode) < 0) {
        DISPE("SetCrtc: %s\n", strerror(errno));
        close();
        return false;
    }
    m_back = 1;

    if (cfg.overlay && !setup_overlay(cfg)) {
        close();
        return false;
    }
    fprintf(stdout, "[I] display: HDMI %ux%u@%uHz, crtc %u, ovl plane %u\n",
            m_mode.hdisplay, m_mode.vdisplay, m_mode.vrefresh,
            m_crtc_id, m_ovl_plane_id);
    fflush(stdout);
    return true;
}

bool HdmiDisplay::pick_connector_and_mode()
{
    drmModeRes *res = drmModeGetResources(m_fd);
    if (!res) {
        DISPE("GetResources: %s\n", strerror(errno));
        return false;
    }
    drmModeConnector *conn = nullptr;
    for (int i = 0; i < res->count_connectors; i++) {
        conn = drmModeGetConnector(m_fd, res->connectors[i]);
        if (conn && conn->connection == DRM_MODE_CONNECTED &&
            conn->connector_type == DRM_MODE_CONNECTOR_HDMIA)
            break;
        if (conn) drmModeFreeConnector(conn);
        conn = nullptr;
    }
    if (!conn) {
        DISPE("no connected HDMI connector\n");
        drmModeFreeResources(res);
        return false;
    }
    m_conn_id = conn->connector_id;

    for (int i = 0; i < conn->count_modes; i++) {
        drmModeModeInfo *m = &conn->modes[i];
        if (m->hdisplay == m_w && m->vdisplay == m_h &&
            !(m->flags & DRM_MODE_FLAG_INTERLACE)) {
            m_mode = *m;
            break;
        }
    }
    if (conn->encoder_id) {
        drmModeEncoder *enc = drmModeGetEncoder(m_fd, conn->encoder_id);
        if (enc) { m_crtc_id = enc->crtc_id; drmModeFreeEncoder(enc); }
    }
    if (!m_crtc_id && res->count_crtcs > 0)
        m_crtc_id = res->crtcs[0];
    for (int i = 0; i < res->count_crtcs; i++)
        if (res->crtcs[i] == m_crtc_id) { m_crtc_index = i; break; }

    drmModeFreeConnector(conn);
    drmModeFreeResources(res);

    if (!m_mode.hdisplay) {
        DISPE("no %dx%d progressive mode\n", m_w, m_h);
        return false;
    }
    if (!m_crtc_id) {
        DISPE("no crtc\n");
        return false;
    }
    return true;
}

bool HdmiDisplay::create_fb(Framebuffer &fb, uint32_t fourcc)
{
    struct drm_mode_create_dumb create = {};
    create.width = m_w;
    create.height = m_h;
    create.bpp = 32;
    if (drmIoctl(m_fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) < 0) {
        DISPE("CREATE_DUMB: %s\n", strerror(errno));
        return false;
    }
    fb.gem_handle = create.handle;
    m_pitch = create.pitch;

    uint32_t handles[4] = { create.handle, 0, 0, 0 };
    uint32_t pitches[4] = { create.pitch, 0, 0, 0 };
    uint32_t offsets[4] = { 0, 0, 0, 0 };
    if (drmModeAddFB2(m_fd, m_w, m_h, fourcc, handles, pitches, offsets,
                      &fb.fb_id, 0) < 0) {
        DISPE("AddFB2: %s\n", strerror(errno));
        return false;
    }
    struct drm_mode_map_dumb mreq = {};
    mreq.handle = create.handle;
    if (drmIoctl(m_fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) < 0) {
        DISPE("MAP_DUMB: %s\n", strerror(errno));
        return false;
    }
    fb.map_size = create.size;
    fb.map = (uint32_t *)mmap(nullptr, create.size, PROT_READ | PROT_WRITE,
                              MAP_SHARED, m_fd, mreq.offset);
    if (fb.map == MAP_FAILED) {
        fb.map = nullptr;
        DISPE("mmap: %s\n", strerror(errno));
        return false;
    }
    if (drmPrimeHandleToFD(m_fd, create.handle, DRM_CLOEXEC | DRM_RDWR,
                           &fb.prime_fd) < 0) {
        DISPE("PrimeHandleToFD: %s\n", strerror(errno));
        return false;
    }
    return true;
}

ImageDesc HdmiDisplay::back_image() const
{
    ImageDesc d;
    d.dma_fd = m_fb[m_back].prime_fd;
    d.va = m_fb[m_back].map;
    d.size = m_fb[m_back].map_size;
    d.width = m_w;
    d.height = m_h;
    d.stride = m_pitch;
    d.format = PixelFormat::BGRX8888;
    return d;
}

ImageDesc HdmiDisplay::overlay_image() const
{
    ImageDesc d;
    if (!m_ovl_plane_id) return d;
    d.dma_fd = m_ovl.prime_fd;
    d.va = m_ovl.map;
    d.size = m_ovl.map_size;
    d.width = m_w;
    d.height = m_h;
    d.stride = m_pitch;
    d.format = PixelFormat::RGBA8888;
    return d;
}

bool HdmiDisplay::setup_overlay(const Config &cfg)
{
    drmModePlaneRes *pres = drmModeGetPlaneResources(m_fd);
    if (!pres) {
        DISPE("GetPlaneResources: %s\n", strerror(errno));
        return false;
    }
    for (uint32_t i = 0; i < pres->count_planes && !m_ovl_plane_id; i++) {
        drmModePlane *pl = drmModeGetPlane(m_fd, pres->planes[i]);
        if (!pl) continue;
        const bool usable = (pl->possible_crtcs & (1u << m_crtc_index)) &&
                            pl->crtc_id == 0;
        bool has_abgr = false;
        for (uint32_t f = 0; f < pl->count_formats; f++)
            if (pl->formats[f] == DRM_FORMAT_ABGR8888) { has_abgr = true; break; }
        if (usable && has_abgr)
            m_ovl_plane_id = pl->plane_id;
        drmModeFreePlane(pl);
    }
    drmModeFreePlaneResources(pres);
    if (!m_ovl_plane_id) {
        DISPE("no free overlay plane\n");
        return false;
    }
    /* ABGR8888 memory order is R,G,B,A - what osd::Layer writes */
    if (!create_fb(m_ovl, DRM_FORMAT_ABGR8888))
        return false;
    memset(m_ovl.map, 0, m_ovl.map_size);

    uint32_t prop_alpha = 0, prop_blend = 0, prop_zpos = 0;
    drmModeObjectProperties *props =
        drmModeObjectGetProperties(m_fd, m_ovl_plane_id, DRM_MODE_OBJECT_PLANE);
    if (props) {
        for (uint32_t i = 0; i < props->count_props; i++) {
            drmModePropertyRes *p = drmModeGetProperty(m_fd, props->props[i]);
            if (!p) continue;
            if (!strcmp(p->name, "alpha")) prop_alpha = p->prop_id;
            else if (!strcmp(p->name, "pixel blend mode")) prop_blend = p->prop_id;
            else if (!strcmp(p->name, "zpos")) prop_zpos = p->prop_id;
            drmModeFreeProperty(p);
        }
        drmModeFreeObjectProperties(props);
    }
    /* set once: a per-frame property ioctl blocks until vblank and costs
     * ~1.6fps, so the lock pulse is done with per-pixel alpha instead */
    if (prop_zpos)
        drmModeObjectSetProperty(m_fd, m_ovl_plane_id, DRM_MODE_OBJECT_PLANE,
                                 prop_zpos, 1);
    if (prop_blend)
        drmModeObjectSetProperty(m_fd, m_ovl_plane_id, DRM_MODE_OBJECT_PLANE,
                                 prop_blend, 1);                  /* Coverage */
    if (prop_alpha)
        drmModeObjectSetProperty(m_fd, m_ovl_plane_id, DRM_MODE_OBJECT_PLANE,
                                 prop_alpha, (uint64_t)cfg.overlay_alpha * 257);

    if (drmModeSetPlane(m_fd, m_ovl_plane_id, m_crtc_id, m_ovl.fb_id, 0,
                        0, 0, m_w, m_h, 0, 0, m_w << 16, m_h << 16) < 0) {
        DISPE("SetPlane: %s\n", strerror(errno));
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------ */
/* presentation                                                        */
/* ------------------------------------------------------------------ */
bool HdmiDisplay::wait_flip(int timeout_ms)
{
    if (!m_flip_pending) return true;

    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    drmEventContext ctx = {};
    ctx.version = 2;
    ctx.page_flip_handler = page_flip_cb;

    /* poll() can wake on a hotplug uevent, which drmHandleEvent silently
     * drops without running the flip callback - keep draining until the
     * flip event itself retires m_flip_pending or the budget runs out */
    while (m_flip_pending) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        const int left = timeout_ms -
            (int)((now.tv_sec - t0.tv_sec) * 1000 +
                  (now.tv_nsec - t0.tv_nsec) / 1000000);
        if (left <= 0) {
            DISPE("flip poll timeout\n");
            return false;
        }
        struct pollfd p = { m_fd, POLLIN, 0 };
        if (poll(&p, 1, left) < 0 && errno != EINTR) {
            DISPE("flip poll: %s\n", strerror(errno));
            return false;
        }
        /* drmHandleEvent does its own read(): reading the fd here first
         * would steal the event and leave it blocked forever in drm_read() */
        drmHandleEvent(m_fd, &ctx);
    }
    return true;
}

bool HdmiDisplay::present()
{
    /* The page-flip event fires at vblank, but the kernel only retires the
     * atomic commit in its worker afterwards; a non-blocking commit issued
     * in that window returns EBUSY. The rk3576 VOP2 additionally returns a
     * transient ENOSPC from its atomic check when window/bandwidth state is
     * mid-transition. Wait both out instead of treating them as fatal -
     * that is what aborted long runs. */
    for (int attempt = 0; attempt < 24; attempt++) {
        if (drmModePageFlip(m_fd, m_crtc_id, m_fb[m_back].fb_id,
                            DRM_MODE_PAGE_FLIP_EVENT, &m_flip_pending) == 0) {
            m_flip_pending = true;
            m_back ^= 1;
            return true;
        }
        if (errno != EBUSY && errno != ENOSPC) {
            DISPE("PageFlip: %s\n", strerror(errno));
            return false;
        }
        if (m_flip_pending) {
            if (!wait_flip(100)) return false;
        } else {
            usleep(500);
        }
    }
    DISPE("PageFlip: EBUSY after 24 retries\n");
    return false;
}

/* ------------------------------------------------------------------ */
/* teardown                                                            */
/* ------------------------------------------------------------------ */
void HdmiDisplay::destroy_fb(Framebuffer &fb)
{
    if (fb.prime_fd >= 0) ::close(fb.prime_fd);
    if (fb.map) munmap(fb.map, fb.map_size);
    if (fb.fb_id) drmModeRmFB(m_fd, fb.fb_id);
    if (fb.gem_handle) {
        struct drm_mode_destroy_dumb d = {};
        d.handle = fb.gem_handle;
        drmIoctl(m_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &d);
    }
    fb = Framebuffer();
}

void HdmiDisplay::close()
{
    if (m_fd < 0) return;

    wait_flip(200);
    if (m_ovl_plane_id) {
        drmModeSetPlane(m_fd, m_ovl_plane_id, m_crtc_id, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0);
        destroy_fb(m_ovl);
        m_ovl_plane_id = 0;
    }
    for (auto &fb : m_fb)
        destroy_fb(fb);

    ::close(m_fd);
    m_fd = -1;
    m_conn_id = m_crtc_id = 0;
    m_flip_pending = false;
    m_back = 1;
}

} // namespace hw
