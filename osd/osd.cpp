/*
 * libosd implementation - see osd/osd.h for the API contract.
 */
#include "osd.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cerrno>
#include <new>
#include <utility>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/dma-heap.h>
#include <linux/dma-buf.h>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define OSD_NEON 1
#endif

namespace osd {
namespace {

int xioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do { r = ioctl(fd, req, arg); } while (r < 0 && errno == EINTR);
    return r;
}

/* RGBA_8888 as RGA names it: memory bytes R,G,B,A
 * => LE 32-bit value = (A<<24)|(B<<16)|(G<<8)|R */
inline uint32_t pack(Color c)
{
    return ((uint32_t)c.a << 24) | ((uint32_t)c.b << 16) |
           ((uint32_t)c.g << 8) | c.r;
}

/* straight (non-premultiplied) alpha SRC-OVER of two packed RGBA pixels */
inline uint32_t blend_over(uint32_t s, uint32_t d)
{
    const uint32_t sa = s >> 24;
    if (sa == 255) return s;
    if (sa == 0)   return d;
    const uint32_t da = d >> 24;
    const uint32_t out_a = sa + da * (255 - sa) / 255;
    if (out_a == 0) return 0;
    const uint32_t w_s = sa * 255 / out_a;
    const uint32_t w_d = 255 - w_s;
    uint32_t out = out_a << 24;
    for (int sh = 0; sh <= 16; sh += 8)
        out |= ((((s >> sh) & 0xFF) * w_s + ((d >> sh) & 0xFF) * w_d) / 255) << sh;
    return out;
}

inline Rect clip(Rect r, int w, int h)
{
    if (r.x < 0) { r.w += r.x; r.x = 0; }
    if (r.y < 0) { r.h += r.y; r.y = 0; }
    if (r.x + r.w > w) r.w = w - r.x;
    if (r.y + r.h > h) r.h = h - r.y;
    if (r.w < 0) r.w = 0;
    if (r.h < 0) r.h = 0;
    return r;
}

/* 3.9x over the scalar store loop at -O2 (measured, probe10) */
inline void fill_row(uint32_t *row, uint32_t p, int n)
{
    int i = 0;
#ifdef OSD_NEON
    const uint32x4_t v = vdupq_n_u32(p);
    for (; i + 16 <= n; i += 16) {
        vst1q_u32(row + i, v);
        vst1q_u32(row + i + 4, v);
        vst1q_u32(row + i + 8, v);
        vst1q_u32(row + i + 12, v);
    }
    for (; i + 4 <= n; i += 4) vst1q_u32(row + i, v);
#endif
    for (; i < n; i++) row[i] = p;
}

/* glyph row with alpha in {0,255}: branchless select, 3.4x scalar. GCC
 * cannot auto-vectorize the scalar form because the store is conditional.
 * Only worth it on cached memory - it reads dst back, and an attached DRM
 * dumb buffer is write-combine, where that read costs more than it saves. */
inline void blit_row_binary(uint32_t *dst, const uint32_t *src, int n)
{
    int i = 0;
#ifdef OSD_NEON
    const uint32x4_t amask = vdupq_n_u32(0xFF000000u);
    for (; i + 4 <= n; i += 4) {
        const uint32x4_t s = vld1q_u32(src + i);
        const uint32x4_t d = vld1q_u32(dst + i);
        vst1q_u32(dst + i, vbslq_u32(vtstq_u32(s, amask), s, d));
    }
#endif
    for (; i < n; i++) { const uint32_t sp = src[i]; if (sp >> 24) dst[i] = sp; }
}

} // namespace

/* ------------------------------------------------------------------ */
/* lifecycle                                                           */
/* ------------------------------------------------------------------ */
Layer::~Layer()
{
    shutdown();
}

bool Layer::init(int w, int h, const char *heap_path)
{
    shutdown();
    m_w = w; m_h = h;
    m_size = (size_t)w * h * 4;

    int heap = open(heap_path, O_RDWR);
    if (heap < 0) { perror("osd: open heap"); return false; }
    struct dma_heap_allocation_data ad = {};
    ad.len = m_size;
    ad.fd_flags = O_RDWR | O_CLOEXEC;
    if (xioctl(heap, DMA_HEAP_IOCTL_ALLOC, &ad) < 0) {
        perror("osd: DMA_HEAP_IOCTL_ALLOC");
        close(heap);
        return false;
    }
    close(heap);
    m_fd = ad.fd;

    m_va = mmap(nullptr, m_size, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, 0);
    if (m_va == MAP_FAILED) {
        perror("osd: mmap");
        close(m_fd); m_fd = -1;
        return false;
    }
    m_attached = false;
    return init_i();
}

bool Layer::attach(int dma_fd, void *va, int w, int h, size_t size)
{
    shutdown();
    m_w = w; m_h = h; m_size = size;
    m_fd = dma_fd;
    m_va = va;
    m_attached = true;
    m_wc = true;              /* DRM dumb buffers are mapped write-combine */
    return init_i();
}

bool Layer::init_mem(int w, int h)
{
    shutdown();
    m_w = w; m_h = h;
    m_size = (size_t)w * h * 4;
    m_px = new (std::nothrow) uint32_t[(size_t)w * h];
    if (!m_px) return false;
    m_va = m_px;
    m_fd = -1;
    m_heap = true;
    m_attached = true;        /* shutdown() must not munmap/close anything */
    return init_i();
}

bool Layer::init_auto(int w, int h)
{
    if (init(w, h))
        return true;
    /* dma-heap unavailable (x86 dev machine, container): drop to memory.
     * Any consumer that needs a dmabuf (RGA import) must use init(). */
    return init_mem(w, h);
}

bool Layer::init_i()
{
    m_px = (uint32_t *)m_va;
    memset(m_px, 0, m_size);
    m_frames = 0;
    m_dirty = true;
    return true;
}

void Layer::shutdown()
{
    for (size_t s = 0; s < m_sets_used; s++)
        for (int i = 0; i < 95; i++)
            if (m_sets[s].glyph[i]) { delete m_sets[s].glyph[i]; m_sets[s].glyph[i] = nullptr; }
    m_sets_used = 0;
    m_cache_count = 0;
    if (m_base) { delete[] m_base; m_base = nullptr; }
    if (m_heap) {
        delete[] m_px;
    } else if (!m_attached) {
        if (m_va) munmap(m_va, m_size);
        if (m_fd >= 0) close(m_fd);
    }
    m_va = nullptr; m_px = nullptr; m_fd = -1;
    m_w = m_h = 0; m_size = 0;
    m_attached = false;
    m_heap = false;
    m_wc = false;
}

void Layer::set_base()
{
    if (!m_base)
        m_base = new uint32_t[(size_t)m_w * m_h];
    memcpy(m_base, m_px, m_size);
    m_dirty = true;
}

void Layer::restore(Rect r)
{
    if (!m_base) { clear(r); return; }
    r = clip(r, m_w, m_h);
    if (r.w <= 0 || r.h <= 0) return;
    for (int y = 0; y < r.h; y++)
        memcpy(&m_px[(size_t)(r.y + y) * m_w + r.x],
               &m_base[(size_t)(r.y + y) * m_w + r.x],
               (size_t)r.w * 4);
    m_dirty = true;
}

void Layer::begin_frame()
{
    m_frames++;
}

void Layer::end_frame()
{
    if (!m_dirty || m_fd < 0)
        return;
    /* The canvas is written by the CPU only and read by RGA only, so a clean
     * (SYNC END) is sufficient; no invalidate is ever required.
     * Attached buffers (DRM dumb) are uncached - writes reach RAM directly,
     * cache maintenance would be a wasted ioctl. */
    if (m_attached) {
        m_dirty = false;
        return;
    }
    struct dma_buf_sync s = {};
    s.flags = DMA_BUF_SYNC_RW | DMA_BUF_SYNC_END;
    if (xioctl(m_fd, DMA_BUF_IOCTL_SYNC, &s) < 0)
        perror("osd: DMA_BUF_IOCTL_SYNC");
    m_dirty = false;
}

/* ------------------------------------------------------------------ */
/* glyph cache                                                         */
/* ------------------------------------------------------------------ */
/* glyph canvas is glyph + 2px outline on every side */
static constexpr int OUT = 2;

const Layer::Glyph *Layer::glyph(char c, const TextStyle &st) const
{
    if (c < FONT_FIRST || c >= FONT_FIRST + 95)
        c = '?';
    int idx = c - FONT_FIRST;
    const bool outlined = st.outline;
    const bool small = st.small;

    const int fw = small ? FONT_W_SMALL : FONT_W;
    const int fh = small ? FONT_H_SMALL : FONT_H;
    const int GW = fw + 4, GH = fh + 4;
    const unsigned char *atlas = small ? FONT_ATLAS_SMALL[idx] : FONT_ATLAS[idx];

    /* find or create the glyph set for this (color, outline, size). The
     * outline and size bits travel in color.a bits 0/1 - apps always draw
     * text at a=255 so those bits are free */
    GlyphSet *set = nullptr;
    const uint8_t style_bits = (uint8_t)outlined | ((uint8_t)small << 1);
    for (size_t s = 0; s < m_sets_used; s++) {
        if (m_sets[s].color.r == st.color.r && m_sets[s].color.g == st.color.g &&
            m_sets[s].color.b == st.color.b &&
            (m_sets[s].color.a & 3) == style_bits) {
            set = &m_sets[s];
            break;
        }
    }
    if (!set) {
        if (m_sets_used >= sizeof(m_sets) / sizeof(m_sets[0]))
            set = &m_sets[0];                    /* degrade: reuse first set */
        else
            set = &m_sets[m_sets_used++];
        set->color = st.color;
        set->color.a = (uint8_t)((set->color.a & 0xFC) | style_bits);
        memset(set->has, 0, sizeof(set->has));
        memset(set->glyph, 0, sizeof(set->glyph));
    }
    if (set->has[idx])
        return set->glyph[idx];

    /* render: solid black outline (thresholded dilation - crisp edge, no
     * translucent halo), then the antialiased colored core on top */
    Glyph *g = new Glyph();           /* value-init zeroes px/binary */
    g->cw = GW;
    g->ch = GH;
    const uint32_t core = pack(st.color);
    const uint32_t outline = pack(Color{0, 0, 0, 255});

    if (outlined) {
        for (int dy = -OUT; dy <= OUT; dy++) {
            for (int dx = -OUT; dx <= OUT; dx++) {
                if (dx == 0 && dy == 0) continue;
                for (int gy = 0; gy < fh; gy++) {
                    int ty = gy + OUT + dy, tx0 = OUT + dx;
                    if (ty < 0 || ty >= GH) continue;
                    for (int gx = 0; gx < fw; gx++) {
                        if (atlas[gy * fw + gx] < 48) continue;
                        int tx = tx0 + gx;
                        if (tx < 0 || tx >= GW) continue;
                        g->px[ty][tx] = outline;
                    }
                }
            }
        }
    }
    for (int gy = 0; gy < fh; gy++) {
        for (int gx = 0; gx < fw; gx++) {
            uint8_t a = atlas[gy * fw + gx];
            if (!a) continue;
            /* alpha lives in bits[31:24] of the packed RGBA value */
            const uint32_t src = (core & 0x00FFFFFFu) | ((uint32_t)a << 24);
            uint32_t &dst = g->px[gy + OUT][gx + OUT];
            /* composite over the outline, never overwrite it: assigning the
             * translucent AA core here replaced opaque black with alpha<255,
             * and RGA2-E miscomposites those pixels into a white fringe */
            dst = blend_over(src, dst);
        }
    }
    /* an outlined glyph must end up strictly alpha 0 or 255: it is meant for
     * a canvas consumed by RGA imblend, whose src alpha<255 math is broken.
     * AA pixels too far from the outline to be resolved above get snapped. */
    if (outlined) {
        for (int ty = 0; ty < GH; ty++)
            for (int tx = 0; tx < GW; tx++) {
                const uint32_t a = g->px[ty][tx] >> 24;
                if (a == 0 || a == 255) continue;
                g->px[ty][tx] = a >= 128 ? (g->px[ty][tx] | 0xFF000000u) : 0;
            }
    }
    /* lets draw_text take the branchless NEON path */
    g->binary = 1;
    for (int ty = 0; ty < GH && g->binary; ty++)
        for (int tx = 0; tx < GW; tx++) {
            const uint32_t a = g->px[ty][tx] >> 24;
            if (a != 0 && a != 255) { g->binary = 0; break; }
        }

    set->glyph[idx] = g;
    set->has[idx] = true;
    m_cache_count++;
    return g;
}

/* ------------------------------------------------------------------ */
/* primitives                                                          */
/* ------------------------------------------------------------------ */
void Layer::clear(Rect r)
{
    r = clip(r, m_w, m_h);
    if (r.w <= 0 || r.h <= 0) return;
    for (int y = 0; y < r.h; y++)
        memset(&m_px[(size_t)(r.y + y) * m_w + r.x], 0, (size_t)r.w * 4);
    m_dirty = true;
}

void Layer::clear_all()
{
    memset(m_px, 0, m_size);
    m_dirty = true;
}

void Layer::fill_rect(Rect r, Color c)
{
    r = clip(r, m_w, m_h);
    if (r.w <= 0 || r.h <= 0) return;
    uint32_t p = pack(c);
    for (int y = 0; y < r.h; y++)
        fill_row(&m_px[(size_t)(r.y + y) * m_w + r.x], p, r.w);
    m_dirty = true;
}

void Layer::fill_rect_blend(Rect r, Color c)
{
    r = clip(r, m_w, m_h);
    if (r.w <= 0 || r.h <= 0) return;
    const uint32_t s = pack(c);
    for (int y = 0; y < r.h; y++) {
        uint32_t *row = &m_px[(size_t)(r.y + y) * m_w + r.x];
        for (int x = 0; x < r.w; x++) row[x] = blend_over(s, row[x]);
    }
    m_dirty = true;
}

void Layer::draw_rect(Rect r, Color c, int t)
{
    if (t <= 0) return;
    fill_rect({r.x, r.y, r.w, t}, c);
    fill_rect({r.x, r.y + r.h - t, r.w, t}, c);
    fill_rect({r.x, r.y + t, t, r.h - 2 * t}, c);
    fill_rect({r.x + r.w - t, r.y + t, t, r.h - 2 * t}, c);
}

void Layer::draw_line(int x0, int y0, int x1, int y1, Color c, int t)
{
    /* Bresenham with thickness via small fill_rect stamps */
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        fill_rect({x0 - t / 2, y0 - t / 2, t, t}, c);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void Layer::draw_text(int x, int y, const char *s, const TextStyle &st)
{
    if (!s) return;
    const bool fast = !m_wc;               /* see blit_row_binary */
    const int fw = st.small ? FONT_W_SMALL : FONT_W;
    const int step = fw + st.tracking;
    for (; *s; s++, x += step) {
        const Glyph *g = glyph(*s, st);
        const int GW = g->cw, GH = g->ch;
        /* clip vertically fast-path */
        if (y + GH <= 0 || y >= m_h) continue;
        for (int gy = 0; gy < GH; gy++) {
            int ty = y + gy;
            if (ty < 0 || ty >= m_h) continue;
            int tx0 = x, len = GW;
            int off = 0;
            if (tx0 < 0) { off = -tx0; len += tx0; tx0 = 0; }
            if (tx0 + len > m_w) len = m_w - tx0;
            if (len <= 0) continue;
            const uint32_t *src = &g->px[gy][off];
            uint32_t *dst = &m_px[(size_t)ty * m_w + tx0];
            /* composite, never blit: a glyph cell is mostly transparent and
             * a raw copy would punch holes through whatever it sits on
             * (label plates, bars, previously drawn elements) */
            if (g->binary && fast) {
                blit_row_binary(dst, src, len);
                continue;
            }
            for (int i = 0; i < len; i++) {
                const uint32_t sp = src[i];
                const uint32_t sa = sp >> 24;
                if (sa == 0) continue;
                dst[i] = sa == 255 ? sp : blend_over(sp, dst[i]);
            }
        }
    }
    m_dirty = true;
}

int Layer::text_width(const char *s, const TextStyle &st) const
{
    int n = 0;
    for (; s && *s; s++) n++;
    if (!n) return 0;
    const int fw = st.small ? FONT_W_SMALL : FONT_W;
    return n * fw + (n - 1) * st.tracking;
}

/* ------------------------------------------------------------------ */
/* composite elements                                                  */
/* ------------------------------------------------------------------ */
void Layer::draw_crosshair(int cx, int cy, Color c, int r)
{
    const int gap = r / 2, arm = r + r / 2;
    fill_rect({cx - gap - arm, cy - 1, arm, 2}, c);
    fill_rect({cx + gap,      cy - 1, arm, 2}, c);
    fill_rect({cx - 1, cy - gap - arm, 2, arm}, c);
    fill_rect({cx - 1, cy + gap,      2, arm}, c);
    for (int a = 0; a < 360; a += 3) {
        float rad = a * 3.14159265f / 180.0f;
        int x = cx + (int)(r * cosf(rad));
        int y = cy + (int)(r * sinf(rad));
        if (x >= 0 && y >= 0 && x < m_w && y < m_h)
            m_px[(size_t)y * m_w + x] = pack(c);
    }
    fill_rect({cx - 2, cy - 2, 4, 4}, c);
    m_dirty = true;
}

void Layer::draw_horizon(int cx, int cy, float pitch_deg, float roll_deg,
                         Color c, int px_per_deg, int half_span)
{
    /* pitch ladder, rungs every 5 deg (10 deg rungs longer), rotated by
     * -roll about the centre. Pitch positive = nose up -> the visible
     * ladder slides down, matching a real attitude indicator. */
    const float rad = -roll_deg * 3.14159265f / 180.0f;
    const float cs = cosf(rad), sn = sinf(rad);
    for (int rung = -20; rung <= 20; rung += 5) {
        if (rung == 0)
            continue;               /* the 0 line is the center reticle */
        const float dy = (rung - pitch_deg) * px_per_deg;
        if (fabsf(dy) > 90.f)
            continue;               /* outside the visible window */
        const int len = (rung % 10 == 0) ? half_span : half_span / 2;
        const int x0 = cx + (int)((-len) * cs - dy * sn);
        const int y0 = cy + (int)((-len) * sn + dy * cs);
        const int x1 = cx + (int)(( len) * cs - dy * sn);
        const int y1 = cy + (int)(( len) * sn + dy * cs);
        draw_line(x0, y0, x1, y1, c, 2);
    }
    m_dirty = true;
}

void Layer::draw_home_arrow(int cx, int cy, float angle_deg, Color c, int size)
{
    /* arrow pointing up in local frame, rotated clockwise by angle */
    const float rad = angle_deg * 3.14159265f / 180.0f;
    const float cs = cosf(rad), sn = sinf(rad);
    auto rot = [&](float lx, float ly) -> std::pair<int, int> {
        return {cx + (int)(lx * cs - ly * sn), cy + (int)(lx * sn + ly * cs)};
    };
    auto [tipx, tipy]   = rot(0, -size);
    auto [lftx, lfty]   = rot(-size / 2, size / 2);
    auto [rgtx, rgty]   = rot(size / 2, size / 2);
    auto [tailx, taily] = rot(0, size / 4);
    draw_line(tipx, tipy, lftx, lfty, c, 2);
    draw_line(tipx, tipy, rgtx, rgty, c, 2);
    draw_line(lftx, lfty, tailx, taily, c, 2);
    draw_line(rgtx, rgty, tailx, taily, c, 2);
}

void Layer::draw_bar(int x, int y, int w, int h, float frac, Color fg, Color bg)
{
    if (frac < 0) frac = 0;
    if (frac > 1) frac = 1;
    fill_rect({x, y, w, h}, bg);
    fill_rect({x + 1, y + 1, (int)((w - 2) * frac), h - 2}, fg);
}

void Layer::draw_warning(int x, int y, const char *s, bool blink_on)
{
    if (!blink_on) return;
    TextStyle st;
    st.color = kRed;
    draw_text(x, y, s, st);
}

Rect Layer::draw_bbox(const BBoxRect &b)
{
    const Rect &r = b.rect;
    const int t = b.locked ? 3 : 2;
    Color c = b.color;
    c.a = b.alpha;          /* plate/brackets; label text below stays opaque */

    /* label text and plate geometry FIRST (the plate is often wider than
     * the box itself, which is exactly why the painted bounds must be
     * tracked instead of estimated from the box) */
    char text[64];
    int n = snprintf(text, sizeof text, "%s %2d%%", b.label,
                     (int)(b.conf * 100 + 0.5f));
    if (b.tag && b.tag[0] && n > 0 && n < (int)sizeof text - 6)
        snprintf(text + n, sizeof text - n, " %s", b.tag);
    TextStyle st;
    st.color = kBlack;
    st.outline = false;
    const int tw = text_width(text, st);
    Rect plate = {r.x, r.y - FONT_H - 8, tw + 12, FONT_H + 6};
    if (plate.y < 0)
        plate.y = r.y + r.h + 2;                    /* flip below the box */
    if (plate.x + plate.w > m_w)                    /* keep inside the right */
        plate.x = r.x + r.w - plate.w;              /* edge (plate > box)   */
    if (plate.x < 0)
        plate.x = 0;

    /* ---- draw ---- */
    if (b.kind == BoxKind::Full) {
        draw_rect(r, c, t);
    } else {
        int L = (r.w < r.h ? r.w : r.h) / 3;
        if (L < 10) L = 10;
        if (L > 36) L = 36;
        fill_rect({r.x, r.y, L, t}, c);
        fill_rect({r.x, r.y, t, L}, c);
        fill_rect({r.x + r.w - L, r.y, L, t}, c);
        fill_rect({r.x + r.w - t, r.y, t, L}, c);
        fill_rect({r.x, r.y + r.h - t, L, t}, c);
        fill_rect({r.x, r.y + r.h - L, t, L}, c);
        fill_rect({r.x + r.w - L, r.y + r.h - t, L, t}, c);
        fill_rect({r.x + r.w - t, r.y + r.h - L, t, L}, c);
        if (b.locked) {
            /* outer echo bracket while locked */
            Rect o = {r.x - 6, r.y - 6, r.w + 12, r.h + 12};
            int L2 = 12;
            fill_rect({o.x, o.y, L2, 2}, c);
            fill_rect({o.x, o.y, 2, L2}, c);
            fill_rect({o.x + o.w - L2, o.y, L2, 2}, c);
            fill_rect({o.x + o.w - 2, o.y, 2, L2}, c);
            fill_rect({o.x, o.y + o.h - 2, L2, 2}, c);
            fill_rect({o.x, o.y + o.h - L2, 2, L2}, c);
            fill_rect({o.x + o.w - L2, o.y + o.h - 2, L2, 2}, c);
            fill_rect({o.x + o.w - 2, o.y + o.h - L2, 2, L2}, c);
        }
    }

    /* pulsing center diamond for locked targets (~3.3Hz) */
    const int cx = r.x + r.w / 2, cy = r.y + r.h / 2;
    if (b.locked && blink_phase()) {
        int s = 8;
        draw_line(cx - s, cy, cx, cy - s, c, 2);
        draw_line(cx, cy - s, cx + s, cy, c, 2);
        draw_line(cx + s, cy, cx, cy + s, c, 2);
        draw_line(cx, cy + s, cx - s, cy, c, 2);
    }

    /* plate: opaque fill, the overlay plane's global alpha (x b.alpha)
     * provides the translucency with exact hardware math */
    if (b.plate_opaque)
        fill_rect(plate, Color{(uint8_t)(c.r / 2), (uint8_t)(c.g / 2),
                               (uint8_t)(c.b / 2), b.alpha});
    else
        fill_rect_blend(plate, Color{(uint8_t)(c.r / 2), (uint8_t)(c.g / 2),
                                     (uint8_t)(c.b / 2), 150});
    {
        Color bc = c;
        bc.a = b.alpha;
        draw_rect(plate, bc, 2);                    /* color-keyed border */
    }
    st.color = Color{255, 255, 255, 255};
    draw_text(plate.x + 6, plate.y + 3, text, st);

    /* ---- exact painted bounds: brackets + echo + diamond + plate ---- */
    int x0 = r.x, y0 = r.y, x1 = r.x + r.w, y1 = r.y + r.h;
    if (b.locked) { x0 -= 8; y0 -= 8; x1 += 8; y1 += 8; }   /* echo frame */
    x0 = std::min(x0, cx - 10);  y0 = std::min(y0, cy - 10);/* diamond    */
    x1 = std::max(x1, cx + 10);  y1 = std::max(y1, cy + 10);
    x0 = std::min(x0, plate.x);          y0 = std::min(y0, plate.y);
    x1 = std::max(x1, plate.x + plate.w); y1 = std::max(y1, plate.y + plate.h);
    return Rect{x0, y0, x1 - x0, y1 - y0};
}

} // namespace osd
