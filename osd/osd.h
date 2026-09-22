/*
 * libosd - lightweight OSD overlay library.
 *
 * Renders FPV/AI-interception style OSD elements (Betaflight-inspired) into
 * an RGBA8888 dma-heap buffer. The buffer is exported as a dmabuf so a 2D
 * engine or a display plane can consume it with zero copy; libosd itself is
 * pure CPU rasterisation and depends on no vendor library.
 *
 * CPU-cost design:
 *  - glyph cache: each (char, color) is pre-rasterized once WITH its black
 *    outline; drawing then only composites the cached cell
 *  - dirty tracking: end_frame() flushes CPU cache (DMA_BUF_IOCTL_SYNC END)
 *    only when something was drawn; the buffer is written by CPU only, so no
 *    invalidate is ever needed
 *  - all drawing is plain integer code, no floating point in hot paths
 *
 * Element vocabulary follows Betaflight osd.h where applicable:
 * crosshairs / home dir / warnings / flymode / link quality / battery usage /
 * rtc datetime / timer / telemetry numbers, plus AI interception additions:
 * detect boxes with label+confidence+track-id, lock indicator, target count.
 */
#pragma once

#include <cstdint>
#include <cstddef>
#include "osd_font.h"    /* FONT_W/FONT_H/FONT_ATLAS (16x32 AA mono font) */

namespace osd {

/* ------------------------------------------------------------------ */
/* basic types                                                         */
/* ------------------------------------------------------------------ */
struct Color { uint8_t r, g, b, a; };

constexpr inline Color RGB(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255)
{ return Color{r, g, b, a}; }

/* palette used by the demo / suggested defaults */
constexpr Color kWhite {235, 235, 235, 255};
constexpr Color kBlack {0, 0, 0, 255};
constexpr Color kAmber {255, 200, 60, 255};
constexpr Color kRed   {255, 70, 70, 255};
constexpr Color kGreen {90, 230, 90, 255};
constexpr Color kCyan  {80, 220, 255, 255};
constexpr Color kBlue  {90, 150, 255, 255};

struct Rect { int x = 0, y = 0, w = 0, h = 0; };

/* font geometry: normal 16x32 AA mono, small 8x16 AA mono */
constexpr int glyph_w() { return FONT_W; }
constexpr int glyph_h() { return FONT_H; }
constexpr int glyph_w_small() { return FONT_W_SMALL; }
constexpr int glyph_h_small() { return FONT_H_SMALL; }

struct TextStyle {
    Color color = kWhite;
    /* 2px black outline, baked into the glyph cache. The antialiased core is
     * resolved against it so an outlined glyph holds only alpha 0 or 255 -
     * required on a canvas fed to RGA imblend, which miscomposites src
     * alpha<255 into a white fringe. Unoutlined text keeps true AA and is
     * only safe on a DRM plane (exact VOP blending) or over an opaque fill. */
    bool outline = true;
    bool small = false;        /* 8x16 atlas: secondary info */
    int  tracking = 0;         /* extra pixels between characters */
};

/* ------------------------------------------------------------------ */
/* AI detection box                                                    */
/* ------------------------------------------------------------------ */
enum class BoxKind {
    CornerBrackets,            /* FPV-style corner brackets, low clutter */
    Full,                      /* full rectangle outline */
};

/* BBoxRect - the dynamic detection-box input type. Color may change every
 * frame at no cost: brackets/plate are rasterised per call and the label
 * text is always fixed opaque white, so box colors never touch the glyph
 * cache. This is the type the layout engine and the ROS bridge inject. */
struct BBoxRect {
    Rect rect;                 /* pixel coords on the OSD canvas */
    const char *label = "";    /* class name, e.g. "UAV" */
    const char *tag = "";      /* short tag, e.g. "T01" or "LCK" */
    float conf = 0.f;          /* 0..1, drawn as percent */
    Color color = kRed;
    BoxKind kind = BoxKind::CornerBrackets;
    bool locked = false;       /* pulsing center diamond + heavier corners */
    /* plate translucency source: true = opaque plate, the DISPLAY plane's
     * global alpha makes it translucent (recommended, exact math);
     * false = CPU per-pixel blend into the canvas (single-layer use) */
    bool plate_opaque = true;

    /* per-pixel alpha for plate/brackets/echo/diamond (label text stays
     * opaque). On an overlay plane this multiplies with the plane's global
     * alpha, giving translucency control without any per-frame ioctl. */
    uint8_t alpha = 255;
};

/* ------------------------------------------------------------------ */
/* layer                                                               */
/* ------------------------------------------------------------------ */
class Layer {
public:
    Layer() = default;
    ~Layer();

    Layer(const Layer &) = delete;
    Layer &operator=(const Layer &) = delete;

    /* allocate RGBA8888 canvas on a dma-heap (cached heap is fine:
     * end_frame() performs the explicit cache clean). Strict: the board
     * pipeline REQUIRES a dmabuf (RGA imports it) - never fall back. */
    bool init(int w, int h,
              const char *heap_path = "/dev/dma_heap/system");
    /* plain heap-memory backing (no dmabuf, dma_fd() == -1). For host-side
     * tests/tools on machines without /dev/dma_heap - anything fed to RGA
     * must NOT use this. */
    bool init_mem(int w, int h);
    /* dma-heap first, plain memory when unavailable (tests, probes). */
    bool init_auto(int w, int h);
    /* wrap an externally-owned buffer (e.g. a DRM dumb buffer mapped by the
     * caller) instead of allocating; shutdown() will not free it */
    bool attach(int dma_fd, void *va, int w, int h, size_t size);
    void shutdown();
    bool valid() const { return m_va != nullptr; }

    int width()  const { return m_w; }
    int height() const { return m_h; }
    int dma_fd() const { return m_fd; }
    void *pixels() const { return m_va; }
    size_t size() const { return m_size; }

    /* ---------------------------------------------------------------- */
    /* primitives                                                       */
    /* ---------------------------------------------------------------- */
    void clear(Rect r);                                /* alpha=0 */
    void clear_all();
    void fill_rect(Rect r, Color c);
    /* per-pixel SRC-OVER fill (straight alpha) - used for translucent
     * elements like detection-box plates so whatever is already on the
     * canvas (HUD text, reticle) shows through correctly. RGA2-E's
     * hardware blend is unreliable for src alpha < 255, so translucency
     * inside the canvas is composited on the CPU instead. */
    void fill_rect_blend(Rect r, Color c);
    void draw_rect(Rect r, Color c, int t = 1);
    void draw_line(int x0, int y0, int x1, int y1, Color c, int t = 1);
    void draw_text(int x, int y, const char *s, const TextStyle &st);
    int  text_width(const char *s, const TextStyle &st) const;

    /* ---------------------------------------------------------------- */
    /* composite elements (Betaflight-inspired + AI interception)      */
    /* ---------------------------------------------------------------- */
    /* center reticle: cross with gap + circle + dot. `r` is the circle
     * radius; arms/cross scale from it (default = compact FPV reticle). */
    void draw_crosshair(int cx, int cy, Color c, int r = 22);

    /* pitch-ladder artificial horizon centred on (cx, cy): rungs every
     * 5 deg (10 deg rungs longer), translated by pitch (positive = nose
     * up -> ladder slides down) and rotated by roll (positive = right
     * roll -> ladder rotates clockwise... reads best combined with the
     * center reticle drawn over it). Caller restores the region each
     * update - use horizon_region() for the restore footprint. */
    void draw_horizon(int cx, int cy, float pitch_deg, float roll_deg,
                      Color c, int px_per_deg = 5, int half_span = 80);

    /* restore footprint covering every line draw_horizon may paint at
     * any pitch/roll within +/-20 deg */
    static constexpr Rect horizon_region(int cx, int cy)
    { return Rect{cx - 128, cy - 100, 256, 200}; }

    /* home-direction arrow, angle 0 = up, clockwise degrees */
    void draw_home_arrow(int cx, int cy, float angle_deg, Color c, int size = 22);

    /* horizontal meter (link quality / battery usage) */
    void draw_bar(int x, int y, int w, int h, float frac,
                  Color fg, Color bg);

    /* flashing warning text (caller passes blink phase) */
    void draw_warning(int x, int y, const char *s, bool blink_on);

    /* AI detection box (BBoxRect): corners, label bar w/ class+conf+tag,
     * lock pulse. Returns the exact painted bounds (everything touched:
     * brackets, echo frame, label plate incl. border) - use it as the clear
     * footprint for the next frame so moving boxes never leave trails. */
    Rect draw_bbox(const BBoxRect &b);

    /* ---------------------------------------------------------------- */
    /* frame lifecycle & RGA integration                               */
    /* ---------------------------------------------------------------- */
    /* static base: snapshot the canvas after drawing the static layer;
     * moving elements then restore() their old footprint from the base
     * instead of punching permanent holes through static elements. */
    void set_base();
    void restore(Rect r);     /* copy base region back into the canvas */
    bool has_base() const { return m_base != nullptr; }

    void begin_frame();       /* advances blink/frame counter */
    void end_frame();         /* DMA_BUF_IOCTL_SYNC(END) if dirty */
    bool dirty() const { return m_dirty; }

    uint32_t frame_count() const { return m_frames; }
    /* blink phase ~3.3Hz at 60fps */
    bool blink_phase() const { return (m_frames / 18u) & 1u; }

    /* glyph cache occupancy (chars x colors rendered) */
    size_t glyph_cache_count() const { return m_cache_count; }

private:
    bool init_i();

    struct Glyph {
        uint32_t px[FONT_H + 4][FONT_W + 4];   /* glyph + 2px outline */
        uint32_t binary;                       /* alpha only 0 or 255 */
        int cw = 0, ch = 0;                    /* cell dims (small font smaller) */
    };
    /* one glyph table per (color, outlined, small) in use, built lazily;
     * apps use a handful of styles so a linear scan is cheaper than a map */
    struct GlyphSet {
        Color color;
        bool  has[95] = {};
        Glyph *glyph[95] = {};
    };
    const Glyph *glyph(char c, const TextStyle &st) const;

    int    m_fd = -1;
    void  *m_va = nullptr;
    size_t m_size = 0;
    int    m_w = 0, m_h = 0;
    uint32_t *m_px = nullptr;

    mutable GlyphSet m_sets[16];               /* up to 16 colors */
    mutable size_t   m_sets_used = 0;
    mutable size_t   m_cache_count = 0;

    uint32_t m_frames = 0;
    bool     m_dirty = false;
    bool     m_attached = false;   /* buffer owned by caller */
    bool     m_heap = false;       /* plain new[] backing (no dmabuf) */
    bool     m_wc = false;         /* write-combine (attached DRM dumb) */
    uint32_t *m_base = nullptr;
};

} // namespace osd
