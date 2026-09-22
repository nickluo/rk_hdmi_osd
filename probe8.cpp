/* probe8: libosd unit probe - fill_rect / text compositing / detect-box plate.
 *
 * Regression targets:
 *  A) glyph packing: alpha must land in bits[31:24], not in the R byte
 *     (the old `(core & ~0xFF) | a` produced cyan fringes on white text)
 *  B) draw_text must COMPOSITE, not memcpy the glyph cell: a raw blit wrote
 *     the cell's transparent pixels too and erased the label plate underneath
 *     (observed: only the 6px left / 2px right plate margins survived)
 */
#include <cstdio>
#include <cstring>
#include "osd/osd.h"

static int g_fail = 0;

static void check(bool ok, const char *what, const char *detail)
{
    printf("%-4s %-34s %s\n", ok ? "PASS" : "FAIL", what, detail);
    if (!ok) g_fail++;
}

struct Px { uint32_t r, g, b, a; };

static Px px(const osd::Layer &l, int x, int y)
{
    uint32_t v = ((const uint32_t *)l.pixels())[(size_t)y * l.width() + x];
    return {v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, v >> 24};
}

int main()
{
    osd::Layer l;
    if (!l.init_auto(720, 240)) { printf("init fail\n"); return 1; }
    char d[160];

    /* ---- A. fill_rect stores the exact packed color ---- */
    l.fill_rect({10, 10, 100, 50}, osd::Color{127, 35, 35, 160});
    Px p = px(l, 60, 35);
    snprintf(d, sizeof d, "got (%u,%u,%u,%u) want (127,35,35,160)", p.r, p.g, p.b, p.a);
    check(p.r == 127 && p.g == 35 && p.b == 35 && p.a == 160, "fill_rect exact", d);

    /* ---- B. glyph packing: core opaque, AA edge keeps the core hue ---- */
    osd::TextStyle st;
    st.color = osd::kWhite;
    st.outline = false;
    l.draw_text(10, 120, "Ag", st);

    bool core_ok = false, edge_seen = false, edge_ok = true;
    uint32_t bad_r = 0, bad_g = 0, bad_b = 0, bad_a = 0;
    for (int y = 120; y < 120 + osd::glyph_h() + 4; y++) {
        for (int x = 10; x < 10 + 2 * osd::glyph_w() + 4; x++) {
            Px q = px(l, x, y);
            if (q.a == 0) continue;
            if (q.a == 255 && q.r == 235 && q.g == 235 && q.b == 235) core_ok = true;
            if (q.a < 255) {
                edge_seen = true;
                /* white text: an AA pixel must stay neutral. The old bug put
                 * the coverage value in R and left A=255 -> cyan fringe. */
                if (!(q.r == q.g && q.g == q.b)) {
                    if (edge_ok) { bad_r = q.r; bad_g = q.g; bad_b = q.b; bad_a = q.a; }
                    edge_ok = false;
                }
            }
        }
    }
    check(core_ok, "glyph core opaque core color", "a=255 rgb=(235,235,235)");
    snprintf(d, sizeof d, edge_ok ? "neutral, aa pixels seen=%d" : "tinted (%u,%u,%u,%u)",
             edge_ok ? (int)edge_seen : (int)bad_r, bad_g, bad_b, bad_a);
    check(edge_seen && edge_ok, "glyph AA edge not tinted", d);

    /* ---- C. text over a plate must not erase the plate ---- */
    const osd::Rect plate{300, 10, 320, 44};
    l.fill_rect(plate, osd::Color{60, 20, 20, 160});
    st.color = osd::Color{255, 255, 255, 255};
    l.draw_text(plate.x + 6, plate.y + 4, "UAV 86% T01", st);

    int kept = 0, holes = 0;
    for (int y = plate.y; y < plate.y + plate.h; y++)
        for (int x = plate.x; x < plate.x + plate.w; x++) {
            Px q = px(l, x, y);
            if (q.a == 0) holes++;
            else if (q.a == 160 && q.r == 60 && q.g == 20 && q.b == 20) kept++;
        }
    const int area = plate.w * plate.h;
    snprintf(d, sizeof d, "kept %d/%d (%.1f%%), transparent holes %d",
             kept, area, 100.0 * kept / area, holes);
    check(holes == 0 && kept > area / 2, "text composites over plate", d);

    /* ---- D. draw_bbox: plate fill present under the label ---- */
    osd::Layer b;
    if (!b.init_auto(720, 240)) { printf("init2 fail\n"); return 1; }
    osd::BBoxRect db;
    db.rect = {150, 120, 100, 70};
    db.label = "UAV";
    db.tag = "T01";
    db.conf = 0.86f;
    db.color = osd::kRed;
    db.alpha = 160;
    db.plate_opaque = true;
    osd::Rect painted = b.draw_bbox(db);

    /* plate sits above the box: {r.x, r.y - FONT_H - 8, tw + 12, FONT_H + 6} */
    const int py = db.rect.y - osd::glyph_h() - 8;
    int plate_px = 0;
    for (int y = py; y < py + osd::glyph_h() + 6; y++)
        for (int x = db.rect.x; x < db.rect.x + 180; x++)
            if (px(b, x, y).a == 160) plate_px++;
    snprintf(d, sizeof d, "alpha-160 plate pixels under label: %d (want >2000)", plate_px);
    check(plate_px > 2000, "detect box plate survives label", d);

    Px br = px(b, db.rect.x + 1, db.rect.y + 1);
    snprintf(d, sizeof d, "got (%u,%u,%u,%u) want (255,70,70,160)", br.r, br.g, br.b, br.a);
    check(br.r == 255 && br.g == 70 && br.b == 70 && br.a == 160, "bracket color/alpha", d);

    snprintf(d, sizeof d, "%d,%d %dx%d (plate y=%d)", painted.x, painted.y,
             painted.w, painted.h, py);
    check(painted.y <= py && painted.y + painted.h >= db.rect.y + db.rect.h &&
          painted.w >= 180, "painted bounds cover plate+box", d);

    printf("\n%s (%d failures)\n", g_fail ? "PROBE8 FAILED" : "PROBE8 OK", g_fail);
    return g_fail ? 1 : 0;
}
