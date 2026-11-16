/* x3f_alcedo_foveon.c — path B implementation, a faithful port of X3Lumina's
 * X3FExtractor display chain (RawRender.swift loadWB/toC/renderCGImage).
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * All legacy switches are process-global in x3f_render.c. A mutex serialises
 * the setter->load / setter->apply windows so concurrent previews cannot swap
 * each other's mode (X3Lumina does the same with a single linQueue).
 */
#include "x3f_alcedo_foveon.h"

#include "x3f_render.h"
#include "x3fcore.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#  include <windows.h>
#else
#  include <pthread.h>
#endif

/* ------------------------------------------------------------------------- */
static char g_error[256] = {0};

#ifdef _WIN32
static CRITICAL_SECTION g_lock;
static INIT_ONCE g_once = INIT_ONCE_STATIC_INIT;
static BOOL CALLBACK init_lock(PINIT_ONCE o, PVOID p, PVOID *c) {
    (void)o; (void)p; (void)c; InitializeCriticalSection(&g_lock); return TRUE;
}
static void lock_legacy(void) {
    InitOnceExecuteOnce(&g_once, init_lock, NULL, NULL);
    EnterCriticalSection(&g_lock);
}
static void unlock_legacy(void) { LeaveCriticalSection(&g_lock); }
#else
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static void lock_legacy(void)   { pthread_mutex_lock(&g_lock); }
static void unlock_legacy(void) { pthread_mutex_unlock(&g_lock); }
#endif

static void set_error(const char *msg) {
    snprintf(g_error, sizeof(g_error), "%s", msg);
}
const char *x3f_alcedo_foveon_error(void) { return g_error; }

/* ------------------------------------------------------------------------- */
void x3f_alcedo_edit_default(x3f_alcedo_edit_t *e) {
    if (!e) return;
    memset(e, 0, sizeof(*e));

    /* Tone / detail: X3Lumina Model.swift defaults. */
    e->detail         = 0.5;
    e->sharp_res      = 1.0;
    e->tc[4]          = 64.0;
    e->tc[5]          = 128.0;
    e->tc[6]          = 192.0;
    e->wb_red = e->wb_green = e->wb_blue = 1.0;

    /* Highlight panel. */
    e->hl_recover     = 0.0;
    e->hl_keep        = 0.5;
    e->sdq_hl         = 0.0;
    e->hl_chroma      = 1.0;
    e->hl_chroma_hue_gate = 1;

    /* User decision (2026-11-15): these three ship OFF in X3Lumina but must be
     * ON by default in the Studio fork. They all resolve into one decode mode;
     * see x3f_alcedo_foveon_resolve_oe() for the per-generation gate. */
    e->oe              = X3F_ALCEDO_OE_MODE8;
    e->hole_fix        = 1;
    e->merrill_hl_neut = 1;

    /* Fill light calibration knobs (RawRender.swift toC). */
    e->fl_tune   = 1;
    e->fl_hlzred = 0.95;
    e->fl_mapgain = 0.75;
    e->fl_zgain  = 1.0;

    /* Quattro NR baseline: off by default, values retained for opt-in. */
    e->nr_q_base    = 0.8;
    e->nr_q_gate    = 1;
    e->nr_q_gate_k  = 0.75;

    /* Monochrome / split / glow defaults. */
    e->mono_mix_r = e->mono_mix_g = e->mono_mix_b = 100.0;
    e->mono_tone_r = e->mono_tone_g = e->mono_tone_b = 100.0;
    e->mono_grain_size      = 0.5;
    e->mono_grain_roughness = 0.5;
    e->split_blending = 50.0;
    e->glow_radius    = 0.5;
    e->glow_threshold = 0.7;

    memcpy(e->color_mode, "AsShot", 7);
    e->color_strength = 50.0;

    e->out_space = X3F_OUT_SRGB;
    e->down      = 1;
}

/* Model.swift:664-670 + RawRender.swift:571-586.
 *   Quattro              -> oe ? 8 : 4, hole-fix allowed
 *   Merrill / classic    -> sky neutralisation 3 wins; hole-fix forced off,
 *                           overexposure selector greyed to "off" (mode 4)
 * effectiveRawEdit() clears merrillHlNeut on Quattro and clears the
 * overexposure switches on non-Quattro; both are reproduced here.
 *
 * `model_class` here is the GATE generation (x3f_alcedo_model_gen), not the
 * narrow array probe: SD Quattro / SD Quattro H are Quattro for every
 * overexposure decision in X3Lumina even though their type_format differs. */
int x3f_alcedo_foveon_resolve_oe(const x3f_alcedo_edit_t *e, int model_class) {
    if (!e) return 4;
    if (model_class == X3F_ALCEDO_QUATTRO) {
        return e->oe == X3F_ALCEDO_OE_MODE8 ? 8 : 4;
    }
    return e->merrill_hl_neut ? 3 : 4;
}

/* FNV-1a over the decode-level fields only. Look sliders are deliberately
 * absent: changing them re-runs apply() on the cached linear frame. */
uint64_t x3f_alcedo_foveon_decode_key(const x3f_alcedo_edit_t *e) {
    if (!e) return 0;
    uint64_t h = 1469598103934665603ULL;
#define MIX_U64(v) do { uint64_t _v = (uint64_t)(v); \
    for (int _i = 0; _i < 8; ++_i) { h ^= (_v >> (_i * 8)) & 0xff; h *= 1099511628211ULL; } \
} while (0)
#define MIX_D(v) do { uint64_t _b; memcpy(&_b, &(v), 8); MIX_U64(_b); } while (0)
    MIX_U64(e->oe);
    MIX_U64(e->hole_fix);
    MIX_U64(e->merrill_hl_neut);
    MIX_U64(e->hlp_fix);
    MIX_U64(e->nlm);
    MIX_U64(e->frame);
    MIX_U64(e->down);
    /* wb_temp / wb_tint / sfd_tone are APPLY-level fields in X3Lumina
     * (x3f_edit_params_t): they only change the display chain and must NOT
     * invalidate the decoded linear frame. Only the WB *preset name* is
     * decode-level: it selects a different per-illuminant colour matrix. */
    for (size_t i = 0; i < sizeof(e->wb_preset); ++i) MIX_U64((unsigned char)e->wb_preset[i]);
#undef MIX_D
#undef MIX_U64
    return h;
}

/* ------------------------------------------------------------------------- */
struct x3f_alcedo_foveon_frame {
    float  *lin;
    float  *clip;
    int     w, h;
    int     orientation;
    int     model_class;
    char    path[1024];       /* original source path (may be an X3I container) */
    char    detected_color_mode[64];  /* header colour mode, empty on Quattro */
    uint64_t decode_key;
};

void x3f_alcedo_foveon_geometry(const x3f_alcedo_foveon_frame_t *f,
                                int *w, int *h, int *orientation) {
    if (!f) return;
    if (w) *w = f->w;
    if (h) *h = f->h;
    if (orientation) *orientation = f->orientation;
}

void x3f_alcedo_foveon_free(x3f_alcedo_foveon_frame_t *f) {
    if (!f) return;
    x3f_render_free(f->lin);
    x3f_render_free(f->clip);
    free(f);
}

int x3f_alcedo_foveon_prepare(const char *path, const x3f_alcedo_edit_t *edit,
                              x3f_alcedo_foveon_frame_t **out) {
    if (!out) { set_error("foveon_prepare: null out"); return X3F_ALCEDO_ERR_ARG; }
    *out = NULL;
    if (!path || !*path || !edit) {
        set_error("foveon_prepare: null argument");
        return X3F_ALCEDO_ERR_ARG;
    }

    char tmp[1024] = {0};
    const char *use = x3f_standalone_path(path, edit->frame, tmp, sizeof(tmp));
    if (!use || !*use) {
        set_error("foveon_prepare: cannot materialize X3I frame");
        return X3F_ALCEDO_ERR_CONTAINER;
    }

    const int model = x3f_alcedo_model_class(use);
    /* Gate decisions follow the Quattro *generation* (DP-Q + SDQ + SDQH + SFD);
     * the narrow array probe would grey out SDQ/SDQH's oe and sdq_hl controls. */
    const int gen   = x3f_alcedo_model_gen(use);
    const int oe    = x3f_alcedo_foveon_resolve_oe(edit, gen);
    const int quattro = (gen == X3F_ALCEDO_QUATTRO);

    x3f_denoise_t dn;
    x3f_denoise_default(&dn);

    float *lin = NULL, *clip = NULL;
    int    w = 0, h = 0;

    lock_legacy();
    /* Decode-level setters, in X3Lumina's loadWB() order. */
    x3f_render_set_hlp_fix(edit->hlp_fix ? 1 : 0);
    x3f_hole_fix_set_enabled((edit->hole_fix && quattro) ? 1 : 0);
    x3f_render_set_oe_neutralize(oe);
    x3f_render_set_oe_hlp(0);
    /* "SDQ 高光" normalisation strength. This was previously declared in the
     * edit struct but never forwarded, so the slider was inert on every body.
     * It is APPLY-level in the C chain (sdq_dig is read at load time and stays
     * fixed), hence it is not part of the decode key either. Non-SDQ files
     * multiply by 1.0 exactly, so forwarding it is a no-op there. */
    x3f_render_set_sdq_hl(edit->sdq_hl);
    const char *wb = edit->wb_preset[0] ? edit->wb_preset : NULL;
    const int   rc = x3f_render_load_wb(use, edit->down > 0 ? edit->down : 1,
                                        wb, edit->nlm, &dn,
                                        &lin, &clip, NULL, NULL, NULL,
                                        &w, &h, NULL);
    unlock_legacy();

    if (rc != X3FRENDER_OK || !lin || w <= 0 || h <= 0) {
        x3f_render_free(lin);
        x3f_render_free(clip);
        x3f_temp_release(tmp);
        set_error("foveon_prepare: x3f_render_load_wb failed");
        return X3F_ALCEDO_ERR_DECODE;
    }

    /* Colour mode is resolved from the standalone frame path; AsShot reads the
     * header's recorded mode and falls back to Standard. */
    char detected[64] = {0};
    x3f_render_probe_header_color_mode(use, detected, (int)sizeof(detected));
    int orientation = x3f_alcedo_header_orientation(use);
    x3f_temp_release(tmp);

    x3f_alcedo_foveon_frame_t *f = (x3f_alcedo_foveon_frame_t *)calloc(1, sizeof(*f));
    if (!f) {
        x3f_render_free(lin);
        x3f_render_free(clip);
        set_error("foveon_prepare: out of memory");
        return X3F_ALCEDO_ERR_NOMEM;
    }
    f->lin          = lin;
    f->clip         = clip;
    f->w            = w;
    f->h            = h;
    f->orientation  = orientation;
    f->model_class  = model;
    f->decode_key   = x3f_alcedo_foveon_decode_key(edit);
    snprintf(f->path, sizeof(f->path), "%s", path);
    snprintf(f->detected_color_mode, sizeof(f->detected_color_mode), "%s", detected);
    *out = f;
    return X3F_ALCEDO_OK;
}

/* ------------------------------------------------------------------------- */
/* Build x3f_edit_params_t exactly like RawRender.swift toC(). */
static void build_params(const x3f_alcedo_foveon_frame_t *f,
                         const x3f_alcedo_edit_t *e,
                         x3f_edit_params_t *p) {
    x3f_edit_defaults(p);

    p->exposure   = e->exposure;
    p->contrast   = e->contrast;
    p->shadow     = e->shadow;
    p->highlight  = e->highlight;
    p->saturation = e->saturation;
    p->detail     = e->detail;
    p->sharpness  = e->sharpness;
    p->sharp_res  = (e->sharp_res > 0.0 && e->sharp_res < 1.0) ? e->sharp_res : 1.0;
    p->wb_temp    = e->wb_temp;
    p->wb_tint    = e->wb_tint;
    p->hl_recover = e->hl_recover;
    p->fill_light = e->fill_light;
    p->hl_keep    = e->hl_keep;
    p->sfd_tone   = e->sfd_tone;

    /* hl_soften: X3Lumina ties it to the overexposure switch. */
    p->hl_soften = (e->oe == X3F_ALCEDO_OE_MODE8) ? 0 : 1;

    p->fl_tune    = e->fl_tune;
    p->fl_hlzred  = e->fl_hlzred;
    p->fl_lmean_r = e->fl_lmean_r;
    p->fl_msat    = e->fl_msat;
    p->fl_mapgain = e->fl_mapgain;
    p->fl_k0off   = e->fl_k0off;
    p->fl_zgain   = e->fl_zgain;

    p->nr_luma    = e->nr_luma;
    p->nr_chroma  = e->nr_chroma;
    p->nr_banding = e->nr_banding;
    p->nr_q_baseline = e->nr_q_baseline;
    p->nr_q_base     = e->nr_q_base;
    p->nr_q_gate     = e->nr_q_gate;
    p->nr_q_gate_k   = e->nr_q_gate_k;

    memcpy(p->tc, e->tc, sizeof(p->tc));

    p->mono                 = e->mono;
    p->mono_mix_r           = e->mono_mix_r;
    p->mono_mix_g           = e->mono_mix_g;
    p->mono_mix_b           = e->mono_mix_b;
    p->mono_tone_r          = e->mono_tone_r;
    p->mono_tone_g          = e->mono_tone_g;
    p->mono_tone_b          = e->mono_tone_b;
    p->mono_grain           = e->mono_grain;
    p->mono_grain_size      = e->mono_grain_size;
    p->mono_grain_roughness = e->mono_grain_roughness;

    p->split_sh_hue  = e->split_sh_hue;
    p->split_sh_sat  = e->split_sh_sat / 100.0;
    p->split_hl_hue  = e->split_hl_hue;
    p->split_hl_sat  = e->split_hl_sat / 100.0;
    p->split_balance = e->split_balance / 100.0;
    p->split_blending= e->split_blending / 100.0;

    p->glow           = e->glow;
    p->glow_radius    = e->glow_radius;
    p->glow_threshold = e->glow_threshold;
    p->wb_red         = e->wb_red;
    p->wb_green       = e->wb_green;
    p->wb_blue        = e->wb_blue;

    p->hl_chroma         = e->hl_chroma;
    p->hl_chroma_win_lo  = 0.78;
    p->hl_chroma_win_hi  = 0.93;
    if (e->hl_chroma_hue_gate) {
        p->hl_chroma_hue_lo = 30.0;
        p->hl_chroma_hue_hi = 75.0;
    } else {
        p->hl_chroma_hue_lo = 0.0;
        p->hl_chroma_hue_hi = 360.0;
    }

    p->out_space      = e->out_space;
    p->look_defer     = e->look_defer;
    p->align_look_off = e->align_look_off;

    /* Colour mode: resolve on the original path. AsShot uses the header mode
     * detected in prepare(); Quattro has none in the header (it lives in the
     * embedded JPEG MakerNote) so Standard is the same no-op fallback X3Lumina
     * applies when detection fails. */
    const char *mode = e->color_mode[0] ? e->color_mode : "AsShot";
    if (strcmp(mode, "AsShot") == 0) {
        /* Host-provided detection (embedded JPEG MakerNote, covers Quattro)
         * wins; the header probe only sees Merrill/classic. */
        mode = e->detected_color_mode[0] ? e->detected_color_mode
             : (f->detected_color_mode[0] ? f->detected_color_mode : "Standard");
    }
    double strength = e->color_strength;
    if (strength < 0.0) strength = 0.0;
    if (strength > 100.0) strength = 100.0;
    x3f_color_mode_default(&p->color);
    (void)x3f_color_mode_resolve(f->path, mode, strength, &p->color);
}

void x3f_alcedo_foveon_image_free(x3f_alcedo_foveon_image_t *img) {
    if (!img) return;
    free(img->rgb);
    free(img->linear_rgb);
    memset(img, 0, sizeof(*img));
}

int x3f_alcedo_foveon_apply(x3f_alcedo_foveon_frame_t *f,
                            const x3f_alcedo_edit_t *edit,
                            x3f_alcedo_foveon_image_t *out) {
    if (!f || !edit || !out) {
        set_error("foveon_apply: null argument");
        return X3F_ALCEDO_ERR_ARG;
    }
    memset(out, 0, sizeof(*out));
    if (f->decode_key != x3f_alcedo_foveon_decode_key(edit)) {
        set_error("foveon_apply: decode-level parameter changed, call prepare() again");
        return X3F_ALCEDO_ERR_ARG;
    }

    x3f_edit_params_t p;
    build_params(f, edit, &p);

    const size_t n = (size_t)f->w * (size_t)f->h * 3u;
    uint16_t *rgb16 = (uint16_t *)malloc(n * sizeof(uint16_t));
    float    *lin   = NULL;
    if (edit->look_defer) lin = (float *)malloc(n * sizeof(float));
    if (!rgb16 || (edit->look_defer && !lin)) {
        free(rgb16);
        free(lin);
        set_error("foveon_apply: out of memory");
        return X3F_ALCEDO_ERR_NOMEM;
    }

    int rc;
    lock_legacy();
    /* Apply-level setters, refreshed on every apply (RawRender.swift toC). */
    x3f_render_set_oe_tone_off(edit->hlp_fix ? 0 : 1);
    x3f_render_set_hl9_odm8(edit->oe_m8od ? 1 : 0);
    x3f_render_set_sdq_hl(edit->sdq_hl);
    rc = x3f_render_apply_ex16(&p, f->lin, f->clip, f->w, f->h,
                               rgb16, f->w * 3 * (int)sizeof(uint16_t));
    if (rc == X3FRENDER_OK && lin) {
        rc = x3f_render_apply_ex_linear(&p, f->lin, f->clip, f->w, f->h, lin);
    }
    unlock_legacy();

    if (rc != X3FRENDER_OK) {
        free(rgb16);
        free(lin);
        set_error("foveon_apply: x3f_render_apply_ex16 failed");
        return X3F_ALCEDO_ERR_DECODE;
    }
    out->rgb         = rgb16;
    out->linear_rgb  = lin;
    out->width       = f->w;
    out->height      = f->h;
    out->orientation = f->orientation;
    return X3F_ALCEDO_OK;
}

int x3f_alcedo_foveon_render(const char *path, const x3f_alcedo_edit_t *edit,
                             x3f_alcedo_foveon_image_t *out) {
    x3f_alcedo_foveon_frame_t *f = NULL;
    const int rc = x3f_alcedo_foveon_prepare(path, edit, &f);
    if (rc != X3F_ALCEDO_OK) return rc;
    const int arc = x3f_alcedo_foveon_apply(f, edit, out);
    x3f_alcedo_foveon_free(f);
    return arc;
}
