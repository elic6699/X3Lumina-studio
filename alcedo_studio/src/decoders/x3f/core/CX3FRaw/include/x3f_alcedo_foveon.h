/* x3f_alcedo_foveon.h — path B: X3Lumina Foveon native-chain preview/export.
 *
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * This is a faithful port of X3Lumina's X3FExtractor display chain
 * (Model.swift RawEditParams -> RawRender.swift toC -> x3f_render_load_wb +
 * x3f_render_apply_ex16). It reproduces the full adjustment panel, including
 * mode8 (SPP Stage2 straight replay), hole-fix and overexposure correction,
 * so switching any of them visibly changes the preview.
 *
 * IMPORTANT — colour contract. Path B is DISPLAY-REFERRED: it runs the legacy
 * f64 tone/gamma chain and produces a 16-bit sRGB *finished image*. Negative
 * channels and >1 scene-linear headroom do not survive; it is not wide-gamut.
 * It is therefore never fed into path A (x3f_alcedo_export.h), the
 * DirectRgb Develop DAG, or any wide-gamut export. The user opts into path B
 * explicitly ("Foveon native chain"); path A stays the default.
 *
 * Preview is two-stage, exactly like X3Lumina's Engine.swift:
 *   prepare()  decodes one DOWNSAMPLED linear frame and caches lin/clip;
 *   apply()    re-runs only the display chain (all look sliders land here).
 * Decode-level switches (overexposure mode / hole-fix / HLP / WB preset / NLM)
 * invalidate the prepared frame; x3f_alcedo_foveon_decode_key() tells the
 * caller when a new prepare() is required.
 *
 * All legacy switches are set with explicit C setters, never setenv(): the
 * process-global environment is unsafe with concurrent previews.
 */
#ifndef X3F_ALCEDO_FOVEON_H
#define X3F_ALCEDO_FOVEON_H

#include <stddef.h>
#include <stdint.h>

#include "x3f_alcedo_export.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Overexposure correction selector, matching X3Lumina's Picker (2 states).
 * The legacy C chain has more modes; only these are exposed in the product. */
enum {
    X3F_ALCEDO_OE_OFF  = 0,  /* UI "关闭" -> decode mode 4 (highlight neutralise) */
    X3F_ALCEDO_OE_MODE8 = 1  /* UI "mode8" -> decode mode 8 (SPP Stage2 replay) */
};

/* Full adjustment state, mirroring X3Lumina's RawEditParams. Field names and
 * ranges follow the Swift model / x3f_edit_params_t 1:1 so the panel can be
 * bound mechanically. Use x3f_alcedo_edit_default() — do NOT zero-init. */
typedef struct {
    /* Tone (Model.swift:323-327) */
    double exposure;        /* -2..+2 EV */
    double contrast;        /* -1..+1   */
    double shadow;          /* -1..+1, positive lifts shadows */
    double highlight;       /* -1..+1, positive darkens highlights */
    double saturation;      /* -1..+1   */

    /* Detail / sharpening (:346, :398) */
    double detail;          /* 0..1, 7 steps, default 0.5 */
    double sharpness;       /* 0..2, 0 = mode default */
    double sharp_res;       /* 0<v<1 scales sharpening for a preview frame */

    /* Tone curve four bands + three range breakpoints (:388-394) */
    double tc[7];           /* tcSh, tcDk, tcLi, tcHi, tcR2(64), tcR3(128), tcR4(192) */

    /* White balance (:587-588, :622-624, :612) */
    double wb_temp;         /* -1..+1 */
    double wb_tint;         /* -1..+1 */
    double wb_red, wb_green, wb_blue;  /* 1.0 = neutral, 0..2 */
    char   wb_preset[32];   /* empty = as-shot; Sunlight/Shade/.../Auto */

    /* ---- Highlight panel (Views.swift:2739-3575) ---- */
    double hl_recover;      /* 高光恢复 0..1, default 0 */
    double hl_keep;         /* 高光保留 0..1, default 0.5 */
    int    oe;              /* 过曝矫正: X3F_ALCEDO_OE_OFF / _MODE8 */
    int    hole_fix;        /* 高光去粉 (hole-fix), Quattro only */
    int    merrill_hl_neut; /* 过曝天空中性化, Merrill + classic only */
    int    hlp_fix;         /* HLP grid-point fix (hidden, bound to oe tone) */
    double sdq_hl;          /* SDQ 高光 0..2, SDQ / SDQH only */
    double hl_chroma;       /* 亮部保色 1.0..2.4, 1.0 = off */
    int    hl_chroma_hue_gate;  /* 只作用于黄/橙, default on */
    int    oe_m8od;         /* hidden m8od (mode8 + od delta) */

    /* Fill light + its 6 calibration knobs (:593-604) */
    double fill_light;      /* -2..+2 step 0.1, SPP same range */
    int    fl_tune;         /* 1 = use the knobs below */
    double fl_hlzred, fl_lmean_r, fl_msat, fl_mapgain, fl_k0off, fl_zgain;

    /* SPP three noise sliders (:781-790) */
    double nr_luma, nr_chroma, nr_banding;   /* 0..1, quantised to 5 steps */
    int    nr_q_baseline;
    double nr_q_base;
    int    nr_q_gate;
    double nr_q_gate_k;

    /* Monochrome page (:840-873) */
    int    mono;
    double mono_mix_r, mono_mix_g, mono_mix_b;
    double mono_tone_r, mono_tone_g, mono_tone_b;
    int    mono_grain;
    double mono_grain_size, mono_grain_roughness;

    /* Split toning (:989-999) / glow (:1008-1012) */
    double split_sh_hue, split_sh_sat, split_hl_hue, split_hl_sat;
    double split_balance, split_blending;
    double glow, glow_radius, glow_threshold;

    /* Colour mode (Model.swift:973-979): "AsShot" or a known mode name.
     * colour_strength 0..100, 50 = the mode as-is.
     *
     * X3Lumina's ColorMode.detect() reads the embedded JPEG MakerNote for
     * Quattro files, because v4 headers have no colour-mode field. The C
     * helper x3f_render_probe_header_color_mode() only sees Merrill/classic
     * headers. The host (which already parses the embedded JPEG at import)
     * passes the detected CAMF short name here; empty means "use the header
     * probe, then fall back to Standard", matching resolvedColorMode(). */
    char   color_mode[32];
    char   detected_color_mode[32];
    double color_strength;

    /* Display chain misc */
    int    sfd_tone;        /* X3I SFD tone table */
    int    out_space;       /* X3F_OUT_* ; path B defaults to sRGB */
    int    look_defer;      /* 1 = scene-linear float output (engine front-end) */
    int    align_look_off;

    /* Decode-level extras */
    int    nlm;             /* export NLM strength 0..10, 0 = off */
    int    frame;           /* X3I frame index, 0 = 0EV */
    int    down;            /* preview downsample factor, >=1 */
} x3f_alcedo_edit_t;

/* Fills X3Lumina's product defaults, including the three user-requested
 * switches that X3Lumina itself ships OFF: overexposure (mode8), hole-fix and
 * Merrill/classic sky neutralisation are all ON here. The model gate in
 * x3f_alcedo_foveon_resolve_oe() applies them by camera generation. */
void x3f_alcedo_edit_default(x3f_alcedo_edit_t *edit);

/* Resolve the mutually exclusive decode modes exactly like Model.swift:664:
 *   Quattro                  -> oe ? 8 : 4   (hole-fix may apply)
 *   Merrill / classic + sky  -> 3            (overrides the oe selector)
 * Returns the value for x3f_render_set_oe_neutralize(). */
int  x3f_alcedo_foveon_resolve_oe(const x3f_alcedo_edit_t *edit, int model_class);

/* Hash of every decode-level field. If it changes, the prepared frame is
 * stale and x3f_alcedo_foveon_prepare() must run again. Look sliders are NOT
 * part of this key: they only need apply(). */
uint64_t x3f_alcedo_foveon_decode_key(const x3f_alcedo_edit_t *edit);

/* Opaque prepared frame (cached lin + clip + geometry). */
typedef struct x3f_alcedo_foveon_frame x3f_alcedo_foveon_frame_t;

/* Stage 1: decode one (optionally downsampled) linear frame and cache it.
 * Applies all decode-level setters before loading. Returns X3F_ALCEDO_OK. */
int  x3f_alcedo_foveon_prepare(const char *path, const x3f_alcedo_edit_t *edit,
                               x3f_alcedo_foveon_frame_t **out);
void x3f_alcedo_foveon_free(x3f_alcedo_foveon_frame_t *frame);

/* Dimensions/orientation of a prepared frame (orientation follows the same
 * header+EXIF fallback as path A). */
void x3f_alcedo_foveon_geometry(const x3f_alcedo_foveon_frame_t *frame,
                                int *w, int *h, int *orientation);

typedef struct {
    uint16_t *rgb;          /* w*h*3, 16-bit sRGB display code values */
    int       width;
    int       height;
    int       orientation;  /* EXIF 1..8, image is in sensor order */
    double   *linear_rgb;   /* non-NULL only when look_defer=1, w*h*3 f32-ish */
} x3f_alcedo_foveon_image_t;

/* Stage 2: re-run the display chain with the current look sliders. Cheap
 * enough to call on every slider change at preview resolution. When
 * edit->look_defer is set, linear_rgb holds unclamped scene-linear floats
 * instead of (in addition to) the 16-bit image. */
int  x3f_alcedo_foveon_apply(x3f_alcedo_foveon_frame_t *frame,
                             const x3f_alcedo_edit_t *edit,
                             x3f_alcedo_foveon_image_t *out);
void x3f_alcedo_foveon_image_free(x3f_alcedo_foveon_image_t *image);

/* Convenience: prepare + apply in one call (export / batch / tests). */
int  x3f_alcedo_foveon_render(const char *path, const x3f_alcedo_edit_t *edit,
                              x3f_alcedo_foveon_image_t *out);

const char *x3f_alcedo_foveon_error(void);

#ifdef __cplusplus
}
#endif

#endif /* X3F_ALCEDO_FOVEON_H */
