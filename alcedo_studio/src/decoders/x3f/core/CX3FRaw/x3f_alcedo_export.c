/* x3f_alcedo_export.c — 相机原生 scene-linear 出口实现。
 * SPDX-License-Identifier: GPL-3.0-only
 * See x3f_alcedo_export.h for the colour-management contract.
 */

#include "x3f_alcedo_export.h"

#include "x3f_render.h"
#include "x3fcore.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#  include <windows.h>
#  include <io.h>
#else
#  include <pthread.h>
#  include <unistd.h>
#endif

/* x3f_render.h is included for x3f_denoise_t / x3f_edit_defaults / the SPP
 * sidecar. It is unambiguous (only CX3FRaw ships that name; x3f_io.h and
 * x3f_process.h are the duplicated ones). The dispatch + metadata helpers
 * below are still declared by hand because the legacy tree defines ABI-
 * incompatible x3f_t / x3f_* types under the same names. */
extern int  x3f_own_dispatch_scene_linear(const char *path, float **data,
                                          unsigned *w, unsigned *h,
                                          unsigned short *orientation,
                                          double *camera_to_rgb,
                                          int highlight_recovery,
                                          unsigned char denoise_intensity,
                                          const void *denoise,
                                          const char *wb_preset);
extern void x3f_decoder_set_override(const char *d);
extern int  x3f_render_probe_is_quattro(const char *path);
extern int  x3f_render_probe_is_merrill(const char *path);
extern int  x3f_render_probe_is_quattro_gen(const char *path);
extern int  x3f_render_probe_is_sdq_body(const char *path);
extern int  x3f_render_probe_sdq_hl(const char *path);
/* CAMF reader from the render layer (declared by hand because x3f_render.h
 * pulls in the legacy x3f_t layout, which this file deliberately avoids). */
extern int  x3f_camf_matrix_f64(const char *path, const char *name,
                                double *out, int n, unsigned *dims);
#define X3F_CAMF_DIMS_LOCAL 3

/* x3fuse metadata-only helpers (libx3f.a). Declared with void* so the vendor
 * x3f_t layout stays out of this translation unit (the legacy tree defines
 * same-named types). These read the header, directory and CAMF block only. */
extern void *x3f_new_from_file(void *infile);
extern int   x3f_delete(void *x3f);
extern void *x3f_get_camf(void *x3f);
extern int   x3f_load_data(void *x3f, void *entry);
extern char *x3f_get_wb(void *x3f);
extern int   x3f_get_raw_to_xyz(void *x3f, char *wb, double *out);

static char g_error[256] = {0};

/* The decoder override lives in a process-wide global inside x3fdec.c. Holding
 * a lock across the decode keeps concurrent X3F requests from switching each
 * other's backend. A per-call decoder argument would remove this bottleneck. */
#ifdef _WIN32
static CRITICAL_SECTION g_lock;
static INIT_ONCE        g_once = INIT_ONCE_STATIC_INIT;
static BOOL CALLBACK init_lock(PINIT_ONCE once, PVOID param, PVOID *ctx) {
    (void)once; (void)param; (void)ctx;
    InitializeCriticalSection(&g_lock);
    return TRUE;
}
static void lock_decoder(void) {
    InitOnceExecuteOnce(&g_once, init_lock, NULL, NULL);
    EnterCriticalSection(&g_lock);
}
static void unlock_decoder(void) { LeaveCriticalSection(&g_lock); }
#else
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static void lock_decoder(void)   { pthread_mutex_lock(&g_lock); }
static void unlock_decoder(void) { pthread_mutex_unlock(&g_lock); }
#endif

static void set_error(const char *msg) {
    snprintf(g_error, sizeof(g_error), "%s", msg);
}

static void mat_mul(const double a[9], const double b[9], double out[9]) {
    double t[9];
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) s += a[r * 3 + k] * b[k * 3 + c];
            t[r * 3 + c] = s;
        }
    }
    memcpy(out, t, sizeof(t));
}

/* Adjugate / determinant. Returns 0 for a singular or degenerate matrix:
 * silently passing one through would collapse the gamut, so callers must fail. */
static int mat_inv(const double m[9], double out[9]) {
    const double a = m[0], b = m[1], c = m[2];
    const double d = m[3], e = m[4], f = m[5];
    const double g = m[6], h = m[7], i = m[8];
    const double A = e * i - f * h;
    const double B = f * g - d * i;
    const double C = d * h - e * g;
    const double det = a * A + b * B + c * C;
    if (!(fabs(det) > 1e-12) || !isfinite(det)) return 0;
    const double t[9] = {
        A / det, (c * h - b * i) / det, (b * f - c * e) / det,
        B / det, (a * i - c * g) / det, (c * d - a * f) / det,
        C / det, (b * g - a * h) / det, (a * e - b * d) / det
    };
    memcpy(out, t, sizeof(t));
    return 1;
}

/* Matches own_decoder.c::srgb_to_xyz, so cam_xyz is the exact inverse of the
 * XYZ_to_sRGB step already folded into camera_to_rgb. */
static void srgb_to_xyz(double m[9]) {
    m[0] = 0.4124; m[1] = 0.3576; m[2] = 0.1805;
    m[3] = 0.2126; m[4] = 0.7152; m[5] = 0.0722;
    m[6] = 0.0193; m[7] = 0.1192; m[8] = 0.9505;
}

const char *x3f_alcedo_export_error(void) { return g_error; }

int x3f_alcedo_model_class(const char *path) {
    if (!path || !*path) return X3F_ALCEDO_CLASSIC;
    /* Decode-backend grain, not the array grain: SD Quattro / SD Quattro H are
     * Quattro bodies with their own RAW type_format (0x25 / 0x27) and must use
     * own decoding like every other Quattro. The render layer keeps its narrow
     * probe where the distinction matters (Quattro green calibration). */
    if (x3f_render_probe_is_quattro_gen(path)) return X3F_ALCEDO_QUATTRO;
    if (x3f_render_probe_is_merrill(path)) return X3F_ALCEDO_MERRILL;
    return X3F_ALCEDO_CLASSIC;
}

/* SD Quattro / SD Quattro H 是独立的 type_format（0x25 / 0x27），旧探针只认
 * DP-Q 的 0x23。面板门控必须用这个更宽的世代口径，否则 SDQ/SDQH 的过曝矫正
 * 与 SDQ 高光会被误置灰（2026-11-16 实测报障）。 */
int x3f_alcedo_model_gen(const char *path) {
    if (!path || !*path) return X3F_ALCEDO_CLASSIC;
    if (x3f_render_probe_is_quattro_gen(path)) return X3F_ALCEDO_QUATTRO;
    if (x3f_render_probe_is_merrill(path)) return X3F_ALCEDO_MERRILL;
    return X3F_ALCEDO_CLASSIC;
}

int x3f_alcedo_sdq_hl_capable(const char *path) {
    if (!path || !*path) return 0;
    /* SDQH only (user decision 2026-11-16): the slider exists to normalise the
     * DigitalISOGain that SD Quattro H reports as (4,4,2). Plain SD Quattro
     * reports (1,1,1) — the same slider there multiplies by 1.0 and does
     * nothing, so it must not be offered at all rather than sit inert. */
    return (x3f_render_probe_is_sdq_body(path) && x3f_render_probe_sdq_hl(path))
               ? 1
               : 0;
}

int x3f_alcedo_sdq_hl_effective(const char *path) {
    if (!path || !*path) return 0;
    return x3f_render_probe_sdq_hl(path) ? 1 : 0;
}

int x3f_alcedo_available_decoders(const char *path) {
    const int model = x3f_alcedo_model_class(path);
    switch (model) {
        case X3F_ALCEDO_QUATTRO:
            return X3F_ALCEDO_HAS_OWN | X3F_ALCEDO_HAS_FUSE;
        case X3F_ALCEDO_MERRILL:
            return X3F_ALCEDO_HAS_FUSE | X3F_ALCEDO_HAS_SPP;
        default:
            return X3F_ALCEDO_HAS_FUSE;
    }
}

static unsigned read_le32(const unsigned char *p) {
    return (unsigned)p[0] | ((unsigned)p[1] << 8) | ((unsigned)p[2] << 16) |
           ((unsigned)p[3] << 24);
}

int x3f_alcedo_header_orientation(const char *path) {
    if (!path || !*path) return 1;
    FILE *f = fopen(path, "rb");
    if (!f) return 1;
    unsigned char b[40] = {0};
    const size_t  n = fread(b, 1, sizeof(b), f);
    fclose(f);
    /* FOVi containers have a different header layout (no rotation field);
     * v4+ Quattro headers do not carry it either. */
    if (n < 40 || memcmp(b, "FOVb", 4) != 0) return 1;
    if (read_le32(b + 4) >= 0x00040000u) return 1;
    switch (read_le32(b + 36)) {
        case 90:  return 6;
        case 180: return 3;
        case 270: return 8;
        default:  return 1;
    }
}

int x3f_alcedo_probe_color(const char *path, int frame, double cam_to_xyz[9]) {
    if (!path || !*path || !cam_to_xyz) {
        set_error("x3f_alcedo_probe_color: null argument");
        return X3F_ALCEDO_ERR_ARG;
    }
    char        tmp[1024] = {0};
    const char *use = x3f_standalone_path(path, frame < 0 ? -1 : frame, tmp, sizeof(tmp));
    if (!use || !*use) {
        set_error("x3f_alcedo_probe_color: cannot materialize X3I frame");
        return X3F_ALCEDO_ERR_CONTAINER;
    }
    FILE *f = fopen(use, "rb");
    if (!f) {
        x3f_temp_release(tmp);
        set_error("x3f_alcedo_probe_color: cannot open X3F");
        return X3F_ALCEDO_ERR_CONTAINER;
    }
    int   rc  = X3F_ALCEDO_ERR_DECODE;
    void *x3f = x3f_new_from_file(f);
    if (x3f) {
        void *camf = x3f_get_camf(x3f);
        if (camf) x3f_load_data(x3f, camf);
        char *wb = x3f_get_wb(x3f);
        if (x3f_get_raw_to_xyz(x3f, wb, cam_to_xyz) == 1) rc = X3F_ALCEDO_OK;
        x3f_delete(x3f);
    }
    if (rc == X3F_ALCEDO_OK) {
        /* The decoder folds DigitalISOGain into camera_to_rgb as a column
         * scale, so the emitted pixels carry it. The probe matrix must carry
         * it too, otherwise the document profile and the decoded values differ
         * by that diagonal and the frame shifts colour (SDQH is 4,4,2 and shows
         * up as a green cast). DP-Q / Merrill / SDQ report 1,1,1 and are
         * bit-identical. */
        double   dg[3]                    = {1.0, 1.0, 1.0};
        unsigned dims[X3F_CAMF_DIMS_LOCAL] = {1, 1, 1};
        if (x3f_camf_matrix_f64(use, "DigitalISOGain", dg, 3, dims) == 3) {
            int valid = 1;
            for (int i = 0; i < 3; ++i) {
                if (!(dg[i] > 0.0) || dg[i] > 64.0) valid = 0;
            }
            if (valid) {
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) cam_to_xyz[r * 3 + c] *= dg[c];
                }
            }
        }
    }
    fclose(f);
    x3f_temp_release(tmp);
    if (rc != X3F_ALCEDO_OK) {
        set_error("x3f_alcedo_probe_color: CAMF colour matrix unavailable");
    }
    return rc;
}

void x3f_alcedo_decode_opts_default(x3f_alcedo_decode_opts_t *o) {
    if (!o) return;
    memset(o, 0, sizeof(*o));
    o->decoder           = X3F_ALCEDO_DECODER_AUTO;
    o->denoise_luminance = X3F_DN_LUMA_AUTO;
    o->denoise_chroma    = 1;
    o->denoise_strength  = 0.0;  /* <=0 keeps x3f_denoise_default()'s 1.0 */
}

static int export_linear_impl(const char *path, int frame,
                              const x3f_alcedo_decode_opts_t *opts,
                              x3f_alcedo_linear_t *out) {
    if (!path || !*path || !out) {
        set_error("x3f_alcedo_export_linear: null argument");
        return X3F_ALCEDO_ERR_ARG;
    }
    memset(out, 0, sizeof(*out));
    g_error[0] = 0;

    x3f_alcedo_decode_opts_t fallback;
    if (!opts) {
        x3f_alcedo_decode_opts_default(&fallback);
        opts = &fallback;
    }
    const int decoder = opts->decoder;
    if (decoder == X3F_ALCEDO_DECODER_SPP) {
        set_error("SPP native chain is a finished-image sidecar, not a scene-linear source");
        return X3F_ALCEDO_ERR_ARG;
    }

    const int model = x3f_alcedo_model_class(path);
    out->model_class = model;

    const char *override = NULL;
    switch (decoder) {
        case X3F_ALCEDO_DECODER_AUTO:
            override = (model == X3F_ALCEDO_QUATTRO) ? "own" : "fuse";
            break;
        case X3F_ALCEDO_DECODER_FUSE:
            override = "fuse";
            break;
        case X3F_ALCEDO_DECODER_OWN:
            override = "own";
            break;
        default:
            set_error("x3f_alcedo_export_linear: unknown decoder");
            return X3F_ALCEDO_ERR_ARG;
    }

    char tmp[1024] = {0};
    const char *use = x3f_standalone_path(path, frame < 0 ? -1 : frame, tmp, sizeof(tmp));
    if (!use || !*use) {
        set_error("x3f_alcedo_export_linear: cannot materialize X3I frame");
        return X3F_ALCEDO_ERR_CONTAINER;
    }

    float         *data = NULL;
    unsigned       w = 0, h = 0;
    unsigned short orientation = 0;
    double         camera_to_rgb[9] = {0};

    /* Decode-stage NLM. With intensity 0 we pass NULL, which is bit-identical
     * to the original path. These options never enter the display chain, so
     * path A keeps its unclamped camera-native f32 output. */
    x3f_denoise_t dn;
    x3f_denoise_default(&dn);
    if (opts->denoise_strength > 0.0) dn.strength = opts->denoise_strength;
    if (opts->denoise_luminance == X3F_DN_LUMA_OFF ||
        opts->denoise_luminance == X3F_DN_LUMA_ON) dn.luminance = opts->denoise_luminance;
    dn.chroma = opts->denoise_chroma ? 1 : 0;
    const int di = opts->denoise_intensity < 0 ? 0 :
                   (opts->denoise_intensity > 10 ? 10 : opts->denoise_intensity);
    const void *dnp = (di > 0) ? (const void *)&dn : NULL;

    lock_decoder();
    x3f_decoder_set_override(override);
    const int rc = x3f_own_dispatch_scene_linear(use, &data, &w, &h, &orientation,
                                                 camera_to_rgb,
                                                 opts->highlight_recovery ? 1 : 0,
                                                 (unsigned char)di, dnp, NULL);
    x3f_decoder_set_override(NULL);
    unlock_decoder();

    /* Resolve the decode-free colour matrix and the legacy-body orientation
     * before the temporary X3I frame is removed: `use` aliases tmp. Bulk
     * import binds the same M to the document camera profile, so the pixels
     * produced here must be expressed in exactly M's camera primaries. */
    double M[9] = {0};
    const int prc = x3f_alcedo_probe_color(use, -1, M);
    int    resolved_orientation = (int)orientation;
    if (resolved_orientation == 1) {
        resolved_orientation = x3f_alcedo_header_orientation(use);
    }

    x3f_temp_release(tmp);

    if (rc != 0 || !data || w == 0 || h == 0) {
        free(data);
        set_error("x3f_alcedo_export_linear: scene-linear decode failed");
        return X3F_ALCEDO_ERR_DECODE;
    }
    if (prc != X3F_ALCEDO_OK) {
        free(data);
        set_error("x3f_alcedo_export_linear: decode-free colour matrix unavailable");
        return prc;
    }

    /* scene is extended linear sRGB: XYZ = sRGB_to_XYZ * scene.
     * Emit pixels in M's camera-native space: M * out == XYZ, hence
     *     out = inverse(M) * sRGB_to_XYZ * scene.
     * No inverse(camera_to_rgb) is needed, and no clamp is applied: negative
     * channels and >1 headroom survive into the AP1 working space. */
    double invM[9], s2x[9], C[9];
    if (!mat_inv(M, invM)) {
        free(data);
        set_error("x3f_alcedo_export_linear: singular CAMF colour matrix");
        return X3F_ALCEDO_ERR_SINGULAR;
    }
    srgb_to_xyz(s2x);
    mat_mul(invM, s2x, C);

    const size_t n = (size_t)w * (size_t)h * 3u;
    for (size_t p = 0; p < n; p += 3) {
        const double r = data[p], g = data[p + 1], b = data[p + 2];
        data[p]     = (float)(C[0] * r + C[1] * g + C[2] * b);
        data[p + 1] = (float)(C[3] * r + C[4] * g + C[5] * b);
        data[p + 2] = (float)(C[6] * r + C[7] * g + C[8] * b);
    }

    memcpy(out->cam_xyz, M, sizeof(M));
    memcpy(out->camera_to_rgb, camera_to_rgb, sizeof(camera_to_rgb));

    /* Diagnostic for colour calibration: M is the matrix that the pixels above
     * are expressed in, C = inv(M)*sRGB_to_XYZ, i.e. the linear map applied to
     * the decoder's extended-linear sRGB. M * (1,1,1) tells whether unit camera
     * RGB is D65 (as_shot_neutral = 1,1,1 is only valid when it is). */
    if (getenv("X3F_ALCEDO_DEBUG_COLOR")) {
        const double w0 = M[0] + M[1] + M[2];
        const double w1 = M[3] + M[4] + M[5];
        const double w2 = M[6] + M[7] + M[8];
        fprintf(stderr, "[color] M = [%g %g %g; %g %g %g; %g %g %g]\n",
                M[0], M[1], M[2], M[3], M[4], M[5], M[6], M[7], M[8]);
        fprintf(stderr, "[color] M*(1,1,1) = [%g %g %g]  (D65 = 0.9505 1 1.089)\n",
                w0, w1, w2);
        fprintf(stderr, "[color] decoder camera_to_rgb = [%g %g %g; %g %g %g; %g %g %g]\n",
                camera_to_rgb[0], camera_to_rgb[1], camera_to_rgb[2],
                camera_to_rgb[3], camera_to_rgb[4], camera_to_rgb[5],
                camera_to_rgb[6], camera_to_rgb[7], camera_to_rgb[8]);
        fprintf(stderr, "[color] C = inv(M)*srgb_to_xyz = [%g %g %g; %g %g %g; %g %g %g]\n",
                C[0], C[1], C[2], C[3], C[4], C[5], C[6], C[7], C[8]);
    }

    out->rgb          = data;
    out->width        = (int)w;
    out->height       = (int)h;
    out->orientation  = resolved_orientation;
    out->decoder_used = (strcmp(override, "own") == 0) ? 1 : 0;
    return X3F_ALCEDO_OK;
}

/* Stage bytes in a private temp file. Returns 1 and fills out_path on success. */
static int stage_bytes(const void *bytes, size_t len, char *out_path, size_t cap) {
#ifdef _WIN32
    char dir[MAX_PATH];
    char path[MAX_PATH];
    if (!GetTempPathA((DWORD)sizeof(dir), dir) ||
        GetTempFileNameA(dir, "x3f", 0, path) == 0) return 0;
    FILE *f = fopen(path, "wb");
    if (!f) { DeleteFileA(path); return 0; }
    const int ok = (fwrite(bytes, 1, len, f) == len) && (fclose(f) == 0);
    if (!ok) { DeleteFileA(path); return 0; }
    if (strlen(path) >= cap) { DeleteFileA(path); return 0; }
    memcpy(out_path, path, strlen(path) + 1);
    return 1;
#else
    const char *dir = getenv("TMPDIR");
    if (!dir || !*dir) dir = P_tmpdir;
    char tmpl[1024];
    if (snprintf(tmpl, sizeof(tmpl), "%s/alcedo_x3f_XXXXXX", dir) >= (int)sizeof(tmpl)) return 0;
    int fd = mkstemp(tmpl);
    if (fd < 0) return 0;
    FILE *f = fdopen(fd, "wb");
    if (!f) { close(fd); unlink(tmpl); return 0; }
    const int ok = (fwrite(bytes, 1, len, f) == len) && (fclose(f) == 0);
    if (!ok) { unlink(tmpl); return 0; }
    if (strlen(tmpl) >= cap) { unlink(tmpl); return 0; }
    memcpy(out_path, tmpl, strlen(tmpl) + 1);
    return 1;
#endif
}

static void unstage(const char *path) {
#ifdef _WIN32
    DeleteFileA(path);
#else
    unlink(path);
#endif
}

/* ---- Public wrappers: path A stays bit-identical when no options are given. */
int x3f_alcedo_export_linear(const char *path, int frame, int decoder,
                             x3f_alcedo_linear_t *out) {
    x3f_alcedo_decode_opts_t opts;
    x3f_alcedo_decode_opts_default(&opts);
    opts.decoder = decoder;
    return export_linear_impl(path, frame, &opts, out);
}

int x3f_alcedo_export_linear_ex(const char *path, int frame,
                                const x3f_alcedo_decode_opts_t *opts,
                                x3f_alcedo_linear_t *out) {
    return export_linear_impl(path, frame, opts, out);
}

static int export_linear_memory_impl(const void *bytes, size_t len,
                                     const x3f_alcedo_decode_opts_t *opts,
                                     x3f_alcedo_linear_t *out) {
    if (!out) {
        set_error("x3f_alcedo_export_linear_memory: null argument");
        return X3F_ALCEDO_ERR_ARG;
    }
    memset(out, 0, sizeof(*out));
    if (!bytes || len < 8 || memcmp(bytes, "FOVb", 4) != 0) {
        set_error("x3f_alcedo_export_linear_memory: expected a standalone FOVb frame");
        return X3F_ALCEDO_ERR_ARG;
    }
    char tmp[1024] = {0};
    if (!stage_bytes(bytes, len, tmp, sizeof(tmp))) {
        set_error("x3f_alcedo_export_linear_memory: cannot stage buffer");
        return X3F_ALCEDO_ERR_NOMEM;
    }
    const int rc = export_linear_impl(tmp, -1, opts, out);
    unstage(tmp);
    return rc;
}

int x3f_alcedo_export_linear_memory(const void *bytes, size_t len, int decoder,
                                    x3f_alcedo_linear_t *out) {
    x3f_alcedo_decode_opts_t opts;
    x3f_alcedo_decode_opts_default(&opts);
    opts.decoder = decoder;
    return export_linear_memory_impl(bytes, len, &opts, out);
}

int x3f_alcedo_export_linear_memory_ex(const void *bytes, size_t len,
                                       const x3f_alcedo_decode_opts_t *opts,
                                       x3f_alcedo_linear_t *out) {
    return export_linear_memory_impl(bytes, len, opts, out);
}

/* ---- Path B sidecar: Merrill SPP native-chain 16-bit finished image. ----
 * x3f_render_spp_* is gated by a process-wide switch; we drive it with the
 * explicit setter (never setenv) and restore -1 ("follow env") afterwards so
 * the default render path is unchanged for other callers. */
int x3f_alcedo_spp_capable(const char *path, int frame, int *w, int *h) {
    if (w) *w = 0;
    if (h) *h = 0;
    if (!path || !*path || !w || !h) return 0;
    char tmp[1024] = {0};
    const char *use = x3f_standalone_path(path, frame, tmp, sizeof(tmp));
    if (!use || !*use) return 0;
    lock_decoder();
    x3f_render_spp_set_enabled(1);
    const int ok = x3f_render_spp_capable(use, w, h);
    x3f_render_spp_set_enabled(-1);
    unlock_decoder();
    x3f_temp_release(tmp);
    return ok;
}

int x3f_alcedo_export_spp16(const char *path, int frame,
                            x3f_alcedo_spp16_t *out) {
    if (!out || !path || !*path) {
        set_error("x3f_alcedo_export_spp16: null argument");
        return X3F_ALCEDO_ERR_ARG;
    }
    memset(out, 0, sizeof(*out));
    int w = 0, h = 0;
    if (!x3f_alcedo_spp_capable(path, frame, &w, &h) || w <= 0 || h <= 0) {
        set_error("x3f_alcedo_export_spp16: file is not SPP-native capable");
        return X3F_ALCEDO_ERR_ARG;
    }
    const size_t n = (size_t)w * (size_t)h * 3u;
    uint16_t *buf = (uint16_t *)calloc(n, sizeof(uint16_t));
    if (!buf) {
        set_error("x3f_alcedo_export_spp16: out of memory");
        return X3F_ALCEDO_ERR_NOMEM;
    }
    char tmp[1024] = {0};
    const char *use = x3f_standalone_path(path, frame, tmp, sizeof(tmp));
    if (!use || !*use) {
        free(buf);
        set_error("x3f_alcedo_export_spp16: cannot materialize frame");
        return X3F_ALCEDO_ERR_CONTAINER;
    }
    int ow = w, oh = h;
    lock_decoder();
    x3f_render_spp_set_enabled(1);
    const int rc = x3f_render_spp_export(use, buf, w * 3 * (int)sizeof(uint16_t), &ow, &oh);
    x3f_render_spp_set_enabled(-1);
    unlock_decoder();
    const int orientation = x3f_alcedo_header_orientation(use);
    x3f_temp_release(tmp);
    if (rc != X3FRENDER_OK) {
        free(buf);
        set_error("x3f_alcedo_export_spp16: SPP render failed");
        return X3F_ALCEDO_ERR_DECODE;
    }
    out->rgb         = buf;
    out->width       = w;
    out->height      = h;
    out->orientation = orientation;
    return X3F_ALCEDO_OK;
}

/* ---- SPP tone-stage bridge: the six X3Lumina sliders stay in SPP's own tone
 * stage. They are not Alcedo Develop-DAG parameters: the DAG works in
 * ACEScc/AP1 and has no equivalent of SPP's camera-code-value tone curve. ---- */
struct x3f_alcedo_spp_session {
    struct x3f_spp_prepared *prepared;
    int  width;
    int  height;
    int  orientation;
    int  nr_level;
    char temp_path[1024];
};

int x3f_alcedo_spp_session_open(const char *path, int frame, int nr_level,
                                x3f_alcedo_spp_session **out,
                                int *w, int *h) {
    if (w) *w = 0;
    if (h) *h = 0;
    if (!out || !path || !*path) {
        set_error("x3f_alcedo_spp_session_open: null argument");
        return X3F_ALCEDO_ERR_ARG;
    }
    *out = NULL;
    if (nr_level < -1 || nr_level > 4) {
        set_error("x3f_alcedo_spp_session_open: nr_level must be -1 or 0..4");
        return X3F_ALCEDO_ERR_ARG;
    }
    char tmp[1024] = {0};
    const char *use = x3f_standalone_path(path, frame < 0 ? -1 : frame,
                                          tmp, sizeof(tmp));
    if (!use || !*use) {
        set_error("x3f_alcedo_spp_session_open: cannot materialize frame");
        return X3F_ALCEDO_ERR_CONTAINER;
    }

    struct x3f_spp_prepared *prepared = NULL;
    int ow = 0, oh = 0;
    lock_decoder();
    x3f_render_spp_set_enabled(1);
    const int rc = x3f_render_spp_prepare(use, nr_level, &prepared, &ow, &oh);
    x3f_render_spp_set_enabled(-1);
    unlock_decoder();

    if (rc != X3FRENDER_OK || prepared == NULL || ow <= 0 || oh <= 0) {
        if (prepared != NULL) x3f_render_spp_prepared_free(prepared);
        x3f_temp_release(tmp);
        set_error("x3f_alcedo_spp_session_open: SPP prepare failed");
        return X3F_ALCEDO_ERR_DECODE;
    }

    x3f_alcedo_spp_session *session =
        (x3f_alcedo_spp_session *)calloc(1, sizeof(*session));
    if (session == NULL) {
        x3f_render_spp_prepared_free(prepared);
        x3f_temp_release(tmp);
        set_error("x3f_alcedo_spp_session_open: out of memory");
        return X3F_ALCEDO_ERR_NOMEM;
    }
    session->prepared = prepared;
    session->width = ow;
    session->height = oh;
    session->nr_level = nr_level;
    const int orientation = x3f_alcedo_header_orientation(use);
    session->orientation = (orientation >= 1 && orientation <= 8) ? orientation : 1;
    if (tmp[0] != '\0') {
        snprintf(session->temp_path, sizeof(session->temp_path), "%s", tmp);
    }
    *out = session;
    if (w) *w = ow;
    if (h) *h = oh;
    return X3F_ALCEDO_OK;
}

int x3f_alcedo_spp_session_apply(x3f_alcedo_spp_session *session,
                                 const x3f_alcedo_spp_tone_t *tone,
                                 uint16_t *dst, int stride, int *w, int *h) {
    if (w) *w = 0;
    if (h) *h = 0;
    if (!session || !session->prepared || !tone || !dst) {
        set_error("x3f_alcedo_spp_session_apply: null argument");
        return X3F_ALCEDO_ERR_ARG;
    }
    if (tone->nr_level != session->nr_level) {
        set_error("x3f_alcedo_spp_session_apply: nr_level differs from prepare");
        return X3F_ALCEDO_ERR_ARG;
    }
    if (stride < session->width * 3 * (int)sizeof(uint16_t)) {
        set_error("x3f_alcedo_spp_session_apply: stride too small");
        return X3F_ALCEDO_ERR_ARG;
    }
    int ow = session->width;
    int oh = session->height;
    lock_decoder();
    x3f_render_spp_set_enabled(1);
    const int rc = x3f_render_spp_apply_tone(
        session->prepared, dst, stride, &ow, &oh, tone->exposure, tone->contrast,
        tone->shadows, tone->highlights, tone->saturation, tone->nr_level);
    x3f_render_spp_set_enabled(-1);
    unlock_decoder();
    if (rc != X3FRENDER_OK) {
        set_error("x3f_alcedo_spp_session_apply: SPP tone stage failed");
        return X3F_ALCEDO_ERR_DECODE;
    }
    if (w) *w = ow;
    if (h) *h = oh;
    return X3F_ALCEDO_OK;
}

int x3f_alcedo_spp_session_orientation(const x3f_alcedo_spp_session *session) {
    if (!session) return 1;
    return (session->orientation >= 1 && session->orientation <= 8)
               ? session->orientation
               : 1;
}

void x3f_alcedo_spp_session_close(x3f_alcedo_spp_session *session) {
    if (!session) return;
    if (session->prepared != NULL) {
        x3f_render_spp_prepared_free(session->prepared);
        session->prepared = NULL;
    }
    x3f_temp_release(session->temp_path);
    free(session);
}

int x3f_alcedo_spp_tone_export(const char *path, int frame,
                               const x3f_alcedo_spp_tone_t *tone,
                               x3f_alcedo_spp16_t *out) {
    if (!out || !tone || !path || !*path) {
        set_error("x3f_alcedo_spp_tone_export: null argument");
        return X3F_ALCEDO_ERR_ARG;
    }
    memset(out, 0, sizeof(*out));

    x3f_alcedo_spp_session *session = NULL;
    int w = 0, h = 0;
    int rc = x3f_alcedo_spp_session_open(path, frame, tone->nr_level, &session,
                                         &w, &h);
    if (rc != X3F_ALCEDO_OK || !session || w <= 0 || h <= 0) {
        if (session) x3f_alcedo_spp_session_close(session);
        return rc != X3F_ALCEDO_OK ? rc : X3F_ALCEDO_ERR_DECODE;
    }

    const size_t n = (size_t)w * (size_t)h * 3u;
    uint16_t *buf = (uint16_t *)calloc(n, sizeof(uint16_t));
    if (!buf) {
        x3f_alcedo_spp_session_close(session);
        set_error("x3f_alcedo_spp_tone_export: out of memory");
        return X3F_ALCEDO_ERR_NOMEM;
    }

    rc = x3f_alcedo_spp_session_apply(session, tone, buf,
                                      w * 3 * (int)sizeof(uint16_t), &w, &h);
    const int orientation = x3f_alcedo_spp_session_orientation(session);
    x3f_alcedo_spp_session_close(session);
    if (rc != X3F_ALCEDO_OK) {
        free(buf);
        return rc;
    }
    out->rgb = buf;
    out->width = w;
    out->height = h;
    out->orientation = orientation;
    return X3F_ALCEDO_OK;
}

void x3f_alcedo_spp16_free(x3f_alcedo_spp16_t *out) {
    if (!out) return;
    free(out->rgb);
    memset(out, 0, sizeof(*out));
}

void x3f_alcedo_export_free(x3f_alcedo_linear_t *out) {
    if (!out) return;
    /* Both x3fuse and the own core allocate with libc malloc, and the dispatch
     * free() consults a thread-local flag. free() is allocator-correct and safe
     * when the buffer is released on a different thread. */
    free(out->rgb);
    memset(out, 0, sizeof(*out));
}
