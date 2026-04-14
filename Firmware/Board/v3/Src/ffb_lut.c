/* ffb_lut.c — Torque Linearization LUT implementation
 *
 * All functions are O(1) and branch-predictable.
 * No dynamic allocation, no floating-point division in the hot path.
 */

#include "ffb_lut.h"
#include <string.h>   /* memcpy */

/* ── ffb_lut_set_identity ────────────────────────────────────────────────── */
void ffb_lut_set_identity(FFB_LUT_t *lut)
{
    if (!lut) return;
    for (uint8_t i = 0; i < FFB_LUT_POINTS; i++) {
        /* evenly spaced: point i maps to i / (N-1) */
        lut->p[i] = (float)i / (float)(FFB_LUT_POINTS - 1U);
    }
    lut->enabled = false;
}

/* ── ffb_lut_from_config ─────────────────────────────────────────────────── */
void ffb_lut_from_config(FFB_LUT_t *out, const float *raw_pts, bool enabled)
{
    if (!out || !raw_pts) return;
    memcpy(out->p, raw_pts, sizeof(float) * FFB_LUT_POINTS);
    out->enabled = enabled;
}

/* ── ffb_lut_apply ───────────────────────────────────────────────────────── */
/*
 * Symmetric LUT evaluation:
 *
 *   |x| ─── map to float index ─── interpolate ─── restore sign
 *
 * index_f = |x| × (N-1)          ∈ [0 .. N-1]
 * lo      = (uint8_t) index_f    ∈ [0 .. N-2]
 * frac    = index_f - lo         ∈ [0 .. 1)
 * out     = p[lo] + frac × (p[lo+1] - p[lo])
 */
float ffb_lut_apply(const FFB_LUT_t *lut, float x)
{
    if (!lut || !lut->enabled) return x;

    /* Extract sign and magnitude */
    float sign = (x >= 0.0f) ? 1.0f : -1.0f;
    float mag  = (x < 0.0f) ? -x : x;

    /* Clamp magnitude to valid range */
    if (mag >= 1.0f) {
        return sign * lut->p[FFB_LUT_POINTS - 1U];
    }
    if (mag <= 0.0f) {
        return sign * lut->p[0];
    }

    /* Map to LUT index (no division — multiply by precomputed (N-1)) */
    float   idx_f = mag * (float)(FFB_LUT_POINTS - 1U);
    uint8_t lo    = (uint8_t)idx_f;

    /* Guard against rounding artifacts at the top */
    if (lo >= FFB_LUT_POINTS - 1U) {
        return sign * lut->p[FFB_LUT_POINTS - 1U];
    }

    /* Linear interpolation: cheaper than any higher-order scheme and
     * indistinguishable from cubic at 8 breakpoints for smooth curves. */
    float frac = idx_f - (float)lo;
    float out  = lut->p[lo] + frac * (lut->p[lo + 1U] - lut->p[lo]);

    return sign * out;
}
