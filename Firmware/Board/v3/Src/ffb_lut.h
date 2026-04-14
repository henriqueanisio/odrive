/* ffb_lut.h — Torque Linearization Look-Up Table for Direct Drive FFB
 *
 * PURPOSE
 * ───────
 * Every motor/gearbox combination has a nonlinear force output:
 *   • Cogging torque causes dead-zones near zero
 *   • Magnetic saturation compresses high-end torque
 *   • Friction/backlash adds asymmetry
 *
 * The LUT maps a normalized desired torque [-1..1] to a corrected command
 * [-1..1] that compensates these nonlinearities.  The LUT is symmetric
 * around zero: only magnitude is corrected, sign is preserved.
 *
 * CALIBRATION (one-time procedure)
 * ─────────────────────────────────
 * 1. Mount a load cell on the wheel rim at a known radius R.
 * 2. For each of the N input levels k = 0, 1, …, N-1:
 *      • Command  T_cmd  = (k / (N-1)) × max_torque  to the motor.
 *      • Measure  F_actual  from the load cell.
 *      • Compute  T_actual  = F_actual × R.
 * 3. Build the inverse map:
 *      LUT[k] = T_cmd / T_actual          (if motor is weak)
 *    or, equivalently, adjust each point so that commanding LUT[k]×max_t
 *    produces the desired linear force.
 * 4. Normalise: LUT[k] ∈ [0..1].
 * 5. Load via GUI / flash storage (CFG_FFB_LUT_P0..P7).
 *
 * INTERPOLATION
 * ─────────────
 * Linear interpolation between the 8 stored points (O(1), no powf).
 * 8 points give ≤ 0.7% error vs true curve for smooth motor responses.
 * Increase FFB_LUT_POINTS (to 16) if higher precision is required — only
 * requires more CFG IDs (1 per point) and a tiny bit more RAM.
 *
 * WHEN DISABLED (default)
 * ────────────────────────
 * ffb_lut_apply() returns its input unchanged — zero branch overhead.
 */

#ifndef FFB_LUT_H
#define FFB_LUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ── LUT dimensioning ────────────────────────────────────────────────────── */
/* 8 evenly-spaced breakpoints over |torque| ∈ [0..1]:
 *   point 0 → |torque| = 0.0  (zero torque)
 *   point 7 → |torque| = 1.0  (max torque)
 * Increase to 16 for higher resolution if motor is strongly nonlinear.     */
#define FFB_LUT_POINTS  8U

/* ── LUT descriptor ─────────────────────────────────────────────────────── */
typedef struct {
    float p[FFB_LUT_POINTS];   /* corrected output values ∈ [0..1]          */
    bool  enabled;             /* false → identity passthrough, zero cost    */
} FFB_LUT_t;

/* ── Public API ─────────────────────────────────────────────────────────── */

/* Apply the LUT to a signed, normalized torque x ∈ [-1..1].
 *   • Sign (direction) is preserved.
 *   • Only magnitude is remapped through the LUT.
 *   • Linear interpolation between breakpoints.
 *   • If lut == NULL or lut->enabled == false: returns x unchanged.        */
float ffb_lut_apply(const FFB_LUT_t *lut, float x);

/* Set all breakpoints to the identity mapping (straight line 0→0 … 1→1).
 * Call this to reset to no-correction baseline.                             */
void  ffb_lut_set_identity(FFB_LUT_t *lut);

/* Build an FFB_LUT_t view directly from the raw config array.
 * Use this to avoid struct copies inside the control loop.
 *   raw_pts : pointer to FFB_LUT_POINTS floats (g_config.ffb_lut[])
 *   enabled : g_config.ffb_lut_enabled != 0                               */
void  ffb_lut_from_config(FFB_LUT_t *out, const float *raw_pts, bool enabled);

#ifdef __cplusplus
}
#endif

#endif /* FFB_LUT_H */
