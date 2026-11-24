/* Host-test scan generator: a flat box face in front of the stopped
   robot, seen by a cone-shaped ultrasonic beam on the 30 mm servo arm,
   run through the production angle lists (s5_coarse_angles,
   s5_fine_angles, s5_merge_trips).  NOT a measured sensor model; it only
   makes repeatable fixtures.  Per angle:
     the beam axis hits the face        -> 3 echoes   [r, r+2, r-1]
     only the cone edge reaches it      -> 1 echo     [0, r, 0]
     nothing                            -> background ping pattern
   Include after obstacle_logic.c and s5_scan_diag.c. */
#ifndef S5_SCAN_MODEL_H
#define S5_SCAN_MODEL_H
#include <math.h>

typedef struct
{
    double face_mm;   /* sensor face (servo at 0) to the box face, straight ahead */
    double centre_mm; /* box centre, + = left */
    double width_mm;  /* box face width */
    double beam_deg;  /* cone half-angle */
    double arm_mm;    /* servo axis to transducer face */
    uint16_t bg[3];   /* pings when nothing is hit (0 = silence) */
} s5_model_t;

static double model_ray(const s5_model_t * m, double sx, double sy, double deg)
{
    double t = deg * 3.14159265358979 / 180.0;
    double X = m->arm_mm + m->face_mm;
    double r, y;
    if (cos(t) <= 1e-6) { return 1e9; }
    r = (X - sx) / cos(t);
    y = sy + r * sin(t);
    if ((y < m->centre_mm - m->width_mm / 2.0) || (y > m->centre_mm + m->width_mm / 2.0)) { return 1e9; }
    return r;
}

static s5_trip_t model_trip(const s5_model_t * m, int deg, s5_src_t src)
{
    double t = deg * 3.14159265358979 / 180.0;
    double sx = m->arm_mm * cos(t), sy = m->arm_mm * sin(t);
    double axis = model_ray(m, sx, sy, deg), best = 1e9, d;
    s5_trip_t tr;
    for (d = -m->beam_deg; d <= m->beam_deg + 1e-9; d += 0.5)
    {
        double r = model_ray(m, sx, sy, deg + d);
        if (r < best) { best = r; }
    }
    tr.deg = (int8_t)deg; tr.src = (uint8_t)src;
    if (axis < 1e8)
    {
        uint16_t r = (uint16_t)(best + 0.5);
        tr.raw[0] = r; tr.raw[1] = (uint16_t)(r + 2); tr.raw[2] = (uint16_t)(r - 1);
    }
    else if (best < 1e8)
    {
        tr.raw[0] = 0; tr.raw[1] = (uint16_t)(best + 0.5); tr.raw[2] = 0;
    }
    else
    {
        tr.raw[0] = m->bg[0]; tr.raw[1] = m->bg[1]; tr.raw[2] = m->bg[2];
    }
    tr.used = s5_median_nonzero(tr.raw, 3);
    return tr;
}

/* The scan task's flow (do_scan) with the model as the sensor.  Returns
   the trips behind the final profile (merged, or coarse on fallback). */
static int32_t model_scan(const s5_model_t * m, s5_trip_t * out, int32_t max)
{
    int8_t ang[64];
    s5_trip_t c[8], f[64];
    s5_sample_t cs[8], ms[64];
    s5_profile_t p;
    int32_t nc, nf, i, n;

    nc = s5_coarse_angles(ang, 8);
    for (i = 0; i < nc; i++) { c[i] = model_trip(m, ang[i], S5_SRC_COARSE); }
    s5_trips_to_samples(c, nc, cs);
    if (!s5_build_profile(cs, nc, &p)) { return s5_merge_trips(c, nc, NULL, 0, out, max); }
    nf = s5_fine_angles(&p, ang, 64);
    for (i = 0; i < nf; i++) { f[i] = model_trip(m, ang[i], S5_SRC_FINE); }
    n = s5_merge_trips(c, nc, f, nf, out, max);
    s5_trips_to_samples(out, n, ms);
    if (!s5_build_profile(ms, n, &p)) { return s5_merge_trips(c, nc, NULL, 0, out, max); }
    return n;
}

/* Mirror a scan left <-> right (angles negated, order reversed). */
static void mirror_trips(const s5_trip_t * in, int32_t n, s5_trip_t * out)
{
    int32_t i;
    for (i = 0; i < n; i++) { out[i] = in[n - 1 - i]; out[i].deg = (int8_t)(-in[n - 1 - i].deg); }
}
#endif
