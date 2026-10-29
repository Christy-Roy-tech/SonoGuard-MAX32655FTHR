/*
 * sg_detector.h - SonoGuard hazard detector (pure C, no hardware calls)
 *
 * Kept separate from main.c so the same logic can be tested on a PC.
 * Time unit everywhere is one FRAME = 1024 samples at 48 kHz = 21.33 ms.
 * Frequency unit is one BIN = 48000 / 1024 = 46.875 Hz.
 *
 * ALL VALUES IN THE "PRESCRIBED" BLOCKS BELOW ARE STARTING GUESSES.
 * Replace them with numbers from your own recordings (calibration).
 */
#ifndef SG_DETECTOR_H
#define SG_DETECTOR_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* ---------- Audio frame (fixed by the hardware setup) ---------- */
#define SG_RATE      48000
#define SG_FFT_SIZE  1024
#define SG_BIN_HZ    (48000.0f / 1024.0f) /* 46.875 Hz per bin */

/* ---------- PRESCRIBED: search band ---------- */
#define SG_BAND_MIN_HZ 500
#define SG_BAND_MAX_HZ 4000
#define SG_BIN_MIN     18 /* 843 Hz  */
#define SG_BIN_MAX     85 /* 3984 Hz */

/* ---------- PRESCRIBED: is this frame a clean tone? ---------- */
#define SG_MIN_RMS       60.0f /* absolute loudness gate (16-bit sample units) */
#define SG_NOISE_MARGIN  3.2f  /* band energy must beat noise floor by this factor (6 dB) */
#define SG_TONAL_RATIO   0.50f /* energy in peak +/-1 bin divided by band energy */
#define SG_LEARN_FRAMES  94    /* about 2 s of quiet at start / after SW2 to learn the floor */
#define SG_HANG_FRAMES   3     /* dropout tolerance inside a pulse (64 ms) */

/* ---------- PRESCRIBED: pulsed alarms (reversing beeper, fire alarm) ---------- */
#define SG_PULSE_MIN_FRAMES 6    /* 128 ms  */
#define SG_PULSE_MAX_FRAMES 40   /* 853 ms: a longer tone is a whistle or a machine hum */
#define SG_GAP_MIN_FRAMES   5    /* 107 ms  */
#define SG_GAP_MAX_FRAMES   40   /* 853 ms  */
#define SG_PULSES_REQUIRED  2    /* pulses in a row, same pitch */
#define SG_FREQ_TOL_BINS    2    /* +/- 94 Hz between pulses */

/* ---------- PRESCRIBED: sweeping sirens (wail / yelp) ---------- */
#define SG_SIREN_MIN_FRAMES   30 /* 640 ms of continuous tone */
#define SG_SIREN_SPREAD_BINS  8  /* peak must move at least 375 Hz */
#define SG_SIREN_STEP_MAX     4  /* a bigger step between frames counts as a jump */
#define SG_SIREN_MAX_JUMPS    0  /* wail and yelp glide smoothly; harmonic flips do not */

/* ---------- PRESCRIBED: after an alert ---------- */
#define SG_COOLDOWN_FRAMES 94 /* about 2 s between reports */

typedef enum {
    SG_ALERT_NONE = 0,
    SG_ALERT_PULSED = 1,
    SG_ALERT_SIREN = 2
} sg_alert_t;

typedef struct {
    /* current run of tonal frames */
    bool active, counted, too_long, siren_fired;
    int len, quiet, first_bin, min_bin, max_bin, prev_bin, jumps;
    /* chain of pulses */
    int chain, chain_bin, gap;
} sg_det_t;

static inline void sg_det_reset(sg_det_t *d)
{
    memset(d, 0, sizeof(*d));
}

/*
 * Call once per frame.
 *   tonal      : this frame is loud enough and has a clean peak
 *   bin        : the peak bin of this frame
 *   alert_bin  : (out) bin to report when an alert is returned
 *   detail     : (out) pulses counted (PULSED) or sweep width in Hz (SIREN)
 */
static inline sg_alert_t sg_det_step(sg_det_t *d, bool tonal, int bin, int *alert_bin, int *detail)
{
    sg_alert_t alert = SG_ALERT_NONE;

    if (!d->active) {
        if (d->chain > 0 && ++d->gap > SG_GAP_MAX_FRAMES) {
            d->chain = 0;
        }
        if (tonal) {
            if (d->chain > 0) {
                bool gap_ok = (d->gap >= SG_GAP_MIN_FRAMES);
                bool freq_ok = (abs(bin - d->chain_bin) <= SG_FREQ_TOL_BINS);
                if (!gap_ok || !freq_ok) {
                    d->chain = 0;
                }
            }
            d->active = true;
            d->counted = false;
            d->too_long = false;
            d->siren_fired = false;
            d->len = 1;
            d->quiet = 0;
            d->first_bin = d->min_bin = d->max_bin = d->prev_bin = bin;
            d->jumps = 0;
        }
        return alert;
    }

    /* inside a run */
    if (tonal) {
        d->len += 1 + d->quiet; /* dropout frames inside the run count as tone */
        d->quiet = 0;
        if (bin < d->min_bin) d->min_bin = bin;
        if (bin > d->max_bin) d->max_bin = bin;
        if (abs(bin - d->prev_bin) > SG_SIREN_STEP_MAX) d->jumps++;
        d->prev_bin = bin;
    } else if (++d->quiet > SG_HANG_FRAMES) {
        /* run is over */
        if (d->counted) {
            d->gap = d->quiet;
        } else {
            d->chain = 0;
        }
        d->active = false;
        return alert;
    }

    /* pulse logic */
    if (!d->too_long && d->len > SG_PULSE_MAX_FRAMES) {
        d->too_long = true;
        d->counted = false;
        d->chain = 0;
    }
    if (!d->counted && !d->too_long && d->len >= SG_PULSE_MIN_FRAMES) {
        d->counted = true;
        d->chain++;
        d->chain_bin = d->first_bin;
        if (d->chain >= SG_PULSES_REQUIRED) {
            alert = SG_ALERT_PULSED;
            *alert_bin = d->chain_bin;
            *detail = d->chain;
        }
    }

    /* siren logic: long, smooth, wide sweep */
    if (!d->siren_fired && d->len >= SG_SIREN_MIN_FRAMES &&
        (d->max_bin - d->min_bin) >= SG_SIREN_SPREAD_BINS && d->jumps <= SG_SIREN_MAX_JUMPS) {
        d->siren_fired = true;
        alert = SG_ALERT_SIREN;
        *alert_bin = (d->min_bin + d->max_bin) / 2;
        *detail = (int)(((d->max_bin - d->min_bin) * 375) / 8); /* bins -> Hz */
    }

    return alert;
}

#endif /* SG_DETECTOR_H */