#include <stdio.h>
#include "sg_detector.h"

static int fails = 0;
static sg_det_t d;
static int alerts_p, alerts_s;

static void frame(bool tonal, int bin)
{
    int ab = 0, dt = 0;
    sg_alert_t a = sg_det_step(&d, tonal, bin, &ab, &dt);
    if (a == SG_ALERT_PULSED) alerts_p++;
    if (a == SG_ALERT_SIREN) alerts_s++;
}
static void run(int n, bool tonal, int bin) { for (int i = 0; i < n; i++) frame(tonal, bin); }
static void begin(void) { sg_det_reset(&d); alerts_p = alerts_s = 0; }
static void check(const char *name, int gotp, int gots, int wantp_min, int wants_min)
{
    bool ok = (wantp_min ? gotp >= wantp_min : gotp == 0) && (wants_min ? gots >= wants_min : gots == 0);
    printf("%-46s pulsed=%d siren=%d  %s\n", name, gotp, gots, ok ? "PASS" : "FAIL");
    if (!ok) fails++;
}

int main(void)
{
    /* 1 beeper: 400 ms on (19 frames), 400 ms off, x4 at bin 21 (984 Hz) */
    begin();
    run(10, false, 0);
    for (int k = 0; k < 4; k++) { run(19, true, 21); run(19, false, 0); }
    check("reversing beeper, 4 pulses", alerts_p, alerts_s, 1, 0);

    /* 2 only two pulses */
    begin();
    run(10, false, 0);
    for (int k = 0; k < 2; k++) { run(19, true, 21); run(19, false, 0); }
    run(60, false, 0);
    check("two pulses only", alerts_p, alerts_s, 0, 0);

    /* 3 continuous whistle 4 s (187 frames) steady pitch */
    begin();
    run(10, false, 0);
    run(187, true, 40);
    run(30, false, 0);
    check("steady whistle 4 s", alerts_p, alerts_s, 0, 0);

    /* 4 wail siren: 60 frames gliding bin 13 -> 32 */
    begin();
    run(10, false, 0);
    for (int i = 0; i < 60; i++) frame(true, 13 + (i * 19) / 59);
    run(30, false, 0);
    check("wail siren sweep", alerts_p, alerts_s, 0, 1);

    /* 5 pulses at different pitches */
    begin();
    run(10, false, 0);
    run(19, true, 21); run(19, false, 0);
    run(19, true, 40); run(19, false, 0);
    run(19, true, 60); run(19, false, 0);
    run(60, false, 0);
    check("three pulses, different pitch each", alerts_p, alerts_s, 0, 0);

    /* 6 beeper with one-frame dropouts inside each pulse */
    begin();
    run(10, false, 0);
    for (int k = 0; k < 4; k++) { run(8, true, 21); frame(false, 0); run(10, true, 21); run(19, false, 0); }
    check("beeper with 1-frame dropouts", alerts_p, alerts_s, 1, 0);

    /* 7 peak flipping between fundamental and harmonic (long tone) */
    begin();
    run(10, false, 0);
    for (int i = 0; i < 80; i++) frame(true, (i % 7 < 4) ? 20 : 40);
    run(30, false, 0);
    check("harmonic flipping long tone", alerts_p, alerts_s, 0, 0);

    /* 8 very short clicks */
    begin();
    for (int k = 0; k < 6; k++) { run(2, true, 21); run(15, false, 0); }
    check("short clicks", alerts_p, alerts_s, 0, 0);

    /* 9 gap too long between pulses (2 s) */
    begin();
    for (int k = 0; k < 4; k++) { run(19, true, 21); run(94, false, 0); }
    check("pulses 2 s apart", alerts_p, alerts_s, 0, 0);

    printf(fails ? "%d FAILED\n" : "all passed\n", fails);
    return fails != 0;
}