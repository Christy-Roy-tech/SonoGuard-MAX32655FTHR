/*
 * SonoGuard - live acoustic hazard listener for the MAX32655FTHR
 *
 * Pipeline: MAX9867 codec -> I2S -> DMA ping-pong (2 x 1024 samples, 48 kHz)
 *           -> Hann window -> 1024-point real FFT (CMSIS-DSP)
 *           -> tone check against a learned noise floor
 *           -> pulse / sweep logic (sg_detector.h) -> alert
 *
 * Serial output (115200 baud, one text line each, all numbers are integers):
 *   I:SG1,rate,fft,spec_points,wave_blocks,bin_min,bin_max,db_fullscale,wave_div
 *   S:<48 hex pairs>        spectrum, 93.75 Hz per point, value*140/255 = dB
 *   W:<32 pairs of min,max> waveform envelope, 3.33 ms per pair, (value-128)*wave_div
 *   T:rms,peak_hz,tone_pct,noise_db,band_db,state,pulses,overruns,ms
 *   A:TYPE,freq_hz,tone_pct,detail,ms     alert event
 * state: 0 learning noise floor, 1 listening, 2 tone present, 3 pulse chain running
 *
 * Buttons: SW2 starts the detector, and pressed again re-learns the noise floor.
 *
 * Needs in project.mk:  LIB_CMSIS_DSP = 1
 *                       PROJ_CFLAGS += -DARM_MATH_CM4
 * Needs the patched max9867.c (2 ms pause after each I2C transfer) in this folder.
 * Copy sg_detector.h into this folder too.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "board.h"
#include "dma.h"
#include "gpio.h"
#include "i2s.h"
#include "icc.h"
#include "led.h"
#include "pb.h"
#include "max9867.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "nvic_table.h"
#include "arm_math.h"

#include "sg_detector.h"

/***** Hardware *****/
#define I2C_MASTER MXC_I2C1 /* FTHR_Apps_P1: SCL P0.16, SDA P0.17 */
#define SMPL_RATE  SG_RATE
#define FFT_SIZE   SG_FFT_SIZE
#define BUF_SIZE   FFT_SIZE
#define BIN_HZ     SG_BIN_HZ

/***** Serial report layout (one report cycle = 5 frames = 107 ms) *****/
#define REPORT_SLOTS       5
#define SPEC_POINTS        48  /* 2 bins per point -> 0 to 4500 Hz */
#define WAVE_BLOCKS        32  /* 5 frames = 5120 samples / 160 per block */
#define WAVE_BLOCK_SAMPLES 160

/* PRESCRIBED display scaling: change after calibration */
#define SPEC_DB_FULLSCALE 140.0f
#define WAVE_DIV          16 /* waveform byte = sample / 16, clips at +/-2032 */

/* PRESCRIBED: noise floor tracking */
#define NF_RISE 0.005f /* slow rise so a long alarm cannot become the "floor" */
#define NF_FALL 0.100f /* quick fall when the room gets quieter */

/* PRESCRIBED: red LED time after an alert, and optional listening mute */
#define ALERT_LED_FRAMES 70
#define MUTE_FRAMES      0 /* set about 40 when the headphone alert tone is added */

#define BUF_SEL(CUR, B1, B2) ((CUR == B1) ? B2 : B1)

/***** Global data *****/
mxc_i2s_req_t i2s_req;
volatile uint8_t i2s_done = 0;
int16_t buf0[BUF_SIZE];
int16_t buf1[BUF_SIZE];

arm_rfft_fast_instance_f32 fft_handler;
float32_t window[FFT_SIZE];
float32_t audio[FFT_SIZE];
float32_t fft_out[FFT_SIZE];
float32_t mags[FFT_SIZE / 2];

static int wave_min[WAVE_BLOCKS];
static int wave_max[WAVE_BLOCKS];

/*****************************************************************/
void i2s_dma_cb(int handle, int error)
{
    i2s_done = 1;
}

void DMA_IRQHandler(void)
{
    MXC_DMA_Handler();
}

int i2s_init(void)
{
    i2s_req.wordSize = MXC_I2S_WSIZE_HALFWORD;
    i2s_req.sampleSize = MXC_I2S_SAMPLESIZE_SIXTEEN;
    i2s_req.bitsWord = 16;
    i2s_req.adjust = MXC_I2S_ADJUST_LEFT;
    i2s_req.justify = MXC_I2S_LSB_JUSTIFY;
    i2s_req.wsPolarity = MXC_I2S_POL_NORMAL;
    i2s_req.channelMode = MXC_I2S_EXTERNAL_SCK_EXTERNAL_WS;
    i2s_req.stereoMode = MXC_I2S_MONO_RIGHT_CH;
    i2s_req.bitOrder = MXC_I2S_LSB_FIRST; /* kept as in the kit example and your working capture */
    i2s_req.clkdiv = MXC_I2S_CalculateClockDiv(SMPL_RATE, i2s_req.wordSize) + 1;
    i2s_req.rawData = NULL;
    i2s_req.txData = NULL;
    i2s_req.rxData = buf0;
    i2s_req.length = sizeof(buf0);

    int err = MXC_I2S_Init(&i2s_req);
    if (err != E_NO_ERROR) {
        return err;
    }

    MXC_I2S_RegisterDMACallback(i2s_dma_cb);
    MXC_NVIC_SetVector(DMA0_IRQn, DMA_IRQHandler);
    return err;
}

/*****************************************************************/
/* Serial helpers: one short line per frame at most, so the UART never
 * holds the loop for longer than one frame (21 ms; 115200 baud = 11.5 bytes/ms). */
static const char HEXD[] = "0123456789ABCDEF";

static void put_hex(char *p, uint8_t v)
{
    p[0] = HEXD[v >> 4];
    p[1] = HEXD[v & 15];
}

static uint8_t mag_to_u8(float m)
{
    float db = 20.0f * log10f(m + 1.0f);
    int v = (int)(db * (255.0f / SPEC_DB_FULLSCALE));
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    return (uint8_t)v;
}

static void send_spectrum(void)
{
    char line[2 + 2 * SPEC_POINTS + 2];
    char *p = line;
    *p++ = 'S';
    *p++ = ':';
    for (int k = 0; k < SPEC_POINTS; k++) {
        float m = mags[2 * k];
        if (mags[2 * k + 1] > m) m = mags[2 * k + 1];
        put_hex(p, mag_to_u8(m));
        p += 2;
    }
    *p++ = '\n';
    *p = 0;
    printf("%s", line);
}

static uint8_t wave_to_u8(int v)
{
    v /= WAVE_DIV;
    if (v > 127) v = 127;
    if (v < -128) v = -128;
    return (uint8_t)(v + 128);
}

static void send_wave(void)
{
    char line[2 + 4 * WAVE_BLOCKS + 2];
    char *p = line;
    *p++ = 'W';
    *p++ = ':';
    for (int b = 0; b < WAVE_BLOCKS; b++) {
        put_hex(p, wave_to_u8(wave_min[b]));
        put_hex(p + 2, wave_to_u8(wave_max[b]));
        p += 4;
    }
    *p++ = '\n';
    *p = 0;
    printf("%s", line);
}

/*****************************************************************/
/* Hook for the headphone alert tone. Not implemented yet: build and test the
 * playback path in a separate project first (earbuds on the table, not in ears). */
static void on_alert(sg_alert_t type)
{
    (void)type;
}

/*****************************************************************/
int main(void)
{
    MXC_ICC_Enable(MXC_ICC0);
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    printf("\n=== SonoGuard listener (FFT %d, %d Hz) ===\n", FFT_SIZE, SMPL_RATE);

    if (i2s_init() != E_NO_ERROR) {
        printf("Error: I2S init failed\n");
        while (1) {}
    }
    int cerr = max9867_init(I2C_MASTER, 12288000, SMPL_RATE);
    if (cerr != E_NO_ERROR) {
        printf("Error: codec init failed (%d)\n", cerr);
        while (1) {}
    }

    arm_rfft_fast_init_f32(&fft_handler, FFT_SIZE);
    for (int i = 0; i < FFT_SIZE; i++) {
        window[i] = 0.5f - 0.5f * cosf(2.0f * 3.14159265f * (float)i / (float)(FFT_SIZE - 1));
    }

    printf("Press SW2 to start.\n");
    LED_On(1);
    while (!PB_Get(0)) {}
    MXC_Delay(MXC_DELAY_MSEC(400));
    while (PB_Get(0)) {}
    printf("Learning the noise floor for 2 s. Keep the room quiet.\n");

    sg_det_t det;
    sg_det_reset(&det);

    void *i2s_buf = buf0;
    i2s_done = 0;
    MXC_DMA_ReleaseChannel(0);
    MXC_I2S_RXDMAConfig(i2s_buf, sizeof(buf0));

    uint32_t frame = 0;
    uint32_t overruns = 0;
    uint32_t alerts_total = 0;
    int learn_left = SG_LEARN_FRAMES;
    float nf_sum = 0.0f;
    float noise_floor = 1.0f;
    int cooldown = 0;
    int led_timer = 0;
    int mute = 0;
    bool btn_prev = false;

    while (1) {
        while (!i2s_done) {}

        int16_t *proc = (int16_t *)i2s_buf;
        i2s_buf = BUF_SEL(i2s_buf, buf0, buf1);
        i2s_done = 0;
        MXC_DMA_ReleaseChannel(0);
        MXC_I2S_RXDMAConfig(i2s_buf, sizeof(buf0)); /* restart capture first */

        int slot = (int)(frame % REPORT_SLOTS);
        uint32_t ms = (frame * 64u) / 3u; /* 21.33 ms per frame */
        frame++;

        /* SW2 again: learn the noise floor again */
        bool btn = PB_Get(0) ? true : false;
        if (btn && !btn_prev) {
            learn_left = SG_LEARN_FRAMES;
            nf_sum = 0.0f;
            sg_det_reset(&det);
            printf("Re-learning the noise floor.\n");
        }
        btn_prev = btn;

        /* remove DC, level, window, waveform envelope */
        float32_t sum = 0.0f;
        for (int i = 0; i < FFT_SIZE; i++) sum += (float32_t)proc[i];
        float32_t mean = sum / (float32_t)FFT_SIZE;

        if (slot == 0) {
            for (int b = 0; b < WAVE_BLOCKS; b++) {
                wave_min[b] = 32767;
                wave_max[b] = -32768;
            }
        }
        float32_t sq = 0.0f;
        int wbase = slot * FFT_SIZE;
        for (int i = 0; i < FFT_SIZE; i++) {
            float32_t v = (float32_t)proc[i] - mean;
            sq += v * v;
            audio[i] = v * window[i];
            int b = (wbase + i) / WAVE_BLOCK_SAMPLES;
            int s = (int)v;
            if (s < wave_min[b]) wave_min[b] = s;
            if (s > wave_max[b]) wave_max[b] = s;
        }
        float32_t rms = sqrtf(sq / (float32_t)FFT_SIZE);

        arm_rfft_fast_f32(&fft_handler, audio, fft_out, 0);
        arm_cmplx_mag_f32(fft_out, mags, FFT_SIZE / 2);
        mags[0] = 0.0f; /* bin 0 holds DC and Nyquist packed together */

        /* band energy and peak */
        float32_t band_total = 0.0f, peak_e = 0.0f;
        int peak_bin = SG_BIN_MIN;
        for (int i = SG_BIN_MIN; i <= SG_BIN_MAX; i++) {
            float32_t e = mags[i] * mags[i];
            band_total += e;
            if (e > peak_e) {
                peak_e = e;
                peak_bin = i;
            }
        }
        float32_t peak_zone = 0.0f;
        for (int k = peak_bin - 1; k <= peak_bin + 1; k++) {
            if (k >= SG_BIN_MIN && k <= SG_BIN_MAX) peak_zone += mags[k] * mags[k];
        }
        float32_t ratio = (band_total > 0.0f) ? (peak_zone / band_total) : 0.0f;

        /* learning phase, then tone decision */
        bool tonal = false;
        if (learn_left > 0) {
            nf_sum += band_total;
            if (--learn_left == 0) {
                noise_floor = nf_sum / (float32_t)SG_LEARN_FRAMES;
                if (noise_floor < 1.0f) noise_floor = 1.0f;
                printf("Noise floor learned. Listening.\n");
            }
        } else {
            bool loud = (rms > SG_MIN_RMS) && (band_total > noise_floor * SG_NOISE_MARGIN);
            tonal = loud && (ratio >= SG_TONAL_RATIO);
            if (mute > 0) {
                mute--;
                tonal = false;
            }
            /* track the floor only while no clean tone is present */
            if (!tonal && !det.active) {
                float32_t k = (band_total < noise_floor) ? NF_FALL : NF_RISE;
                noise_floor += (band_total - noise_floor) * k;
                if (noise_floor < 1.0f) noise_floor = 1.0f;
            }
        }

        /* detector */
        int alert_bin = 0, detail = 0;
        sg_alert_t alert = SG_ALERT_NONE;
        if (learn_left == 0) {
            alert = sg_det_step(&det, tonal, peak_bin, &alert_bin, &detail);
        }
        if (cooldown > 0) cooldown--;
        if (alert != SG_ALERT_NONE && cooldown == 0) {
            int freq = (int)((float32_t)alert_bin * BIN_HZ);
            printf("A:%s,%d,%d,%d,%lu\n", (alert == SG_ALERT_SIREN) ? "SIREN" : "PULSED", freq,
                   (int)(ratio * 100.0f), detail, (unsigned long)ms);
            LED_On(0);
            led_timer = ALERT_LED_FRAMES;
            cooldown = SG_COOLDOWN_FRAMES;
            mute = MUTE_FRAMES;
            alerts_total++;
            on_alert(alert);
        }
        if (led_timer > 0 && --led_timer == 0) LED_Off(0);

        /* late check: if the other buffer is already full, this frame took too long */
        if (i2s_done) overruns++;

        /* reports: at most one line per frame */
        if (slot == 0) {
            send_spectrum();
        } else if (slot == 1) {
            int state = (learn_left > 0) ? 0 : (det.active ? 2 : ((det.chain > 0) ? 3 : 1));
            printf("T:%d,%d,%d,%d,%d,%d,%d,%lu,%lu\n", (int)rms, (int)((float32_t)peak_bin * BIN_HZ),
                   (int)(ratio * 100.0f), (int)(10.0f * log10f(noise_floor + 1.0f)),
                   (int)(10.0f * log10f(band_total + 1.0f)), state, det.chain,
                   (unsigned long)overruns, (unsigned long)ms);
        } else if (slot == 2) {
            if (((frame / REPORT_SLOTS) % 20) == 0) {
                printf("I:SG1,%d,%d,%d,%d,%d,%d,%d,%d\n", SMPL_RATE, FFT_SIZE, SPEC_POINTS,
                       WAVE_BLOCKS, SG_BIN_MIN, SG_BIN_MAX, (int)SPEC_DB_FULLSCALE, WAVE_DIV);
            }
        } else if (slot == 4) {
            send_wave();
        }
    }
}