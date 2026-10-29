#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
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

#define I2C_MASTER MXC_I2C1
#define BUF_SIZE 512
#define SMPL_RATE 48000
#define FFT_SIZE 512
#define BIN_HZ_VALUE 93.75f

/* Broad Industrial Hazard Spectrum: 800 Hz to 3200 Hz */
#define HAZARD_BIN_MIN 8
#define HAZARD_BIN_MAX 34

/* Minimum spectral energy concentration to consider a frame tonal */
#define TONAL_RATIO_THRESHOLD 0.55f
#define MIN_RMS_ENERGY 25000000.0f

/* Cadence timing constraints (1 frame = 10.67 ms) */
#define MIN_PULSE_FRAMES 15    /* ~160 ms */
#define MAX_PULSE_FRAMES 55    /* ~580 ms */
#define MIN_GAP_FRAMES   15    /* ~160 ms */
#define MAX_GAP_FRAMES   60    /* ~640 ms */
#define MAX_TONE_HOLD    95    /* > 1000 ms continuous tone = Whistle / Rejection */

#define BUF_SEL(CUR, B1, B2) ((CUR == B1) ? B2 : B1)

mxc_i2s_req_t i2s_req;
volatile uint8_t i2s_done = 0;
int16_t buf0[BUF_SIZE];
int16_t buf1[BUF_SIZE];

arm_rfft_fast_instance_f32 fft_handler;
float32_t audio_float_buffer[FFT_SIZE];
float32_t fft_output[FFT_SIZE];
float32_t magnitudes[FFT_SIZE / 2];

typedef enum {
    CADENCE_IDLE = 0,
    CADENCE_FIRST_PULSE,
    CADENCE_GAP,
    CADENCE_SECOND_PULSE
} cadence_state_t;

void i2s_dma_cb(int handle, int error) { i2s_done = 1; }
void DMA_IRQHandler(void) { MXC_DMA_Handler(); }

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
    i2s_req.bitOrder = MXC_I2S_MSB_FIRST;
    i2s_req.clkdiv = MXC_I2S_CalculateClockDiv(SMPL_RATE, i2s_req.wordSize) + 1;
    i2s_req.rawData = NULL;
    i2s_req.txData = NULL;
    i2s_req.rxData = buf0;
    i2s_req.length = sizeof(buf0);

    int err = MXC_I2S_Init(&i2s_req);
    if (err != E_NO_ERROR) return err;

    MXC_I2S_RegisterDMACallback(i2s_dma_cb);
    MXC_NVIC_SetVector(DMA0_IRQn, DMA_IRQHandler);
    return err;
}

void live_detector(void)
{
    printf("Initializing DSP...\n");
    arm_rfft_fast_init_f32(&fft_handler, FFT_SIZE);

    void *i2s_buf = buf0;
    int16_t *process_buf = buf0;
    i2s_done = 0;
    MXC_DMA_ReleaseChannel(0);
    MXC_I2S_RXDMAConfig(i2s_buf, sizeof(buf0));

    printf("SonoGuard Running: Multi-Band Cadence Recognition Active.\n");

    cadence_state_t state = CADENCE_IDLE;
    int pulse_counter = 0;
    int gap_counter = 0;
    int primary_hazard_freq = 0;
    int cooldown_timer = 0;

    while (1) {
        while (!i2s_done) {}

        process_buf = (int16_t *)i2s_buf;
        i2s_buf = BUF_SEL(i2s_buf, buf0, buf1);
        i2s_done = 0;
        MXC_DMA_ReleaseChannel(0);
        MXC_I2S_RXDMAConfig(i2s_buf, sizeof(buf0));

        /* Convert to float and remove DC */
        float32_t sum = 0.0f;
        for (int i = 0; i < FFT_SIZE; i++) sum += (float32_t)process_buf[i];
        float32_t mean = sum / (float32_t)FFT_SIZE;

        for (int i = 0; i < FFT_SIZE; i++) {
            audio_float_buffer[i] = (float32_t)process_buf[i] - mean;
        }

        arm_rfft_fast_f32(&fft_handler, audio_float_buffer, fft_output, 0);
        arm_cmplx_mag_f32(fft_output, magnitudes, FFT_SIZE / 2);

        float32_t total_energy = 0.0f;
        float32_t peak_energy = 0.0f;
        int peak_bin = HAZARD_BIN_MIN;

        for (int i = 3; i < (FFT_SIZE / 2); i++) {
            float32_t e = magnitudes[i] * magnitudes[i];
            total_energy += e;
            if (i >= HAZARD_BIN_MIN && i <= HAZARD_BIN_MAX) {
                if (e > peak_energy) {
                    peak_energy = e;
                    peak_bin = i;
                }
            }
        }

        float32_t peak_band = 0.0f;
        for (int k = peak_bin - 1; k <= peak_bin + 1; k++) {
            if (k >= HAZARD_BIN_MIN && k <= HAZARD_BIN_MAX) {
                peak_band += magnitudes[k] * magnitudes[k];
            }
        }

        float32_t ratio = (total_energy > 0.0f) ? (peak_band / total_energy) : 0.0f;
        bool is_tonal = (total_energy > MIN_RMS_ENERGY) && (ratio >= TONAL_RATIO_THRESHOLD);
        int current_freq = (int)((float32_t)peak_bin * BIN_HZ_VALUE);

        if (cooldown_timer > 0) cooldown_timer--;

        /* Cadence Verification Engine */
        switch (state) {
        case CADENCE_IDLE:
            if (is_tonal) {
                state = CADENCE_FIRST_PULSE;
                pulse_counter = 1;
                primary_hazard_freq = current_freq;
            }
            break;

        case CADENCE_FIRST_PULSE:
            if (is_tonal) {
                pulse_counter++;
                /* Reject continuous tones (whistles, constant machinery noise) */
                if (pulse_counter > MAX_TONE_HOLD) {
                    state = CADENCE_IDLE;
                    pulse_counter = 0;
                }
            } else {
                if (pulse_counter >= MIN_PULSE_FRAMES && pulse_counter <= MAX_PULSE_FRAMES) {
                    state = CADENCE_GAP;
                    gap_counter = 1;
                } else {
                    state = CADENCE_IDLE;
                }
            }
            break;

        case CADENCE_GAP:
            if (!is_tonal) {
                gap_counter++;
                if (gap_counter > MAX_GAP_FRAMES) {
                    state = CADENCE_IDLE;
                }
            } else {
                if (gap_counter >= MIN_GAP_FRAMES && gap_counter <= MAX_GAP_FRAMES) {
                    state = CADENCE_SECOND_PULSE;
                    pulse_counter = 1;
                } else {
                    state = CADENCE_IDLE;
                }
            }
            break;

        case CADENCE_SECOND_PULSE:
            if (is_tonal) {
                pulse_counter++;
                if (pulse_counter >= MIN_PULSE_FRAMES) {
                    /* Hazard Confirmed by Cadence Pattern */
                    LED_On(0);
                    if (cooldown_timer == 0) {
                        int ratio_pct = (int)(ratio * 100.0f);
                        printf("ALERT:TYPE=BEEPER,FREQ=%d,RATIO=%d,STATUS=CONFIRMED\n",
                               primary_hazard_freq, ratio_pct);
                        cooldown_timer = 120; /* ~1.2 seconds cooldown */
                    }
                }
            } else {
                state = CADENCE_IDLE;
                LED_Off(0);
            }
            break;
        }

        /* Periodic raw telemetry output (every ~200 ms) for smooth UI plotting */
        static int tele_count = 0;
        if (++tele_count >= 19) {
            tele_count = 0;
            int ratio_pct = (int)(ratio * 100.0f);
            printf("DATA:%d,%d,%d\n", is_tonal ? current_freq : 0, ratio_pct, (state == CADENCE_SECOND_PULSE) ? 1 : 0);
        }
    }
}