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
#define BIN_HZ (SMPL_RATE / (float)FFT_SIZE) /* 93.75 Hz */
#define FIRST_BIN 3 /* skip DC/low rumble (bin 0 is also corrupted by packing) */
#define FRAMES_PER_REPORT 94 /* about 1 second */
#define FIRST_BIN 5 
/* Calibration gates. Adjust after the quiet-room test. */
#define MIN_RMS 150.0f
#define TONAL_FRAME_SHARE 0.60f

#define BUF_SEL(CUR, B1, B2) ((CUR == B1) ? B2 : B1)

mxc_i2s_req_t i2s_req;
volatile uint8_t i2s_done = 0;
int16_t buf0[BUF_SIZE];
int16_t buf1[BUF_SIZE];

arm_rfft_fast_instance_f32 fft_handler;
float32_t window[FFT_SIZE];
float32_t audio[FFT_SIZE];
float32_t fft_out[FFT_SIZE];
float32_t mags[FFT_SIZE / 2];

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
    i2s_req.bitOrder = MXC_I2S_LSB_FIRST;
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

int main(void)
{
    MXC_ICC_Enable(MXC_ICC0);
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    printf("\n=== SonoGuard listener ===\n");

    if (i2s_init() != E_NO_ERROR) { printf("I2S init failed\n"); while (1) {} }
    if (max9867_init(I2C_MASTER, 12288000, SMPL_RATE) != E_NO_ERROR) {
        printf("Codec init failed\n"); while (1) {}
    }

    arm_rfft_fast_init_f32(&fft_handler, FFT_SIZE);
    for (int i = 0; i < FFT_SIZE; i++) {
        window[i] = 0.5f - 0.5f * cosf(2.0f * 3.14159265f * i / (FFT_SIZE - 1));
    }

    printf("Bin width %d Hz. Columns: level(max rms) | peak Hz | tonal %% | tonal frames/loud frames\n",
           (int)BIN_HZ);

    void *i2s_buf = buf0;
    i2s_done = 0;
    MXC_DMA_ReleaseChannel(0);
    MXC_I2S_RXDMAConfig(i2s_buf, sizeof(buf0));
    LED_On(1); /* green = listening */

    int frame_count = 0, loud_frames = 0, tonal_frames = 0;
    float32_t max_rms = 0.0f;
    int max_rms_freq = 0, max_rms_tonal = 0;

    while (1) {
        while (!i2s_done) {}
        int16_t *proc = (int16_t *)i2s_buf;
        i2s_buf = BUF_SEL(i2s_buf, buf0, buf1);
        i2s_done = 0;
        MXC_DMA_ReleaseChannel(0);
        MXC_I2S_RXDMAConfig(i2s_buf, sizeof(buf0)); /* restart capture first */

        /* Remove DC, compute RMS, apply Hann window */
        float32_t sum = 0.0f;
        for (int i = 0; i < FFT_SIZE; i++) sum += proc[i];
        float32_t mean = sum / FFT_SIZE;
        float32_t sq = 0.0f;
        for (int i = 0; i < FFT_SIZE; i++) {
            float32_t v = proc[i] - mean;
            sq += v * v;
            audio[i] = v * window[i];
        }
        float32_t rms = sqrtf(sq / FFT_SIZE);

        arm_rfft_fast_f32(&fft_handler, audio, fft_out, 0);
        arm_cmplx_mag_f32(fft_out, mags, FFT_SIZE / 2);

        /* Total energy and peak bin */
        float32_t total = 0.0f, peak_e = 0.0f;
        int peak_bin = FIRST_BIN;
        for (int i = FIRST_BIN; i < FFT_SIZE / 2; i++) {
            float32_t e = mags[i] * mags[i];
            total += e;
            if (e > peak_e) { peak_e = e; peak_bin = i; }
        }

        float32_t peak_zone = 0.0f;
        for (int k = peak_bin - 1; k <= peak_bin + 1; k++) {
            if (k >= FIRST_BIN && k < FFT_SIZE / 2) peak_zone += mags[k] * mags[k];
        }
        float32_t tonal = (total > 0.0f) ? (peak_zone / total) : 0.0f;

        bool loud = (rms > MIN_RMS);
        bool is_tonal = loud && (tonal > TONAL_FRAME_SHARE);
        if (loud) loud_frames++;
        if (is_tonal) tonal_frames++;
        if (is_tonal) LED_On(0); else LED_Off(0);

        if (rms > max_rms) {
            max_rms = rms;
            max_rms_freq = (int)(peak_bin * BIN_HZ);
            max_rms_tonal = (int)(tonal * 100.0f);
        }

        if (++frame_count >= FRAMES_PER_REPORT) {
            printf("lvl=%d | peak=%d Hz | tonal=%d%% | %d/%d\n",
                   (int)max_rms, max_rms_freq, max_rms_tonal,
                   tonal_frames, loud_frames);
            frame_count = loud_frames = tonal_frames = 0;
            max_rms = 0.0f;
            max_rms_freq = max_rms_tonal = 0;
        }
    }
}