#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Maxim Hardware Headers */
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
#include "wut.h"
#include "rtc.h"
#include "trimsir_regs.h"

/* Cordio WSF and BLE Headers from BLE_fit */
#include "wsf_types.h"
#include "wsf_trace.h"
#include "wsf_bufio.h"
#include "wsf_msg.h"
#include "wsf_assert.h"
#include "wsf_buf.h"
#include "wsf_heap.h"
#include "wsf_cs.h"
#include "wsf_timer.h"
#include "wsf_os.h"
#include "sec_api.h"
#include "hci_handler.h"
#include "dm_handler.h"
#include "l2c_handler.h"
#include "att_handler.h"
#include "smp_handler.h"
#include "l2c_api.h"
#include "att_api.h"
#include "smp_api.h"
#include "app_api.h"
#include "hci_core.h"
#include "app_terminal.h"

#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
#include "ll_init_api.h"
#endif

#include "pal_bb.h"
#include "pal_cfg.h"
#include "fit_api.h"
#include "app_ui.h"

#include "sg_detector.h"

/* Defined in fit_main.c (part of this BLE_fit project) but not declared in any header */
void StackInitFit(void);

/* ---- Build switches. Start with both 0 (audio only). Turn ON one at a time. ---- */
#define SG_ENABLE_BLE   1 /* 1 = Bluetooth beacon to the ESP32 */
#define SG_ENABLE_CHIME 1 /* 1 = headphone chime (turns on the I2S transmit side) */

/*********************************************************************
 * Hardware & Audio Configurations
 *********************************************************************/
#define I2C_MASTER         MXC_I2C1
#define SMPL_RATE          SG_RATE
#define FFT_SIZE           SG_FFT_SIZE
#define BUF_SIZE           FFT_SIZE
#define BIN_HZ             SG_BIN_HZ

#define REPORT_SLOTS       5
#define SPEC_POINTS        48
#define WAVE_BLOCKS        32
#define WAVE_BLOCK_SAMPLES 160
#define SPEC_DB_FULLSCALE  140.0f
#define WAVE_DIV           16

#define NF_RISE            0.005f
#define NF_FALL            0.100f

#define ALERT_LED_FRAMES   70
#define MUTE_FRAMES        45  /* Suppress mic while alert chime plays */

#define BUF_SEL(CUR, B1, B2) ((CUR == B1) ? B2 : B1)

/* Headphone alert chime: 200 ms at 48 kHz = 9600 samples */
#define ALERT_TONE_SAMPLES 9600
static int16_t alert_tone_buf[ALERT_TONE_SAMPLES];

/*********************************************************************
 * BLE Stack Configuration (from BLE_fit)
 *********************************************************************/
#define PLATFORM_UART_TERMINAL_BUFFER_SIZE 2048U
#define DEFAULT_TX_POWER 0 /* dBm */

static wsfBufPoolDesc_t mainPoolDesc[] = { { 16, 8 }, { 32, 4 }, { 192, 8 }, { 256, 8 } };

#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
static LlRtCfg_t mainLlRtCfg;
#endif

volatile int wutTrimComplete;
static int ble_broadcast_timer = 0;
static volatile bool ble_ready = false; /* set once the stack has had time to finish its reset */

/* SonoGuard Peer Broadcast Packet (conforms to ESP32 scanner format) */
static uint8_t sonoGuard_adv_data[] = {
    /* 1. Flags (3 bytes) */
    0x02,
    DM_ADV_TYPE_FLAGS,
    DM_FLAG_LE_GENERAL_DISC | DM_FLAG_LE_BREDR_NOT_SUP,

    /* 2. Complete Local Name: "SonoGuard" (11 bytes) */
    0x0A,
    DM_ADV_TYPE_LOCAL_NAME,
    'S', 'o', 'n', 'o', 'G', 'u', 'a', 'r', 'd',

    /* 3. Manufacturer Specific Data: Hazard Signature (6 bytes) */
    0x05,
    DM_ADV_TYPE_MANUFACTURER,
    0x01,       /* Byte 16: Hazard Type (1 = Pulsed, 2 = Siren) */
    0x03, 0xE8, /* Bytes 17-18: Frequency (Big-Endian, e.g. 1000 Hz) */
    0x50        /* Byte 19: Tonal Ratio % (e.g. 80%) */
};

/*********************************************************************
 * Audio Subsystem Buffers
 *********************************************************************/
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

/*********************************************************************
 * Callbacks and Helper Functions
 *********************************************************************/
void i2s_dma_cb(int handle, int error) { i2s_done = 1; }
void DMA_IRQHandler(void) { MXC_DMA_Handler(); }

void wutTrimCb(int err)
{
    wutTrimComplete = 1;
}

#if SG_ENABLE_BLE
static void mainWsfInit(void)
{
#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
    const uint16_t maxRptBufSize = 12 + 2 + 255;
    const uint16_t aclBufSize = 12 + mainLlRtCfg.maxAclLen + 4 + BB_DATA_PDU_TAILROOM;

    mainPoolDesc[2].len = maxRptBufSize;
    mainPoolDesc[2].num = mainLlRtCfg.maxAdvReports;
    mainPoolDesc[3].len = aclBufSize;
    mainPoolDesc[3].num = mainLlRtCfg.numTxBufs + mainLlRtCfg.numRxBufs;
#endif

    const uint8_t numPools = sizeof(mainPoolDesc) / sizeof(mainPoolDesc[0]);
    uint16_t memUsed;

    WsfCsEnter();
    memUsed = WsfBufCalcSize(numPools, mainPoolDesc);
    WsfHeapAlloc(memUsed);
    WsfBufInit(numPools, mainPoolDesc);
    WsfCsExit();

    WsfOsInit();
    WsfTimerInit();
}
#endif

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
#if SG_ENABLE_CHIME
    i2s_req.txData = alert_tone_buf; /* Enables TX peripheral for headphone chime */
#else
    i2s_req.txData = NULL;
#endif
    i2s_req.rxData = buf0;
    i2s_req.length = sizeof(buf0);

    int err = MXC_I2S_Init(&i2s_req);
    if (err != E_NO_ERROR) return err;

    MXC_I2S_RegisterDMACallback(i2s_dma_cb);
    MXC_NVIC_SetVector(DMA0_IRQn, DMA_IRQHandler);
    return err;
}

/* Serial streaming helpers for the PySide6 dashboard */
static const char HEXD[] = "0123456789ABCDEF";
static void put_hex(char *p, uint8_t v) { p[0] = HEXD[v >> 4]; p[1] = HEXD[v & 15]; }

static uint8_t mag_to_u8(float m)
{
    float db = 20.0f * log10f(m + 1.0f);
    int v = (int)(db * (255.0f / SPEC_DB_FULLSCALE));
    if (v < 0) { v = 0; }
    if (v > 255) { v = 255; }
    return (uint8_t)v;
}

static void send_spectrum(void)
{
    char line[2 + 2 * SPEC_POINTS + 2];
    char *p = line;
    *p++ = 'S'; *p++ = ':';
    for (int k = 0; k < SPEC_POINTS; k++) {
        float m = mags[2 * k];
        if (mags[2 * k + 1] > m) m = mags[2 * k + 1];
        put_hex(p, mag_to_u8(m));
        p += 2;
    }
    *p++ = '\n'; *p = 0;
    printf("%s", line);
}

static uint8_t wave_to_u8(int v)
{
    v /= WAVE_DIV;
    if (v > 127) { v = 127; }
    if (v < -128) { v = -128; }
    return (uint8_t)(v + 128);
}

static void send_wave(void)
{
    char line[2 + 4 * WAVE_BLOCKS + 2];
    char *p = line;
    *p++ = 'W'; *p++ = ':';
    for (int b = 0; b < WAVE_BLOCKS; b++) {
        put_hex(p, wave_to_u8(wave_min[b]));
        put_hex(p + 2, wave_to_u8(wave_max[b]));
        p += 4;
    }
    *p++ = '\n'; *p = 0;
    printf("%s", line);
}

/*********************************************************************
 * Hazard Alert Handler: Audio Chime + BLE Relay Burst
 *********************************************************************/
static void on_alert(sg_alert_t type, int freq, int ratio_pct)
{
#if SG_ENABLE_CHIME
    /* 1. Play dual-tone audio chime through headphones */
    MXC_DMA_ReleaseChannel(1);
    MXC_I2S_TXDMAConfig(alert_tone_buf, sizeof(alert_tone_buf));
#endif
#if SG_ENABLE_BLE

    /* 2. Update BLE advertising packet for the ESP32 peer node.
     * Only after the stack has finished starting up: calling these too early crashed the board. */
    if (!ble_ready) {
        return;
    }
    sonoGuard_adv_data[16] = (type == SG_ALERT_SIREN) ? 0x02 : 0x01;
    sonoGuard_adv_data[17] = (uint8_t)((freq >> 8) & 0xFF);
    sonoGuard_adv_data[18] = (uint8_t)(freq & 0xFF);
    sonoGuard_adv_data[19] = (uint8_t)ratio_pct;

    /* Stop the stock "Fit" advertising, then send our beacon */
    AppAdvStop();
    AppSetAdvType(DM_ADV_NONCONN_UNDIRECT);
    AppAdvSetData(APP_ADV_DATA_DISCOVERABLE, sizeof(sonoGuard_adv_data), (uint8_t *)sonoGuard_adv_data);

    /* 3. Broadcast non-connectable alert beacon for about 3 seconds (advertising interval is 500 ms) */
    AppAdvStart(APP_MODE_DISCOVERABLE);
    ble_broadcast_timer = 140; /* 140 frames * 21.33 ms = ~3 seconds */
#endif
    (void)type; (void)freq; (void)ratio_pct;
}

/*********************************************************************
 * Main Entry Point
 *********************************************************************/
int main(void)
{
    MXC_ICC_Enable(MXC_ICC0);
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    printf("\n=== SonoGuard: Audio Listener + BLE Safety Relay ===\n");

#if SG_ENABLE_BLE
    /* -------------------------------------------------------------
     * Initialize Bluetooth Link Layer & 32 kHz Crystal (from BLE_fit)
     * ------------------------------------------------------------- */
#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
    static BbRtCfg_t mainBbRtCfg;
    PalBbLoadCfg((PalBbCfg_t *)&mainBbRtCfg);
    LlGetDefaultRunTimeCfg(&mainLlRtCfg);
#if (BT_VER >= LL_VER_BT_CORE_SPEC_5_0)
    mainLlRtCfg.btVer = LL_VER_BT_CORE_SPEC_5_0;
#endif
    PalCfgLoadData(PAL_CFG_ID_LL_PARAM, &mainLlRtCfg.maxAdvSets, sizeof(LlRtCfg_t) - 9);
#if (BT_VER >= LL_VER_BT_CORE_SPEC_5_0)
    PalCfgLoadData(PAL_CFG_ID_BLE_PHY, (uint8_t *)&mainLlRtCfg.phy2mSup, 4);
#endif
    mainBbRtCfg.clkPpm = 20;
    mainLlRtCfg.defTxPwrLvl = DEFAULT_TX_POWER;
#endif

    mainWsfInit();

#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
    uint32_t llmemUsed;
    WsfCsEnter();
    LlInitRtCfg_t llCfg = {
        .pBbRtCfg = &mainBbRtCfg,
        .wlSizeCfg = 4, .rlSizeCfg = 4, .plSizeCfg = 4,
        .pLlRtCfg = &mainLlRtCfg,
        .pFreeMem = WsfHeapGetFreeStartAddress(),
        .freeMemAvail = WsfHeapCountAvailable()
    };
    llmemUsed = LlInitSetRtCfg(&llCfg);
    WsfHeapAlloc(llmemUsed);
    llCfg.pFreeMem = WsfHeapGetFreeStartAddress();
    llCfg.freeMemAvail = WsfHeapCountAvailable();
    if (llmemUsed != LlInit(&llCfg)) {
        WSF_ASSERT(0);
    }
    WsfCsExit();

    bdAddr_t bdAddr;
    PalCfgLoadData(PAL_CFG_ID_BD_ADDR, bdAddr, sizeof(bdAddr_t));
    LlSetBdAddr((uint8_t *)&bdAddr);

    /* 32 kHz trim procedure */
    PalBbEnable();
    wutTrimComplete = 0;
    MXC_WUT_TrimCrystalAsync(MXC_WUT0, wutTrimCb);
    while (!wutTrimComplete) {}
    PalBbDisable();
#endif

/* Replace this: */
    /* AppInit(); */
    /* AppSetAdvType(DM_ADV_NONCONN_UNDIRECTED); */
    /* AppSetAdvInterval(160); */
    /* AppAdvSetData(APP_ADV_DATA_DISCOVERABLE, sizeof(sonoGuard_adv_data), (uint8_t *)sonoGuard_adv_data); */

    /* With the correct Cordio stack calls: */
    StackInitFit();
    FitStart(); /* registers the callbacks and resets the Bluetooth controller (as in the SDK BLE_fit example) */
    printf("Stage: Bluetooth started\n");
#endif
    /* -------------------------------------------------------------
     * Initialize Audio Hardware (I2S, Codec, Chime, DSP)
     * ------------------------------------------------------------- */
    /* Synthesize dual-tone attention chime (1200 Hz / 1800 Hz) */
    for (int i = 0; i < ALERT_TONE_SAMPLES; i++) {
        float f = (i < ALERT_TONE_SAMPLES / 2) ? 1200.0f : 1800.0f;
        float sample = sinf(2.0f * 3.14159265f * f * (float)i / (float)SMPL_RATE);
        alert_tone_buf[i] = (int16_t)(sample * 2000.0f);
    }

    if (i2s_init() != E_NO_ERROR) {
        printf("Error: I2S init failed\n");
        while (1) {}
    }
    printf("Stage: I2S started\n");
    int cerr = max9867_init(I2C_MASTER, 12288000, SMPL_RATE);
    if (cerr != E_NO_ERROR) {
        printf("Error: codec init failed (%d)\n", cerr);
        while (1) {}
    }
    printf("Stage: codec started\n");

    arm_rfft_fast_init_f32(&fft_handler, FFT_SIZE);
    for (int i = 0; i < FFT_SIZE; i++) {
        window[i] = 0.5f - 0.5f * cosf(2.0f * 3.14159265f * (float)i / (float)(FFT_SIZE - 1));
    }

    /* LED self-test: LED 0, 1, 2 light one after another so you can see which colour is which */
    for (int i = 0; i < 3; i++) {
        LED_On(i);
        MXC_Delay(MXC_DELAY_MSEC(300));
        LED_Off(i);
    }
    printf("Press SW2 to start (or wait 5 s and it starts by itself).\n");
    LED_On(1);
    {
        int waited_ms = 0;
        while (!PB_Get(0) && waited_ms < 5000) {
            MXC_Delay(MXC_DELAY_MSEC(10));
            waited_ms += 10;
        }
        if (PB_Get(0)) {
            printf("SW2 detected.\n");
            MXC_Delay(MXC_DELAY_MSEC(400));
            while (PB_Get(0)) {}
        } else {
            printf("No SW2 press seen, starting automatically.\n");
        }
    }
    printf("Learning noise floor for 2 s. Keep room quiet.\n");

    sg_det_t det;
    sg_det_reset(&det);

    void *i2s_buf = buf0;
    i2s_done = 0;
    MXC_DMA_ReleaseChannel(0);
    MXC_I2S_RXDMAConfig(i2s_buf, sizeof(buf0));

    uint32_t frame = 0;
    uint32_t overruns = 0;
    int learn_left = SG_LEARN_FRAMES;
    float nf_sum = 0.0f;
    float noise_floor = 1.0f;
    int cooldown = 0;
    int led_timer = 0;
    int mute = 0;
    bool btn_prev = false;

    /* -------------------------------------------------------------
     * Interleaved Real-Time Loop (Audio Processing + BLE Dispatcher)
     * ------------------------------------------------------------- */
    while (1) {
        /* While waiting for the 21.3 ms audio frame, dispatch BLE tasks */
#if SG_ENABLE_BLE
        while (!i2s_done) {
            wsfOsDispatcher();
        }
#else
        while (!i2s_done) {}
#endif

        int16_t *proc = (int16_t *)i2s_buf;
        i2s_buf = BUF_SEL(i2s_buf, buf0, buf1);
        i2s_done = 0;
        MXC_DMA_ReleaseChannel(0);
        MXC_I2S_RXDMAConfig(i2s_buf, sizeof(buf0)); /* Re-arm DMA immediately */

        int slot = (int)(frame % REPORT_SLOTS);
        uint32_t ms = (frame * 64u) / 3u;
        frame++;
#if SG_ENABLE_BLE
        if (frame > 150) { ble_ready = true; } /* about 3 s after the loop starts */
#endif

        /* Re-learn noise floor if SW2 is pressed */
        bool btn = PB_Get(0) ? true : false;
        if (btn && !btn_prev) {
            learn_left = SG_LEARN_FRAMES;
            nf_sum = 0.0f;
            sg_det_reset(&det);
            printf("Re-learning noise floor.\n");
        }
        btn_prev = btn;

        /* DC offset removal, RMS, Hann window, waveform envelope */
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

        /* Real FFT & Magnitudes */
        arm_rfft_fast_f32(&fft_handler, audio, fft_out, 0);
        arm_cmplx_mag_f32(fft_out, mags, FFT_SIZE / 2);
        mags[0] = 0.0f;

        /* Spectral energy and peak tracking */
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

        /* Ambient Noise Floor Learning & Tone Evaluation */
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
            if (!tonal && !det.active) {
                float32_t k = (band_total < noise_floor) ? NF_FALL : NF_RISE;
                noise_floor += (band_total - noise_floor) * k;
                if (noise_floor < 1.0f) noise_floor = 1.0f;
            }
        }

        /* Temporal Cadence / Siren Recognition Engine */
        int alert_bin = 0, detail = 0;
        sg_alert_t alert = SG_ALERT_NONE;
        if (learn_left == 0) {
            alert = sg_det_step(&det, tonal, peak_bin, &alert_bin, &detail);
        }
        if (cooldown > 0) cooldown--;

        /* Confirmed Hazard Trigger */
        if (alert != SG_ALERT_NONE && cooldown == 0) {
            int freq = (int)((float32_t)alert_bin * BIN_HZ);
            int ratio_pct = (int)(ratio * 100.0f);

            printf("A:%s,%d,%d,%d,%lu\n", (alert == SG_ALERT_SIREN) ? "SIREN" : "PULSED",
                   freq, ratio_pct, detail, (unsigned long)ms);

            LED_On(0);
            led_timer = ALERT_LED_FRAMES;
            cooldown = SG_COOLDOWN_FRAMES;
            mute = MUTE_FRAMES;

            /* Fire Headphone Alert Chime + Start BLE Broadcast to ESP32 */
            on_alert(alert, freq, ratio_pct);
        }
        if (led_timer > 0 && --led_timer == 0) LED_Off(0);

        /* Stop BLE advertising after the burst timer finishes */
#if SG_ENABLE_BLE
        if (ble_broadcast_timer > 0) {
            if (--ble_broadcast_timer == 0) {
                AppAdvStop();
            }
        }
#endif

        if (i2s_done) overruns++;

        /* Telemetry reports for PySide6 GUI */
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