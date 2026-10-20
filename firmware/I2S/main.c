#include "ff.h"
#include "gpio.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "dma.h"
#include "i2s.h"
#include "icc.h"
#include "led.h"
#include "max9867.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_sys.h"
#include "nvic_table.h"
#include "pb.h"

/***** Definitions *****/
#if defined(BOARD_FTHR_APPS_P1)
#define I2C_MASTER MXC_I2C1 // SCL P0_16; SDA P0_17
#define CONNECT_NUM 2
#else
#define I2C_MASTER MXC_I2C2 // SCL P0_30; SDA P0_31
#define CONNECT_NUM 3
#endif

// Reduced buffer size for safer SD card latency 
#define BUF_SIZE 4096
#define SMPL_RATE 48000
#define BUF_SEL(CUR_BUF, BUF1, BUF2) ((CUR_BUF == BUF1) ? BUF2 : BUF1)

/***** Global Data *****/
mxc_i2s_req_t i2s_req;
volatile uint8_t i2s_done = 0;
int16_t buf0[BUF_SIZE];
int16_t buf1[BUF_SIZE];

/*****************************************************************/
void i2s_dma_cb(int handle, int error)
{
    i2s_done = 1;
}

/*****************************************************************/
void DMA_IRQHandler(void)
{
    MXC_DMA_Handler();
}

/*****************************************************************/
int i2s_init(void)
{
    int err = E_NO_ERROR;

    i2s_req.wordSize = MXC_I2S_WSIZE_HALFWORD; 
    i2s_req.sampleSize = MXC_I2S_SAMPLESIZE_SIXTEEN;
    i2s_req.bitsWord = 16;
    i2s_req.adjust = MXC_I2S_ADJUST_LEFT;
    i2s_req.justify = MXC_I2S_LSB_JUSTIFY;
    i2s_req.wsPolarity = MXC_I2S_POL_NORMAL;
    i2s_req.channelMode = MXC_I2S_EXTERNAL_SCK_EXTERNAL_WS; 
    i2s_req.stereoMode = MXC_I2S_MONO_RIGHT_CH; // Capturing 1 channel
    i2s_req.bitOrder = MXC_I2S_LSB_FIRST;
    i2s_req.clkdiv = MXC_I2S_CalculateClockDiv(SMPL_RATE, i2s_req.wordSize) + 1;
    i2s_req.rawData = NULL;
    i2s_req.txData = NULL;
    i2s_req.rxData = buf0;
    i2s_req.length = sizeof(buf0);

    err = MXC_I2S_Init(&i2s_req); 
    if (err != E_NO_ERROR) {
        return err;
    }

    MXC_I2S_RegisterDMACallback(i2s_dma_cb);
    MXC_NVIC_SetVector(DMA0_IRQn, DMA_IRQHandler);

    return err;
}

/*****************************************************************/
void record_to_sd(const char *filename)
{
    // 1. Power on the SD Card (P0.15 Active Low)
    mxc_gpio_cfg_t sd_power;
    sd_power.port = MXC_GPIO0;
    sd_power.mask = MXC_GPIO_PIN_15;
    sd_power.pad = MXC_GPIO_PAD_NONE;
    sd_power.func = MXC_GPIO_FUNC_OUT;
    MXC_GPIO_Config(&sd_power);
    MXC_GPIO_OutClr(sd_power.port, sd_power.mask);
    MXC_Delay(MXC_DELAY_MSEC(100)); // Allow power to stabilize

    // 2. Mount the File System
    FATFS fs;
    FRESULT err = f_mount(&fs, "", 1);
    if (err != FR_OK) {
        printf("SD Card Mount Failed: %d\n", err);
        return;
    }
    printf("SD Card Mounted.\n");

    // 3. Create the WAV File
    FIL file;
    err = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (err != FR_OK) {
        printf("Failed to create %s: %d\n", filename, err);
        return;
    }

    printf("Created file: %s\n", filename);

    // 4. Calculate sizes for 5 seconds of audio
    uint32_t sample_rate = SMPL_RATE;
    uint16_t channels = 1; // We set MXC_I2S_MONO_RIGHT_CH above
    uint16_t bits_per_sample = 16;
    uint32_t num_seconds = 5;
    
    uint32_t byte_rate = sample_rate * channels * (bits_per_sample / 8);
    uint32_t data_size = byte_rate * num_seconds;
    uint32_t file_size = 36 + data_size;
    UINT bw;

    // 5. Write the RIFF WAV Header
    f_write(&file, "RIFF", 4, &bw);
    f_write(&file, &file_size, 4, &bw);
    f_write(&file, "WAVEfmt ", 8, &bw);
    uint32_t fmt_size = 16;
    f_write(&file, &fmt_size, 4, &bw);
    uint16_t format_type = 1; // PCM
    f_write(&file, &format_type, 2, &bw);
    f_write(&file, &channels, 2, &bw);
    f_write(&file, &sample_rate, 4, &bw);
    f_write(&file, &byte_rate, 4, &bw);
    uint16_t block_align = channels * (bits_per_sample / 8);
    f_write(&file, &block_align, 2, &bw);
    f_write(&file, &bits_per_sample, 2, &bw);
    f_write(&file, "data", 4, &bw);
    f_write(&file, &data_size, 4, &bw);

    printf("Recording 5 seconds of audio...\n");

    uint32_t bytes_written_total = 0;
    void *i2s_buf = buf0;
    void *write_buf = buf0;

    i2s_done = 0;
    MXC_DMA_ReleaseChannel(0);
    MXC_I2S_RXDMAConfig(i2s_buf, sizeof(buf0)); // Start filling the first buffer

    LED_On(0); // Turn on Red LED during recording

    // 6. Capture and write the audio using DMA ping-pong buffers
    while (bytes_written_total < data_size) {
        // Wait for the DMA to finish filling the current buffer
        while (!i2s_done) {}
        
        write_buf = i2s_buf; // The buffer that is now full
        i2s_buf = BUF_SEL(i2s_buf, buf0, buf1); // Swap to the empty buffer
        
        i2s_done = 0; 
        MXC_DMA_ReleaseChannel(0);
        MXC_I2S_RXDMAConfig(i2s_buf, sizeof(buf0)); // Restart DMA immediately in the background
        
        // Write the full buffer to the SD card while DMA fills the other buffer
        f_write(&file, write_buf, sizeof(buf0), &bw); 
        bytes_written_total += bw;
    }

    // 7. Clean up and close
    LED_Off(0);
    f_close(&file);
    f_unmount("");
    printf("Recording saved to SD card successfully.\n");
}

/*****************************************************************/
int main(void)
{
    int err;
    int file_index = 0;
    char current_filename[16];

    MXC_ICC_Enable(MXC_ICC0); 
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO); 
    SystemCoreClockUpdate();

    printf("\n******************* SonoGuard SD Card Capture *******************\n");

    err = i2s_init(); // Initialize I2S interface
    if (err != E_NO_ERROR) {
        printf("Error configuring I2S interface.\n");
        while (1) {}
    }

    err = max9867_init(I2C_MASTER, 12288000, SMPL_RATE); // Initialize audio codec
    if (err != E_NO_ERROR) {
        printf("Error configuring audio codec: %i\n", err);
        while (1) {}
    }

    // Infinite loop to allow multiple recordings
    while(1) {
        printf("\nPress SW2 to record file: cap%d.wav\n", file_index);
        LED_On(1); // Green LED indicates ready state
        
        while (!PB_Get(0)) {} // Wait for SW2 button press
        MXC_Delay(MXC_DELAY_MSEC(500)); // Button debounce
        LED_Off(1);

        // Generate the new filename (e.g., cap0.wav, cap1.wav)
        sprintf(current_filename, "cap%d.wav", file_index);
        
        // Execute the SD card routine with the new filename
        record_to_sd(current_filename);
        
        // Increment the counter for the next recording
        file_index++;
    }

    return E_NO_ERROR;
}