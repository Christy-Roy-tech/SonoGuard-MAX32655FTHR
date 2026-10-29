void live_detector(void)
{
    printf("Initializing DSP...\n");
    arm_rfft_fast_init_f32(&fft_handler, FFT_SIZE);
    
    void *i2s_buf = buf0;
    int16_t *process_buf = buf0;
    i2s_done = 0;
    MXC_DMA_ReleaseChannel(0);
    MXC_I2S_RXDMAConfig(i2s_buf, sizeof(buf0)); 
    
    printf("Live Detection Running. Play a 1250Hz tone!\n");

    while (1) {
        while (!i2s_done) {} 
        
        process_buf = (int16_t *)i2s_buf; 
        i2s_buf = BUF_SEL(i2s_buf, buf0, buf1); 
        i2s_done = 0; 
        MXC_DMA_ReleaseChannel(0);
        MXC_I2S_RXDMAConfig(i2s_buf, sizeof(buf0)); 
        
        for (int i = 0; i < FFT_SIZE; i++) {
            audio_float_buffer[i] = (float32_t)process_buf[i];
        }

        arm_rfft_fast_f32(&fft_handler, audio_float_buffer, fft_output, 0);
        arm_cmplx_mag_f32(fft_output, magnitudes, FFT_SIZE / 2);

        float32_t total_energy = 0.0f;
        float32_t band_energy = 0.0f;

        for (int i = 0; i < (FFT_SIZE / 2); i++) {
            float32_t energy = magnitudes[i] * magnitudes[i];
            total_energy += energy;
            if (i >= TARGET_BIN_LOW && i <= TARGET_BIN_HIGH) {
                band_energy += energy;
            }
        }

        if (total_energy > 50000000.0f) { 
            float32_t energy_ratio = band_energy / total_energy;
            if (energy_ratio > 0.40f) { 
                // FIX 2: Integer math only for printf to prevent format crashes
                int ratio_pct = (int)(energy_ratio * 100.0f);
                printf("[HAZARD DETECTED] Beeper/Siren Signature! Ratio: %d%%\n", ratio_pct);
                LED_On(0);  
            } else {
                LED_Off(0); 
            }
        } else {
            LED_Off(0);
        }
    }
}

int main(void)
{
    // FIX 1: Clock select lines removed. We now trust the MSDK's automatic Board_Init().

    printf("\n\n[--- SONOGUARD BOOTING ---]\n");

    printf("1. Testing I2S... ");
    int err_i2s = i2s_init();
    if (err_i2s != E_NO_ERROR) {
        printf("FAILED (Error %d)\n", err_i2s);
        while(1);
    }
    printf("OK\n");

    printf("2. Testing Codec... ");
    int err_codec = max9867_init(I2C_MASTER, 12288000, SMPL_RATE);
    if (err_codec != E_NO_ERROR) {
        printf("FAILED (Error %d)\n", err_codec);
        while(1);
    }
    printf("OK\n");

    printf("\nPress SW2 to start the Live Hazard Detector.\n");
    LED_On(1); 
    
    while (!PB_Get(0)) {} 
    MXC_Delay(MXC_DELAY_MSEC(500)); 
    LED_Off(1);

    live_detector(); 

    while(1);
    return 0;
}