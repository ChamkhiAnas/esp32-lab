#include <Arduino.h> 
#include "driver/i2s.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define I2S_BCLK_PIN    12 
#define I2S_LRC_PIN     14  
#define I2S_DOUT_PIN    13  

#define SAMPLE_RATE     44100
#define I2S_NUM         I2S_NUM_0
#define BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_16BIT
#define CHANNEL_FORMAT  I2S_CHANNEL_FMT_RIGHT_LEFT
#define COMMUNICATION_FORMAT I2S_COMM_FORMAT_STAND_I2S

#define TEST_FREQUENCY  440  
#define AMPLITUDE       30000
#define BUFFER_SIZE     1024

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 MAX98357A Audio Test Starting...");
    
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = BITS_PER_SAMPLE,
        .channel_format = CHANNEL_FORMAT,
        .communication_format = COMMUNICATION_FORMAT,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = BUFFER_SIZE,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK_PIN,
        .ws_io_num = I2S_LRC_PIN,
        .data_out_num = I2S_DOUT_PIN,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    esp_err_t result = i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    if (result != ESP_OK) {
        Serial.printf("Error installing I2S driver: %d\n", result);
        return;
    }

    result = i2s_set_pin(I2S_NUM, &pin_config);
    if (result != ESP_OK) {
        Serial.printf("Error setting I2S pins: %d\n", result);
        return;
    }

    Serial.println("I2S initialized successfully!");
    Serial.println("You should hear a 440Hz test tone...");
}

void loop() {
    int16_t audio_buffer[BUFFER_SIZE * 2]; 
    
    static float phase = 0.0;
    float phase_increment = 2.0 * M_PI * TEST_FREQUENCY / SAMPLE_RATE;
    
    for (int i = 0; i < BUFFER_SIZE; i++) {
        int16_t sample = (int16_t)(sin(phase) * AMPLITUDE);
        
        audio_buffer[i * 2] = sample;    
        audio_buffer[i * 2 + 1] = sample; 
        
        phase += phase_increment;
        if (phase >= 2.0 * M_PI) {
            phase -= 2.0 * M_PI;
        }
    }
    
    // Write audio data to I2S
    size_t bytes_written;
    esp_err_t result = i2s_write(I2S_NUM, audio_buffer, sizeof(audio_buffer), &bytes_written, portMAX_DELAY);
    
    if (result != ESP_OK) {
        Serial.printf("Error writing to I2S: %d\n", result);
        delay(1000);
    }
    
    static unsigned long last_print = 0;
    if (millis() - last_print > 3000) {
        Serial.printf("Playing test tone... Bytes written: %d\n", bytes_written);
        last_print = millis();
    }
}

void playTestPattern() {
    Serial.println("Playing test pattern: beeps and silence");
    
    for (int cycles = 0; cycles < 100; cycles++) {
        int16_t audio_buffer[BUFFER_SIZE * 2];
        static float phase = 0.0;
        float phase_increment = 2.0 * M_PI * 1000 / SAMPLE_RATE; 
        
        for (int i = 0; i < BUFFER_SIZE; i++) {
            int16_t sample = (int16_t)(sin(phase) * 4000);
            audio_buffer[i * 2] = sample;
            audio_buffer[i * 2 + 1] = sample;
            phase += phase_increment;
            if (phase >= 2.0 * M_PI) phase -= 2.0 * M_PI;
        }
        
        size_t bytes_written;
        i2s_write(I2S_NUM, audio_buffer, sizeof(audio_buffer), &bytes_written, portMAX_DELAY);
    }
    
    int16_t silence_buffer[BUFFER_SIZE * 2] = {0};
    for (int cycles = 0; cycles < 100; cycles++) {
        size_t bytes_written;
        i2s_write(I2S_NUM, silence_buffer, sizeof(silence_buffer), &bytes_written, portMAX_DELAY);
    }
}