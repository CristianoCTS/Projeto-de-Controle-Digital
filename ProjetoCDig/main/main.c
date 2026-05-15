#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include <stdint.h>


// Definições de Hardware
#define PWM_Heater 10
#define ADC_UNIT ADC_UNIT_1
#define ADC_Heater ADC_CHANNEL_0 
#define ADC_Ambient ADC_CHANNEL_1 
#define Set_Heater(Value) ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, Value, 0)

// Parametros
#define Sample_Period 1000  
#define PWM_Base 150   
#define PWM_Var 40

// Seed for random
static adc_oneshot_unit_handle_t adc_handle;

static uint16_t lfsr = 0xACE1u;

uint8_t gerar_prbs_3_10(void) {
    uint16_t bit = ((lfsr >> 0) ^ (lfsr >> 1) ^ (lfsr >> 3) ^ (lfsr >> 12)) & 1u;
    lfsr = (lfsr >> 1) | (bit << 15);
    return (lfsr & 0x07) + 3;
}

void init_pwm() {

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PWM_Heater,
        .duty = PWM_Base,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);

    ledc_fade_func_install(0);
}

void init_adc_oneshot() {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT,
    };
    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    adc_oneshot_config_channel(adc_handle, ADC_Heater, &config);
    adc_oneshot_config_channel(adc_handle, ADC_Ambient, &config);
}

float ADC_Read(adc_channel_t channel) {
    int raw;
    adc_oneshot_read(adc_handle, channel, &raw);
    
    float voltage = (raw * 3.3f) / 4095.0f;
    return voltage / 0.01f;
}

void app_main(void) {
    init_pwm();
    init_adc_oneshot();
    bool Switch = false;
    uint32_t PWM_Value = (PWM_Base - PWM_Var);
    int64_t last_time = -1;
    printf("-| Tempo (s) | PWM | Heater (°C) | Ambient (°C) |\n");
    uint8_t interval = gerar_prbs_3_10();
    uint8_t last_interval = 0;
    int64_t Last_impulse_time = 0;

    while (1) {
        
        int64_t Time = esp_timer_get_time()/1000;
        int64_t impulse_time = Time/60000;
        
        if (Time != last_time) {
            if (Time % (60000) == 0)
            {
                if ((impulse_time - Last_impulse_time) == interval) {
                    Last_impulse_time = impulse_time;
                    last_interval = interval;
                    Switch = !Switch;
                    PWM_Value = Switch ? (PWM_Base + PWM_Var) : (PWM_Base - PWM_Var);
                    Set_Heater(PWM_Value);
                    interval = gerar_prbs_3_10();
                    while (abs(interval - last_interval) <= 1){
                        interval = gerar_prbs_3_10();
                    }
                }
            } 
                    
            if (Time % Sample_Period == 0)
            {
                float Temp_H = ADC_Read(ADC_Heater);
                float Temp_A = ADC_Read(ADC_Ambient);
                printf("-| %lld | %lu | %.2f | %.2f |\n", (Time/1000), PWM_Value, Temp_H, Temp_A);
            }

            last_time = Time;
        }
        if (Time % Sample_Period == 3)
        {
            vTaskDelay(1);
        }
        

    }
}