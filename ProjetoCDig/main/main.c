#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include <stdint.h>
#include <stdbool.h>

// Definições de Hardware
#define PWM_Heater 10
#define ADC_UNIT ADC_UNIT_1
#define ADC_Heater ADC_CHANNEL_0
#define ADC_Ambient ADC_CHANNEL_1
#define Set_Heater(Value) ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, Value, 0)

// Parâmetros
#define Sample_Period 1000   // ms
#define PO            60.0f  // Ponto de operação °C
#define PWM_Base      150
#define PWM_Var       40
#define MEDIAN_SIZE   5
#define ALPHA         0.15f

// Saturação PWM
#define USmax  1023.0f
#define USmin  0.0f

// Controlador
#define K1   0.0213f
#define K2   0.4825f
#define KI_c 0.1017f
#define NB   27.492f

// Observador aumentado
// L = [58.4773; 18.2415; 3.9058; -0.6964]
#define L1  58.4773f
#define L2  18.2415f
#define L3   3.9058f
#define L4  -0.6964f

// Matriz [x1e; x2e; xs1; xs2]
// Ad (4x4), Bd (4x1), Cd (1x4)
static const float Ad[4][4] = {
    { 0.9975f,  0.0f,    0.2351f,  0.1177f },
    { 0.0f,     0.9782f, 0.2328f,  0.1169f },
    { 0.0f,     0.0f,    0.9952f,  0.9984f },
    { 0.0f,     0.0f,   -0.0096f,  0.9952f }
};
static const float Bd[4] = { 0.2354f, 0.2331f, 0.0f, 0.0f };
static const float Cd[4] = { 0.0007f, 0.0158f, 0.0f, 0.0f };

// Filtros
static adc_oneshot_unit_handle_t adc_handle;
static float median_sorted[MEDIAN_SIZE];

typedef struct {
    float buffer[MEDIAN_SIZE];
    uint8_t index;
    uint8_t count;
} MedianFilter;

typedef struct {
    float value;
    bool initialized;
} EWMFilter;

float median_update(MedianFilter *f, float new_val) {
    f->buffer[f->index] = new_val;
    f->index = (f->index + 1) % MEDIAN_SIZE;
    if (f->count < MEDIAN_SIZE) f->count++;
    uint8_t n = f->count;
    for (int i = 0; i < n; i++) median_sorted[i] = f->buffer[i];
    for (int i = 0; i < n - 1; i++)
        for (int j = i + 1; j < n; j++)
            if (median_sorted[j] < median_sorted[i]) {
                float tmp = median_sorted[i];
                median_sorted[i] = median_sorted[j];
                median_sorted[j] = tmp;
            }
    return median_sorted[n / 2];
}

float ewm_update(EWMFilter *f, float new_val) {
    if (!f->initialized) {
        f->value = new_val;
        f->initialized = true;
    } else {
        f->value = ALPHA * new_val + (1.0f - ALPHA) * f->value;
    }
    return f->value;
}

// Configurações hardware
void init_pwm(void) {
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

void init_adc_oneshot(void) {
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT };
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

//main
void app_main(void) {
    init_pwm();
    init_adc_oneshot();

    MedianFilter mf_heater = {0}, mf_ambient = {0};
    EWMFilter    ef_heater = {0}, ef_ambient = {0};

    float xe[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float xi = 0.0f;
    float u_prev = 0.0f;

    int64_t last_time = -1;

    printf("| Tempo(s) | PWM | Us | Ye | e_sin | Temp_H | Temp_A | Ref | Erro |\n");

    while (1) {
        int64_t Time = esp_timer_get_time() / 1000;

        if (Time != last_time && Time % Sample_Period == 0) {
            float raw_H = ADC_Read(ADC_Heater);
            float raw_A = ADC_Read(ADC_Ambient);
            float Temp_H = ewm_update(&ef_heater,  median_update(&mf_heater,  raw_H));
            float Temp_A = ewm_update(&ef_ambient, median_update(&mf_ambient, raw_A));
            float Ye = Cd[0]*xe[0] + Cd[1]*xe[1] + Cd[2]*xe[2] + Cd[3]*xe[3];
            float innov = Temp_H - PO - Ye;
            float e_sin = xe[2];
            float Kxe = K1*xe[0] + K2*xe[1];
            float er = PO - Temp_H;
            float Us = (NB*PO + KI_c*xi) - Kxe - e_sin;
            float xe_new[4];
            bool clamping = ((er > 0.0f) && (Us > USmax)) || ((er < 0.0f) && (Us< USmin));

            if (!clamping) {
                xi += er;
            }
            if (Us > USmax) {
                Us = USmax;
            }
            else if (Us < USmin) {
                Us = USmin;
            }

            for (int i = 0; i < 4; i++) {
                xe_new[i] = Ad[i][0]*xe[0] + Ad[i][1]*xe[1] + Ad[i][2]*xe[2] + Ad[i][3]*xe[3] + Bd[i]*u_prev;
            }
            xe_new[0] += L1 * innov;
            xe_new[1] += L2 * innov;
            xe_new[2] += L3 * innov;
            xe_new[3] += L4 * innov;

            for (int i = 0; i < 4; i++) {
                xe[i] = xe_new[i];
            }

            Set_Heater(Us);
            u_prev = Us;

            printf("| %lld | %lu | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f |\n",
                   (Time/1000), PWM_Value, Us, Ye, e_sin, Temp_H, Temp_A, PO, er);

            last_time = Time;
        }

        if (Time % Sample_Period == 3) {
            vTaskDelay(1);
        }
    }
}