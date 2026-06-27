#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include <stdint.h>
#include <math.h>

// Definições de Hardware
#define PWM_Heater 10
#define ADC_UNIT ADC_UNIT_1
#define ADC_Heater ADC_CHANNEL_0 
#define ADC_Ambient ADC_CHANNEL_1 
#define Set_Heater(Value) ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, Value, 0)

// Seed pro random
static uint16_t lfsr = 0xACE1u;

// Parametros do código
#define Sample_Period 4000  
#define PWM_Base 150   
#define PWM_Var 40
#define MEDIAN_SIZE 5
#define KP 0.03f
#define KI (KP * 17.0f)
#define SATmax 1024.0f
#define SATmin 0.0f
#define Smplg 1.0f
#define ALPHA 0.15f
#define R_quadradico true

static adc_oneshot_unit_handle_t adc_handle;
static float median_sorted[MEDIAN_SIZE];
bool Coleta_de_Dados = false;
uint32_t PWM_Value = 0;

//Parametros do projeto
#define KP_MOD 0.2357f
#define TP1 400.0f
#define TP2 45.337f
#define A_GAIN (KP_MOD * TP1)
#define B_GAIN (KP_MOD * TP2)
#define NB 27.497f
#define KI_GAIN 1.9241f
#define KA1 0.0213f
#define KA2 0.4825f
#define KXE (-1.9241f)
#define L1 58.4773f
#define L2 18.2415f
#define L3 3.9058f
#define L4 (-0.6964f)
#define W0 (2.0f * (float)M_PI / 300.0f)

static float g_pi_c;
static float g_ad1, g_bd1, g_ld1;
static float g_ad2, g_bd2, g_ld2;
static float g_Fss[2][2];
static float g_Gss[2];
static float s_xi = 0.0f;
static float s_er_prev = 0.0f;
static float s_x1e = 0.0f;
static float s_x2e = 0.0f;
static float s_u_prev = 0.0f;
static float s_ee_prev = 0.0f;
static float s_xs1 = 0.0f;
static float s_xs2 = 0.0f;
static float s_eesin_prev = 0.0f;
float estados[2];

//Parametros dos filtros
typedef struct {
    float integrador;
} PIController;
typedef struct {
    float buffer[MEDIAN_SIZE];
    uint8_t index;
    uint8_t count;
} MedianFilter;
typedef struct {
    float value;
    bool initialized;
} EWMFilter;

//Funções
void parametros_projeto(float Ts)
{
    float T2 = Ts * 0.5f;
    g_pi_c = KI_GAIN * T2;
    float a1   = 1.0f / TP1;
    float den1 = 1.0f + a1 * T2;
    g_ad1 = (1.0f - a1 * T2) / den1;
    g_bd1 = (T2 * (A_GAIN / TP1)) / den1;
    g_ld1 = (L1 * T2) / den1;
    float a2   = 1.0f / TP2;
    float den2 = 1.0f + a2 * T2;
    g_ad2 = (1.0f - a2 * T2) / den2;
    g_bd2 = (T2 * (B_GAIN / TP2)) / den2;
    g_ld2 = (L2 * T2) / den2;
    float w2  = W0 * W0;
    float det = 1.0f + w2 * T2 * T2;
    g_Fss[0][0] =  (1.0f - w2 * T2 * T2) / det;
    g_Fss[0][1] =  (2.0f * T2)/ det;
    g_Fss[1][0] = -(2.0f * w2 * T2)/ det;
    g_Fss[1][1] =  (1.0f - w2 * T2 * T2) / det;
    g_Gss[0] =  Ts * (L3+ T2 * L4)/ det;
    g_Gss[1] =  Ts * (-w2 * T2 * L3+ L4)/ det;
}

float PI_control(float Th, float Target, float sat_max, float sat_min)
{
    float er = Target - Th;
    float xi_new = s_xi + g_pi_c * (er + s_er_prev);
    float Upi = NB * er + KI_GAIN * xi_new;
    bool clamping = ((er > 0.0f) && (Upi > sat_max)) || ((er < 0.0f) && (Upi < sat_min));

    if (clamping) {
        xi_new = s_xi;
        Upi   = s_xi;
    }
    if (Upi > sat_max) {
        Upi = sat_max;
    }
    else if (Upi < sat_min) {
        Upi = sat_min;
    }
    s_xi = xi_new;
    s_er_prev = er;
    return Upi;
}

float observador_processo(float Th, float Uobs, float estados[2])
{
    float Ye_prev = s_x1e + s_x2e;
    float ee = Th - Ye_prev;
    float x1e_new = g_ad1 * s_x1e + g_bd1 * (Uobs + s_u_prev) + g_ld1 * (ee   + s_ee_prev);
    float x2e_new = g_ad2 * s_x2e + g_bd2 * (Uobs + s_u_prev) + g_ld2 * (ee   + s_ee_prev);
    s_x1e    = x1e_new;
    s_x2e    = x2e_new;
    s_u_prev = Uobs;
    s_ee_prev = ee;
    float Ye = x1e_new + x2e_new;
    estados[0] = x1e_new;
    estados[1] = x2e_new;
    return Ye;
}

float observador_senoidal(float ee)
{
    float ee_sum = ee + s_eesin_prev;
    float xs1_new = g_Fss[0][0] * s_xs1 + g_Fss[0][1] * s_xs2 + (g_Gss[0] * ee_sum * 0.5f);
    float xs2_new = g_Fss[1][0] * s_xs1 + g_Fss[1][1] * s_xs2 + (g_Gss[1] * ee_sum * 0.5f);
    s_xs1 = xs1_new;
    s_xs2 = xs2_new;
    s_eesin_prev = ee;
    return xs1_new;
}

float realimentacao_estados(float estados[2])
{
    float Uk = KA1 * estados[0] + KA2 * estados[1] + KXE * s_xs1;
    return Uk;
}

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

uint8_t gerar_prbs_1_15(void) {
    uint16_t bit = ((lfsr >> 0) ^ (lfsr >> 1) ^ (lfsr >> 3) ^ (lfsr >> 12)) & 1u;
    lfsr = (lfsr >> 1) | (bit << 15);
    return (lfsr & 0x0F) + 1;
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

void app_main(void) {
    init_pwm();
    init_adc_oneshot();
    int64_t last_time = -1;
    printf("| Tempo (s) | PWM | Tensão(V) | Heater Normalizado(°C) | Ambient Normalizado(°C) | Heater(°C) | Ambient(°C) | Temp Alvo(°C) | Erro(°C) |\n");
    MedianFilter mf_heater = {0}, mf_ambient = {0};
    EWMFilter    ef_heater = {0}, ef_ambient = {0};
    parametros_projeto(Sample_Period/1000);
    Set_Heater(PWM_Value);
    float Uobs = 0.0f;
    float Target = 60.0f;
    bool Switch = false;

    while (1) {
        int64_t Time = esp_timer_get_time()/1000;

        if (Time != last_time) {
            if ((Time % (400*1000) == 0) && R_quadradico) {
                Switch = !Switch;
                Target = Switch ? (Target + 5.0f) : (Target - 5.0f);
            } 
            else {
                Target = 60.0f;
            } 
            if (Time % Sample_Period == 0)
            {
                float Th_raw = ADC_Read(ADC_Heater);
                float Ta_raw = ADC_Read(ADC_Ambient);
                float Th = ewm_update(&ef_heater,  median_update(&mf_heater,  Th_raw));
                float Ta = ewm_update(&ef_ambient, median_update(&mf_ambient, Ta_raw));
                float Volts = (PWM_Value*0.008f);
                float Error = Th - Target;
                
                printf("-| %lld | %lu | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f |\n", (Time/1000), PWM_Value, Volts, Th, Ta, Th_raw, Ta_raw, Target, Error);

                float Upi = PI_control(Th, Target, SATmax, SATmin);
                float Ye = observador_processo(Th, Uobs, estados);
                float Uk = realimentacao_estados(estados);
                Uobs = Upi - Uk;
                float ee = Th - Ye;
                float e_sin = observador_senoidal(ee);
                PWM_Value = (uint32_t)(Uobs-e_sin);

                Set_Heater(PWM_Value);
            }

            last_time = Time;
        }
        if (Time % Sample_Period == 3)
        {
            vTaskDelay(1);
        }
        

    }
}