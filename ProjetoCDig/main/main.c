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
#define PWM_W 11
#define ADC_UNIT ADC_UNIT_1
#define ADC_Heater ADC_CHANNEL_0 
#define ADC_Ambient ADC_CHANNEL_1 
#define Set_Heater(Value) ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, Value, 0)
#define Set_W(ValueW) ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, ValueW, 0)

// Seed pro random
static uint16_t lfsr = 0xACE1u;

// Parametros do código
#define Coleta_de_Dados false
bool transient = true;
#define R_quadradico true

#define Sample_Period 3700  
#define PWM_Base 150   
#define PWM_Var 40
#define MEDIAN_SIZE 5
#define KP 0.03f
#define KI (KP * 17.0f)
#define SATmax 600.0f
#define SATmin 0.0f
#define Smplg 1.0f
#define ALPHA 0.15f
#define UpperLimit 600.0f
#define LowerLimit SATmin
#define Wstep 800
#define Wfreq 0.02f

static adc_oneshot_unit_handle_t adc_handle;
static float median_sorted[MEDIAN_SIZE];
uint32_t PWM_Value = 0;

// Parametros do projeto atualizados conforme OutputMatLab.txt
#define L1 47.893f
#define L2 18.225f
#define L3 3.3082f
#define L4 -0.70755f
#define K 0.009604f
#define w sqrtf(K)
#define Kp 0.23573f
#define K1 0.02865f
#define K2 0.5643f
#define Tp1 400.0f
#define Tp2 45.337f
#define ATp1 0.00070489f
#define BTp2 0.015838f
#define Ki 1.2463f
#define Nb 20.0f //30.964f
#define Ts Sample_Period/1000.0f

#define euler1 exp(-Ts/Tp1)
#define euler2 exp(-Ts/Tp2)
#define a1 2*cosf(w*Ts)
#define b1 (L3/w)*(sinf(w*Ts))+((L4/(K))*(1-cosf(w*Ts)))
#define c1 ((-L3/w)*(sinf(w*Ts)))+((L4/(K))*(1-cosf(w*Ts)))
#define a2 2*cosf(w*Ts)
#define b2 ((L4/w)*(sinf(w*Ts)))-(L3*(1-cosf(w*Ts)))
#define c2 ((-L4/w)*(sinf(w*Ts)))-(L3*(1-cosf(w*Ts)))

float xs1_old2 = 0;
float xs1_old = 0;
float xs2_old2 = 0;
float xs2_old = 0;
float eep_old2 = 0;
float ees_old2 = 0;
float eep_old = 0;
float ees_old = 0;
float x1e = 0;
float x2e = 0;
float x1e_old = 0;
float x2e_old = 0;
float Uobs_old = 0;
float xi_old = 0;
float xi = 0;
float er_old = 0;
float ee = 0;
float Ye = 25;



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
float perturbacao(float t_s)
{
    float fase = fmodf(t_s, 2700.0f);
    if (fase < 900.0f) {
        return 0.0f;
    } else if (fase < 1800.0f) {
        return Wstep;
    } else {
        float t_sine = fase - 1800.0f;
        float sine   = Wstep * sinf(2.0f * (float)M_PI * Wfreq * t_sine);
        return Wstep + sine;
    }
}

float PI_control(float Th, float Target, float sat_max, float sat_min)
{
    xi = xi_old + (Sample_Period*er_old);
    float Upi = (Nb*Target) + xi;
    float er = Target - Th;
    bool clamping = ((er > 0.0f) && (Upi > sat_max)) || ((er < 0.0f) && (Upi < sat_min));

    if (clamping) {
        //pass
    }if (Upi > sat_max) {
        Upi = sat_max;
    }else if (Upi < sat_min) {
        Upi = sat_min;
    }else {
        xi_old = xi;
    }
    er_old = er;
    
    return Upi;
}

float observador_processo(float Uobs, float ee)
{
    x1e = (euler1*x1e_old) + ((Tp1*(1-euler1))*((L1*eep_old)+(Kp*(Uobs_old))));
    x2e = (euler2*x2e_old) + ((Tp2*(1-euler2))*((L2*eep_old)+(Kp*(Uobs_old))));
    float Yestimado = ((ATp1)*x1e) + ((BTp2)*x2e);

    x1e_old = x1e;
    x2e_old = x2e;
    Uobs_old = Uobs;
    eep_old = ee;
    if (Yestimado < 20.0f) {
        Yestimado = 20.0f;
    }

    return Yestimado;
}

float observador_senoidal(float ee)
{
    float xs1 = (a1*xs1_old) - xs1_old2 +(b1*ees_old)+(c1*ees_old2);
    float xs2 = (a2*xs2_old) - xs2_old2 +(b2*ees_old)+(c2*ees_old2);
    xs1_old2 = xs1_old;
    xs1_old = xs1;
    xs2_old2 = xs2_old;
    xs2_old = xs2;
    ees_old2 = ees_old;
    ees_old = ee;

    if (xs1 > SATmax) {
        xs1 = SATmax;
    }else if (xs1 < -SATmax) {
        xs1 = -SATmax;
    }else {
        //pass
    }
    return xs1;
}

float realimentacao_estados()
{
    float Kxe = (K1*x1e) + (K2*x2e);

    if (Kxe > SATmax) {
        Kxe = SATmax;
    }else if (Kxe < -SATmax) {
        Kxe = -SATmax;
    }else {
        //pass
    }
    return Kxe;
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
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz         = 5000,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel0 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = PWM_Heater,
        .duty       = PWM_Base,
        .hpoint     = 0
    };
    ledc_channel_config(&ledc_channel0);

    ledc_channel_config_t ledc_channel1 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_1,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = 11,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ledc_channel1);

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
    printf("| Tempo (s) | PWM | Tensão(V) | Heater(°C) | Heater raw(°C) | Ambient(°C) | Ambient raw(°C) | Temp Alvo(°C) | Erro(°C) | W(°C) |\n");
    MedianFilter mf_heater = {0}, mf_ambient = {0};
    EWMFilter    ef_heater = {0}, ef_ambient = {0};
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
            else if (!R_quadradico){
                Target = 60.0f;
            } 
            if (Time % Sample_Period == 0)
            {
                float perturb = perturbacao(Time/1000);
                Set_W((uint32_t)perturb);
                
                float Th_raw = ADC_Read(ADC_Heater);
                float Ta_raw = ADC_Read(ADC_Ambient);
                float Th = ewm_update(&ef_heater,  median_update(&mf_heater,  Th_raw));
                float Ta = ewm_update(&ef_ambient, median_update(&mf_ambient, Ta_raw));
                float Volts = (PWM_Value*0.008f);
                float Error = Th - Target;
                float perturbC = -perturb*0.005f;
                
                printf("-| %lld | %lu | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f |\n", (Time/1000), PWM_Value, Volts, Th, Th_raw, Ta, Ta_raw, Target, Error, perturbC);
                
                if (transient){
                    float Upi = PI_control(Th_raw, Target, SATmax, SATmin);
                    printf("Uobs: %.2f | Upi: %.2f | Ye: %.2f | Uk: %.2f | ee: %.2f | e_sin: %.2f\n", 0.0f, Upi, 0.0f, 0.0f, 0.0f, 0.0f);

                    PWM_Value = (uint32_t)(Upi);

                    if (PWM_Value > SATmax) {
                        PWM_Value = SATmax;
                    }else if (PWM_Value < SATmin) {
                        PWM_Value = SATmin;
                    }
                    Set_Heater(PWM_Value);
                    if (Th > 60.0f) {
                        transient = false;
                    }
                }else{
                    ee = Th_raw - Ye;
                    float Upi = PI_control(Th_raw, Target, SATmax, SATmin);
                    Ye = observador_processo(Uobs, ee);
                    float Uk = realimentacao_estados();
                    Uobs = Upi - Uk;
                    float e_sin = observador_senoidal(ee);
                    printf("Uobs: %.2f | Upi: %.2f | Ye: %.2f | Uk: %.2f | ee: %.2f | e_sin: %.2f\n", Uobs, Upi, Ye, Uk, ee, e_sin);


                    if (e_sin > Uobs){
                        PWM_Value = SATmin;
                    }else if (PWM_Value < SATmin){
                        PWM_Value = SATmin;
                    }else {
                        PWM_Value = (uint32_t)(Uobs-e_sin);
                    }

                    if (PWM_Value > SATmax) {
                        PWM_Value = SATmax;
                    }else if (PWM_Value < SATmin) {
                        PWM_Value = SATmin;
                    }
                    Set_Heater(PWM_Value);
                }
            }

            last_time = Time;
        }
        if (Time % Sample_Period == 3)
        {
            vTaskDelay(1);
        }
        

    }
}