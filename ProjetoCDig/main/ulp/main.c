#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"

#define PERTURB_GPIO  GPIO_NUM_8
#define HALF_PERIOD_US  500000   // 500ms → onda de 1Hz

int main(void)
{
    ulp_riscv_gpio_init(PERTURB_GPIO);
    ulp_riscv_gpio_output_enable(PERTURB_GPIO);

    while (1) {
        ulp_riscv_gpio_output_level(PERTURB_GPIO, 1);
        ulp_riscv_delay_cycles(HALF_PERIOD_US * ULP_RISCV_CYCLES_PER_US);

        ulp_riscv_gpio_output_level(PERTURB_GPIO, 0);
        ulp_riscv_delay_cycles(HALF_PERIOD_US * ULP_RISCV_CYCLES_PER_US);
    }

    return 0; // nunca chega aqui
}