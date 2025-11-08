#include "stm32f103xe.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"

void led_init()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef cfg = {};
    cfg.Pin = GPIO_PIN_0;
    cfg.Mode = GPIO_MODE_OUTPUT_PP;
    cfg.Pull = GPIO_NOPULL;
    cfg.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOB, &cfg);
}


int main()
{
    led_init();

    for ( ; ; )
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_Delay(2000);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_Delay(2000);
    }
}

void SysTick_Handler(void)
{
    HAL_IncTick();

}
