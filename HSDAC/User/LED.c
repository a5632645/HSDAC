#include "LED.h"
#include "ch32v30x_gpio.h"
#include "ch32v30x_rcc.h"

#define XSMT_PIN GPIO_Pin_7

void LED_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    
    GPIO_InitTypeDef xsmt;
    xsmt.GPIO_Mode = GPIO_Mode_Out_PP;
    xsmt.GPIO_Pin = XSMT_PIN;
    xsmt.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &xsmt);

    GPIO_WriteBit(GPIOC, XSMT_PIN, Bit_RESET);
}

void LED_Enable(bool enable) {
    if (enable) {
        GPIO_WriteBit(GPIOC, XSMT_PIN, Bit_SET);
    }
    else {
        GPIO_WriteBit(GPIOC, XSMT_PIN, Bit_RESET);
    }
}
