#include "debug.h"
#include "codec.h"
#include <math.h>
#include "tick.h"
#include "ch32v30x_gpio.h"
#include "usb/ch32v30x_usbhs_device.h"

static void ES9018K_Init(void) {
    Codec_Init();
    GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
    Delay_Ms(100);
    GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
    Delay_Ms(100);

    Codec_PollWrite(0, 0xf1);
    Codec_PollWrite(1, 0b10000000);
    volatile uint8_t e = Codec_PollRead(64);
    volatile uint32_t i = 0;
    for (;;) {
        ++i;
    }
}

int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    Tick_Init();

    // ES9018K_Init();
    USBHS_RCC_Init();
    USBHS_Device_Init(ENABLE);
    for (;;) {
    }
}
