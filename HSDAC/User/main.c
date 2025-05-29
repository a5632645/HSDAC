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
}

extern uint32_t num_usb;
uint32_t usb_bck = 0;
uint32_t dma_bck = 0;

int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    Tick_Init();

    ES9018K_Init();
    USBHS_RCC_Init();
    USBHS_Device_Init(ENABLE);

    uint32_t i = 0;
    uint32_t t = Tick_GetTick();
    for (;;) {
        uint32_t ct = Tick_GetTick();
        if ((ct - t) > 10) {
            t = ct;

            if (USBHS_DevEnumStatus) {
                printf("%ld,", i++);
                // fflush(stdout);
            }
            // usb_bck = num_usb * 4;
            // num_usb = 0;
            // dma_bck = Codec_GetDMALen();
        }
    }
}
