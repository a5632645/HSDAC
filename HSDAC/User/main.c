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
    Codec_PollWrite(0, 0xf0);
    Codec_PollWrite(1, 0b10000000);
}

extern volatile uint32_t max_uac_len_ever;
extern volatile uint32_t min_uac_len_ever;
extern uint32_t mesured_dma_sample_rate_;
extern uint32_t mesured_usb_sample_rate_;
extern float report_fs_;

int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    Tick_Init();

    ES9018K_Init();
    USBHS_RCC_Init();
    USBHS_Device_Init(ENABLE);

    uint32_t t = Tick_GetTick();
    uint32_t t2 = Tick_GetTick();
    for (;;) {
        uint32_t ct = Tick_GetTick();
        if ((ct - t) > 100) {
            t = ct;

            if (USBHS_DevEnumStatus) {
                printf("[fs]usb: %luHz, dma: %luHz, fb:%luHz\n\r", mesured_usb_sample_rate_, mesured_dma_sample_rate_, (uint32_t)report_fs_);
                printf("[uac]len: %lu, min: %lu, max:%lu\n\r\n\r", Codec_GetUACBufferLen(), min_uac_len_ever, max_uac_len_ever);
            }
        }

        if ((ct - t2) > 10) {
            t2 = ct;
            if (min_uac_len_ever < 2048 / 2)
                ++min_uac_len_ever;
            if (max_uac_len_ever > 2048 / 2)
                --max_uac_len_ever;
        }
    }
}
