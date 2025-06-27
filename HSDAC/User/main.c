#include "debug.h"
#include "codec.h"
#include <math.h>
#include "tick.h"
#include "ch32v30x_gpio.h"
#include "usb/ch32v30x_usbhs_device.h"
#include "hid_queue.h"

static void ES9018K_Init(void) {
    Codec_Init();
    GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
    Delay_Ms(100);
    GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
    Delay_Ms(100);

    // 尼玛，手册里寄存器写 #12 ,然后是0x12, cnm
    Codec_PollWrite(0, 0xf1);
    Delay_Ms(100);
    Codec_PollWrite(0, 0xf0);
    Delay_Ms(100);
    Codec_PollWrite(1, 0b10000000);
}

extern volatile uint32_t max_uac_len_ever;
extern volatile uint32_t min_uac_len_ever;
extern uint32_t mesured_dma_sample_rate_;
extern uint32_t mesured_usb_sample_rate_;
extern float report_fs_;
extern uint8_t vol_[];

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
        if ((ct - t) > 1000) {
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

        Codec_CheckVolumeEvent();

        uint32_t len;
        HID_Queue_Read(&g_hid_queue, &len);
        for (uint32_t i = 0; i < len; ++i) {
            struct HID_Event* e = HID_Queue_GetItem(&g_hid_queue, i);
            if (USBHS_DevEnumStatus) {
                printf("[hid]type: %d, reg: %d, val: %d\n\r", (int)e->type, e->reg, e->val);
            }

            if (e->type == 0) {
                // dac param setting
                Codec_PollWrite(e->reg, e->val);
                Delay_Ms(10);
            }
            else if (e->type == 1 && e->reg == 0x12 && e->val == 0x34) {
                // firmware update
                RCC_APB1PeriphClockCmd( RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE );
                BKP_TamperPinCmd( DISABLE );
                PWR_BackupAccessCmd( ENABLE );
                BKP_ClearFlag();

                BKP_WriteBackupRegister(BKP_DR1, 0x1234);
                BKP_WriteBackupRegister(BKP_DR2, 0x5678);
                Codec_DeInit();
                USBHS_Device_Init(DISABLE);
                NVIC_SystemReset();
            }
        }
        HID_Queue_Finish(&g_hid_queue, len);
    }
}
