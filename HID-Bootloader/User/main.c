#include "debug.h"
#include "usb/ch32v30x_usbhs_device.h"
#include "ch32v30x_bkp.h"
#include "ch32v30x_flash.h"
#include "ch32v30x_gpio.h"

#define APP_START (FLASH_BASE + 1024 * 8)
uint32_t flash_program_address = APP_START;
#define BOOTLOADER_PAGE (FLASH_BASE + 1024 * 8 - 256)

#define PROGRAM_READY 0x1
#define VERIFY_READY  0x2
#define PROGRAMING    0x3
#define VERIFYING     0x4
#define FINISHED      0x5

static uint32_t GetLastAppState(void) {
    return *(uint32_t*)BOOTLOADER_PAGE;
}

static uint32_t ReadFakeBootPin(void) {
    return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8);
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();

    // 初始化伪BOOT引脚
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef init;
    init.GPIO_Mode = GPIO_Mode_IPU;
    init.GPIO_Speed = GPIO_Speed_50MHz;
    init.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOC, &init);

    // 读取后备寄存器
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE );
	BKP_TamperPinCmd( DISABLE );
    PWR_BackupAccessCmd( ENABLE );
    BKP_ClearFlag();

    uint16_t magic1 = BKP_ReadBackupRegister(BKP_DR1);
    uint16_t magic2 = BKP_ReadBackupRegister(BKP_DR2);
    uint8_t state = PROGRAM_READY;
    if ((magic1 == 0x1234 && magic2 == 0x5678) || GetLastAppState() == 1 || ReadFakeBootPin() == 0) {
        USBHS_RCC_Init();
        USBHS_Device_Init(ENABLE);

        while (USBHS_DevEnumStatus == 0) {}

_download:
        state = PROGRAM_READY;
        FLASH_Unlock_Fast();
        FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);

        // 烧写
        flash_program_address = APP_START;
        while (1) {
            if (HID_HasData()) {
                uint32_t* ptr = (uint32_t*)HID_GetDataPtr();
                if (ptr[0] == 0) {
                    // erase
                    FLASH_ErasePage_Fast(flash_program_address);
                    // program
                    FLASH_ProgramPage_Fast(flash_program_address, ptr + 1);
                    flash_program_address += 256;
                }
                else if (ptr[0] == 0xffffffff) {
                    // end
                    break;
                }
                else if (ptr[0] == 0xff000000) {
                    // get current state
                    HID_ReportOK(state);
                }
                else if (ptr[0] == 0x00ff0000) {
                    // get last app state
                    HID_ReportOK(GetLastAppState());
                }
                HID_NextData();
                state = PROGRAMING;
            }
        }

        // 检测
        state = VERIFY_READY;
        HID_NextData();
        flash_program_address = APP_START;
        uint32_t bad_flash = 0;
        while (1) {
            if (HID_HasData()) {
                uint32_t* ptr = (uint32_t*)HID_GetDataPtr();
                if (ptr[0] == 0) {
                    for (uint32_t i = 0; i < 64; ++i) {
                        if (*(uint32_t*)flash_program_address != ptr[i + 1]) {
                            bad_flash = 1;
                        }
                        flash_program_address += 4;
                    }
                }
                else if (ptr[0] == 0xffffffff) {
                    // end
                    break;
                }
                else if (ptr[0] == 0xff000000) {
                    // get current state
                    HID_ReportOK(state);
                }
                else if (ptr[0] == 0x00ff0000) {
                    // get last app state
                    HID_ReportOK(GetLastAppState());
                }
                HID_NextData();
                state = VERIFYING;
            }
        }

        uint32_t last_app_state = *(uint32_t*)BOOTLOADER_PAGE;
        if (last_app_state != bad_flash) {
            FLASH_ErasePage_Fast(BOOTLOADER_PAGE);
            uint32_t wtf[64] = {
                bad_flash
            };
            FLASH_ProgramPage_Fast(BOOTLOADER_PAGE, wtf);
        }
        FLASH_Lock_Fast();

        BKP_WriteBackupRegister(BKP_DR1, 0);
        BKP_WriteBackupRegister(BKP_DR2, 0);

        HID_NextData();
        state = FINISHED;
        while (1) {
            if (HID_HasData()) {
                uint32_t* ptr = (uint32_t*)HID_GetDataPtr();
                if (ptr[0] == 0xffffffff) {
                    // end
                    break;
                }
                else if (ptr[0] == 0xff000000) {
                    // get current state
                    HID_ReportOK(state);
                }
                else if (ptr[0] == 0x00ff0000) {
                    // get last app state
                    HID_ReportOK(bad_flash);
                }
                HID_NextData();
            }
        }

        if (bad_flash == 1) {
            goto _download;
        }
    }

    GPIO_DeInit(GPIOC);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, DISABLE);
    USBHS_Device_Init(DISABLE);

    uint32_t vector_base = 1024 * 8;
    __asm volatile("jr %0": :"r"(vector_base));
}
