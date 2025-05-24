/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v30x_usbhs_device.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2023/11/20
* Description        : This file provides all the USBHS firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v30x_usbhs_device.h"

#define MY_MIN(a, b) ((a) > (b) ? (b) : (a))
#define MY_MAX(a, b) ((a) > (b) ? (a) : (b))

/******************************************************************************/
/* Variable Definition */
/* test mode */
// volatile uint8_t  USBHS_Test_Flag;
// __attribute__ ((aligned(4))) uint8_t IFTest_Buf[ 53 ] =
// {
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
//     0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE,
//     0xFE,//26
//     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,//37
//     0x7F, 0xBF, 0xDF, 0xEF, 0xF7, 0xFB, 0xFD,//44
//     0xFC, 0x7E, 0xBF, 0xDF, 0xEF, 0xF7, 0xFB, 0xFD, 0x7E//53
// };

/* Setup Request */
struct _USBD_SetupPack {
    volatile uint8_t  bmRequestType;
    volatile uint8_t  bRequest;
    volatile uint16_t wValue;
    volatile uint16_t wIndex;
    volatile uint16_t wLength;
    const uint8_t* pConfig;
};
static struct _USBD_SetupPack usbd_setup;

/* USB Device Status */
struct _USBD_State {
    volatile uint8_t  USBHS_DevConfig;
    volatile uint8_t  USBHS_DevAddr;
    volatile uint16_t USBHS_DevMaxPackLen;
    volatile uint8_t  USBHS_DevSpeed;
    volatile uint8_t  USBHS_DevSleepStatus;
    volatile uint8_t  USBHS_DevEnumStatus;
};
static struct _USBD_State usbd_state;

/* Endpoint Buffer */
__attribute__ ((aligned(4))) uint8_t USBHS_EP0_Buf[DEF_USBD_UEP0_SIZE];
__attribute__ ((aligned(4))) uint8_t USBHS_EP1_TX_Buf[DEF_USB_EP1_HS_SIZE];
__attribute__ ((aligned(4))) uint8_t USBHS_EP2_TX_Buf[DEF_USB_EP1_HS_SIZE];
__attribute__ ((aligned(4))) uint8_t USBHS_EP2_RX_Buf[DEF_USB_EP1_HS_SIZE];

/* Endpoint tx busy flag */
volatile uint8_t  USBHS_Endp_Busy[DEF_UEP_NUM];

/******************************************************************************/
/* Interrupt Service Routine Declaration*/
void USBHS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      USB_TestMode_Deal
 *
 * @brief   Eye Diagram Test Function Processing.
 *
 * @return  none
 *
 */
// void USB_TestMode_Deal( void )
// {
//     /* start test */
//     USBHS_Test_Flag &= ~0x80;
//     if( USBHS_SetupReqIndex == 0x0100 )
//     {
//         /* Test_J */
//         USBHSD->SUSPEND &= ~TEST_MASK;
//         USBHSD->SUSPEND |= TEST_J;
//     }
//     else if( USBHS_SetupReqIndex == 0x0200 )
//     {
//         /* Test_K */
//         USBHSD->SUSPEND &= ~TEST_MASK;
//         USBHSD->SUSPEND |= TEST_K;
//     }
//     else if( USBHS_SetupReqIndex == 0x0300 )
//     {
//         /* Test_SE0_NAK */
//         USBHSD->SUSPEND &= ~TEST_MASK;
//         USBHSD->SUSPEND |= TEST_SE0;
//     }
//     else if( USBHS_SetupReqIndex == 0x0400 )
//     {
//         /* Test_Packet */
//         USBHSD->SUSPEND &= ~TEST_MASK;
//         USBHSD->SUSPEND |= TEST_PACKET;

//         USBHSD->CONTROL |= USBHS_UC_HOST_MODE;
//         USBHSH->HOST_EP_CONFIG = USBHS_UH_EP_TX_EN | USBHS_UH_EP_RX_EN;
//         USBHSH->HOST_EP_TYPE |= 0xff;

//         USBHSH->HOST_TX_DMA = (uint32_t)(&IFTest_Buf[ 0 ]);
//         USBHSH->HOST_TX_LEN = 53;
//         USBHSH->HOST_EP_PID = ( USB_PID_SETUP << 4 );
//         USBHSH->INT_FG = USBHS_UIF_TRANSFER;
//     }
// }

/*********************************************************************
 * @fn      USBHS_RCC_Init
 *
 * @brief   Initializes the clock for USB2.0 High speed device.
 *
 * @return  none
 */
void USBHS_RCC_Init( void )
{
    RCC_USBCLK48MConfig( RCC_USBCLK48MCLKSource_USBPHY );
    RCC_USBHSPLLCLKConfig( RCC_HSBHSPLLCLKSource_HSE );
    RCC_USBHSConfig( RCC_USBPLL_Div2 );
    RCC_USBHSPLLCKREFCLKConfig( RCC_USBHSPLLCKREFCLK_4M );
    RCC_USBHSPHYPLLALIVEcmd( ENABLE );
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_USBHS, ENABLE );
}

static void Halt(void) {
    volatile uint32_t i = 0;
    for (;;) {
        ++i;
    }
}

#define USB_DWORD(X) (uint8_t)(X & 0xff), (uint8_t)((X >> 8) & 0xff), (uint8_t)((X >> 16) & 0xff), (uint8_t)(X >> 24)

const uint8_t com_cfg[] = {
    USB_DWORD(115200), 1, 0, 8, 1
};

/*********************************************************************
 * @fn      USBHS_Device_Endp_Init
 *
 * @brief   Initializes USB device endpoints.
 *
 * @return  none
 */
void USBHS_Device_Endp_Init ( void )
{
    USBHSD->ENDP_CONFIG = USBHS_UEP1_T_EN | USBHS_UEP2_T_EN | USBHS_UEP2_R_EN;

    USBHSD->UEP0_MAX_LEN  = DEF_USBD_UEP0_SIZE;
    USBHSD->UEP0_DMA    = (uint32_t)(uint8_t *)USBHS_EP0_Buf;
    USBHSD->UEP0_TX_LEN  = 0;
    USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
    USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK;

    USBHSD->UEP1_MAX_LEN = sizeof(USBHS_EP1_TX_Buf);
    USBHSD->UEP1_TX_DMA = (uint32_t)&USBHS_EP1_TX_Buf[0];
    USBHSD->UEP1_TX_LEN = 0;
    USBHSD->UEP1_TX_CTRL = USBHS_UEP_T_RES_NAK; // 没有数据发送，回复NAC

    USBHSD->UEP2_MAX_LEN = sizeof(USBHS_EP2_TX_Buf);
    USBHSD->UEP2_TX_DMA = (uint32_t)&USBHS_EP2_TX_Buf[0];
    USBHSD->UEP2_TX_LEN = 0;
    USBHSD->UEP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
    USBHSD->UEP2_RX_DMA = (uint32_t)&USBHS_EP2_RX_Buf[0];
    USBHSD->UEP2_RX_CTRL = USBHS_UEP_R_RES_ACK;

    /* Clear End-points Busy Status */
    for(uint8_t i=0; i < DEF_UEP_NUM; i++)
    {
        USBHS_Endp_Busy[i] = 0;
    }
}

/*********************************************************************
 * @fn      USBHS_Device_Init
 *
 * @brief   Initializes USB high-speed device.
 *
 * @return  none
 */
void USBHS_Device_Init ( FunctionalState sta )
{
    if(sta)
    {
        USBHSD->CONTROL = USBHS_UC_CLR_ALL | USBHS_UC_RESET_SIE;
        Delay_Us(10);
        USBHSD->CONTROL &= ~USBHS_UC_RESET_SIE;
        USBHSD->HOST_CTRL = USBHS_UH_PHY_SUSPENDM;
        USBHSD->CONTROL = USBHS_UC_DMA_EN | USBHS_UC_INT_BUSY | USBHS_UC_SPEED_HIGH;
        USBHSD->INT_EN = USBHS_UIE_SETUP_ACT | USBHS_UIE_TRANSFER | USBHS_UIE_DETECT | USBHS_UIE_SUSPEND;
        USBHS_Device_Endp_Init( );
        USBHSD->CONTROL |= USBHS_UC_DEV_PU_EN;
        NVIC_EnableIRQ(USBHS_IRQn);
    }
    else
    {
        USBHSD->CONTROL = USBHS_UC_CLR_ALL | USBHS_UC_RESET_SIE;
        Delay_Us(10);
        USBHSD->CONTROL &= ~USBHS_UC_RESET_SIE;
        NVIC_DisableIRQ(USBHS_IRQn);
    }
}

/*********************************************************************
 * @fn      USBHS_IRQHandler
 *
 * @brief   This function handles USBHS exception.
 *
 * @return  none
 */
void USBHS_IRQHandler( void )
{
    volatile uint8_t intflag = USBHSD->INT_FG;
    volatile uint8_t intst = USBHSD->INT_ST;

    if (intflag & USBHS_UIF_SETUP_ACT) {
        {
            const USB_SETUP_REQ* setup_pack = (USB_SETUP_REQ*)USBHS_EP0_Buf;
            usbd_setup.bmRequestType = setup_pack->bmRequestType;
            usbd_setup.bRequest = setup_pack->bRequest;
            usbd_setup.wIndex = setup_pack->wIndex;
            usbd_setup.wValue = setup_pack->wValue;
            usbd_setup.wLength = setup_pack->wLength;
        }

        if (usbd_setup.bmRequestType & 0x80) { // input
            volatile uint8_t requestType = (usbd_setup.bmRequestType & 0x60) >> 5; // 请求类型
            volatile uint8_t requestReciver = usbd_setup.bmRequestType & 0x1f;     // 请求接收者
            switch (requestType) {
            case 0: // 标准请求
                switch (usbd_setup.bRequest) {
                case USB_GET_DESCRIPTOR:
                    switch (usbd_setup.wValue & 0xff00) { // 描述符的类型
                    case 0x100: // 设备描述符
                        usbd_setup.pConfig = MyDevDescr;
                        usbd_setup.wLength = MY_MIN(usbd_setup.wLength, DEF_USBD_DEVICE_DESC_LEN);
                        break;

                    case 0x200: // 配置描述符
                        usbd_setup.pConfig = MyCfgDescr_HS;
                        usbd_setup.wLength = MY_MIN(usbd_setup.wLength, DEF_USBD_CONFIG_HS_DESC_LEN);
                        break;

                    case 0x300: // 字符串描述符
                        switch (usbd_setup.wValue & 0xff) {
                        case 0: // LANG ID
                            usbd_setup.pConfig = MyLangDescr;
                            usbd_setup.wLength = MY_MIN(usbd_setup.wLength, DEF_USBD_LANG_DESC_LEN);
                            break;

                        case 1:
                            usbd_setup.pConfig = MyManuInfo;
                            usbd_setup.wLength = MY_MIN(usbd_setup.wLength, DEF_USBD_MANU_DESC_LEN);
                            break;

                        case 2:
                            usbd_setup.pConfig = MyProdInfo;
                            usbd_setup.wLength = MY_MIN(usbd_setup.wLength, DEF_USBD_PROD_DESC_LEN);
                            break;

                        case 3:
                            usbd_setup.pConfig = MySerNumInfo;
                            usbd_setup.wLength = MY_MIN(usbd_setup.wLength, DEF_USBD_SN_DESC_LEN);
                            break;

                        default:
                            usbd_setup.pConfig = NULL;
                            usbd_setup.wLength = MY_MIN(usbd_setup.wLength, 0);
                            break;
                        } // switch: 字符串描述符
                        break;

                    case 0x400: // 接口描述符
                        Halt();
                        break;

                    case 0x500: // 端点描述符
                        Halt();
                        break;
                    } // switch: 读描述符
                    memcpy(USBHS_EP0_Buf, usbd_setup.pConfig, usbd_setup.wLength);
                    break;

                case USB_GET_CONFIGURATION:
                    USBHS_EP0_Buf[0] = usbd_state.USBHS_DevConfig;
                    usbd_setup.wLength = MY_MIN(usbd_setup.wLength, 1);
                    break;

                case USB_GET_INTERFACE:
                    USBHS_EP0_Buf[0] = 0;
                    usbd_setup.wLength = MY_MIN(usbd_setup.wLength, 1);
                    break;

                case USB_GET_STATUS:
                    Halt();
                    break;

                case USB_SYNCH_FRAME:
                    Halt();
                    break;

                default:
                    Halt();
                    break;
                } // switch: 标准请求
                break;

            case 1: // 类请求
                switch (usbd_setup.bRequest) {
                case CDC_GET_LINE_CODING:
                    usbd_setup.pConfig = &com_cfg[0];
                    usbd_setup.wLength = MY_MIN(usbd_setup.wLength, 8);
                    break;

                default:
                    Halt();
                    break;
                }
                break;

            case 2: // 厂商请求
                Halt();
                break;

            case 3: // 保留
                Halt();
                break;
            } // switch: 请求类型

            // 设置下一事务的发送, setup后必须跟DATA1
            uint16_t tx_len = MY_MIN(usbd_setup.wLength, sizeof(USBHS_EP0_Buf));
            USBHSD->UEP0_TX_LEN = tx_len;
            usbd_setup.wLength -= tx_len;
            usbd_setup.pConfig += tx_len;
            USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
        } // if: input setup
        else { // output
            volatile uint8_t requestType = (usbd_setup.bmRequestType & 0x60) >> 5; // 请求类型
            volatile uint8_t requestReciver = usbd_setup.bmRequestType & 0x1f;     // 请求接收者
            switch (requestType) {
            case 0: // 标准请求
                switch (usbd_setup.bRequest) {
                case USB_CLEAR_FEATURE:
                    Halt();
                    break;

                case USB_SET_FEATURE:
                    Halt();
                    break;

                case USB_SET_ADDRESS:
                    usbd_state.USBHS_DevAddr = usbd_setup.wValue;
                    break;

                case USB_SET_DESCRIPTOR:
                    Halt();
                    break;

                case USB_SET_CONFIGURATION:
                    usbd_state.USBHS_DevConfig = usbd_setup.wValue & 0xff;
                    usbd_state.USBHS_DevEnumStatus = 1;
                    break;

                case USB_SET_INTERFACE:
                    Halt();
                    break;

                default:
                    Halt();
                    break;
                } // switch: 标准请求
                break;

            case 1: // 类请求
                switch (usbd_setup.bRequest) {
                case CDC_SET_LINE_CODING:
                    break;

                case CDC_SET_LINE_CTLSTE:
                    break;

                case CDC_SEND_BREAK:
                    break;

                default:
                    break;
                } // switch: class request
                break;

            case 2: // 厂商请求
                Halt();
                break;

            case 3: // 保留
                Halt();
                break;
            } // switch: 请求类型

            if (usbd_setup.wLength == 0) { // 没有数据阶段，状态阶段需要发DATA1的空IN/TX
                USBHSD->UEP0_TX_LEN = 0;
                USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
            }
            else { // 有数据阶段，准备DATA1的OUT/RX
                USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
            }
        } // else: output setup

        USBHSD->INT_FG = USBHS_UIF_SETUP_ACT;
    } // if: setup int
    else if (intflag & USBHS_UIF_TRANSFER) {
        switch (intst & USBHS_UIS_TOKEN_MASK) {
        case USBHS_UIS_TOKEN_IN:    // 上一次事务是IN
            switch (intst & 0x0f) { // 输入端点号
            case 0:
                if (usbd_setup.wLength == 0) { // 状态流程，接收0字节长的DATA1
                    USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
                }

                switch (usbd_setup.bRequest) {
                case USB_SET_ADDRESS:
                    USBHSD->DEV_AD = usbd_state.USBHS_DevAddr;
                    break;

                case USB_GET_DESCRIPTOR: {
                    uint16_t tx_len = MY_MIN(usbd_setup.wLength, sizeof(USBHS_EP0_Buf));
                    memcpy(USBHS_EP0_Buf, usbd_setup.pConfig, tx_len);
                    usbd_setup.wLength -= tx_len;
                    usbd_setup.pConfig += tx_len;
                    USBHSD->UEP0_TX_LEN = tx_len;
                    USBHSD->UEP0_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
                }
                    break;

                default:
                    USBHSD->UEP0_TX_LEN = 0;
                    break;
                }
                break;

            case 2: // CDC批量读
                USBHSD->UEP2_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
                USBHSD->UEP2_TX_CTRL &= ~USBHS_UEP_T_RES_MASK;
                USBHSD->UEP2_TX_CTRL |= USBHS_UEP_T_RES_NAK;
                break;

            case 1: // CDC中断读
                Halt();
                break;

            default:
                Halt();
                break;
            }
            break;

        case USBHS_UIS_TOKEN_OUT: {
            uint16_t rx_len = USBHSD->RX_LEN;
            switch (intst & 0x0f) { // 端口号
            case 0:
                if (usbd_setup.wLength == 0) {
                    USBHSD->UEP0_TX_LEN = 0;
                    USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
                }
                break;

            case 2: // CDC批量写
                USBHSD->UEP1_RX_CTRL ^= USBHS_UEP_R_TOG_DATA1;
                memcpy(USBHS_EP2_TX_Buf, USBHS_EP2_RX_Buf, rx_len);
                USBHSD->UEP2_RX_CTRL &= ~USBHS_UEP_R_RES_MASK;
                USBHSD->UEP2_RX_CTRL |= USBHS_UEP_R_RES_ACK;

                USBHSD->UEP2_TX_LEN = rx_len;
                USBHSD->UEP2_TX_CTRL &= ~USBHS_UEP_T_RES_MASK;
                USBHSD->UEP2_TX_CTRL |= USBHS_UEP_T_RES_ACK;
                break;

            default:
                Halt();
                break;
            }
        }
            break;

        case USBHS_UIS_TOKEN_SETUP:
            break;

        case USBHS_UIS_TOKEN_SOF:
            break;
        } // switch: transfer token

        USBHSD->INT_FG = USBHS_UIF_TRANSFER;
    }
    else if (intflag & USBHS_UIF_BUS_RST) {
        usbd_state.USBHS_DevAddr = 0;
        usbd_state.USBHS_DevConfig = 0;
        usbd_state.USBHS_DevEnumStatus = 0;
        usbd_state.USBHS_DevSleepStatus = 0;
        usbd_state.USBHS_DevSpeed = 0;
        usbd_state.USBHS_DevMaxPackLen = 0;
        USBHSD->DEV_AD = 0;
        USBHS_Device_Endp_Init();

        USBHSD->INT_FG = USBHS_UIF_BUS_RST;
    }
    else if (intflag & USBHS_UIF_SUSPEND) {
        USBHSD->INT_FG = USBHS_UIF_SUSPEND;
    }
    else {
        USBHSD->INT_FG = intflag;
    }
}
