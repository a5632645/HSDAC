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
#include "codec.h"
#include "tick.h"

/******************************************************************************/
/* Variable Definition */
/* test mode */
volatile uint8_t USBHS_Test_Flag;
__attribute__ ((aligned (4))) uint8_t IFTest_Buf[53] =
    {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
        0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE,
        0xFE,                                                              // 26
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  // 37
        0x7F, 0xBF, 0xDF, 0xEF, 0xF7, 0xFB, 0xFD,                          // 44
        0xFC, 0x7E, 0xBF, 0xDF, 0xEF, 0xF7, 0xFB, 0xFD, 0x7E               // 53
};

/* Global */
const uint8_t *pUSBHS_Descr;

/* Setup Request */
volatile uint8_t USBHS_SetupReqCode;
volatile uint8_t USBHS_SetupReqType;
volatile uint16_t USBHS_SetupReqValue;
volatile uint16_t USBHS_SetupReqIndex;
volatile uint16_t USBHS_SetupReqLen;

/* USB Device Status */
volatile uint8_t USBHS_DevConfig;
volatile uint8_t USBHS_DevAddr;
volatile uint16_t USBHS_DevMaxPackLen;
volatile uint8_t USBHS_DevSpeed;
volatile uint8_t USBHS_DevSleepStatus;
volatile uint8_t USBHS_DevEnumStatus;

/* Endpoint Buffer */
__attribute__ ((aligned (4))) uint8_t USBHS_EP0_Buf[DEF_USBD_UEP0_SIZE];
__attribute__ ((aligned (4))) uint8_t USBHS_EP1_Rx_Buf[DEF_USB_EP1_HS_SIZE];
__attribute__ ((aligned (4))) uint8_t USBHS_EP3_Tx_Buf[DEF_USB_EP3_HS_SIZE];
__attribute__ ((aligned (4))) uint8_t USBHS_EP2_Tx_Buf[DEF_USB_EP2_HS_SIZE];
__attribute__ ((aligned (4))) uint8_t USBHS_EP2_Rx_Buf[DEF_USB_EP2_HS_SIZE];

/* Endpoint tx busy flag */
volatile uint8_t USBHS_Endp_Busy[DEF_UEP_NUM];

/******************************************************************************/
/* Interrupt Service Routine Declaration*/
void USBHS_IRQHandler (void) __attribute__ ((interrupt ("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      USB_TestMode_Deal
 *
 * @brief   Eye Diagram Test Function Processing.
 *
 * @return  none
 *
 */
void USB_TestMode_Deal (void) {
    /* start test */
    USBHS_Test_Flag &= ~0x80;
    if (USBHS_SetupReqIndex == 0x0100) {
        /* Test_J */
        USBHSD->SUSPEND &= ~TEST_MASK;
        USBHSD->SUSPEND |= TEST_J;
    } else if (USBHS_SetupReqIndex == 0x0200) {
        /* Test_K */
        USBHSD->SUSPEND &= ~TEST_MASK;
        USBHSD->SUSPEND |= TEST_K;
    } else if (USBHS_SetupReqIndex == 0x0300) {
        /* Test_SE0_NAK */
        USBHSD->SUSPEND &= ~TEST_MASK;
        USBHSD->SUSPEND |= TEST_SE0;
    } else if (USBHS_SetupReqIndex == 0x0400) {
        /* Test_Packet */
        USBHSD->SUSPEND &= ~TEST_MASK;
        USBHSD->SUSPEND |= TEST_PACKET;

        USBHSD->CONTROL |= USBHS_UC_HOST_MODE;
        USBHSH->HOST_EP_CONFIG = USBHS_UH_EP_TX_EN | USBHS_UH_EP_RX_EN;
        USBHSH->HOST_EP_TYPE |= 0xff;

        USBHSH->HOST_TX_DMA = (uint32_t)(&IFTest_Buf[0]);
        USBHSH->HOST_TX_LEN = 53;
        USBHSH->HOST_EP_PID = (USB_PID_SETUP << 4);
        USBHSH->INT_FG = USBHS_UIF_TRANSFER;
    }
}

/*********************************************************************
 * @fn      USBHS_RCC_Init
 *
 * @brief   Initializes the clock for USB2.0 High speed device.
 *
 * @return  none
 */
void USBHS_RCC_Init (void) {
    RCC_USBCLK48MConfig (RCC_USBCLK48MCLKSource_USBPHY);
    RCC_USBHSPLLCLKConfig (RCC_HSBHSPLLCLKSource_HSE);
    RCC_USBHSConfig (RCC_USBPLL_Div2);
    RCC_USBHSPLLCKREFCLKConfig (RCC_USBHSPLLCKREFCLK_4M);
    RCC_USBHSPHYPLLALIVEcmd (ENABLE);
    RCC_AHBPeriphClockCmd (RCC_AHBPeriph_USBHS, ENABLE);
}

/*********************************************************************
 * @fn      USBHS_Device_Endp_Init
 *
 * @brief   Initializes USB device endpoints.
 *
 * @return  none
 */
void USBHS_Device_Endp_Init (void) {

    USBHSD->ENDP_CONFIG = USBHS_UEP1_R_EN 
                        | USBHS_UEP2_T_EN | USBHS_UEP2_R_EN
                        | USBHS_UEP3_T_EN;
    USBHSD->ENDP_TYPE = USBHS_UEP1_R_TYPE;

    USBHSD->UEP0_MAX_LEN = DEF_USBD_UEP0_SIZE;
    USBHSD->UEP1_MAX_LEN = DEF_USB_EP1_HS_SIZE;
    USBHSD->UEP2_MAX_LEN = DEF_USB_EP2_HS_SIZE;
    USBHSD->UEP3_MAX_LEN = DEF_USB_EP3_HS_SIZE;

    USBHSD->UEP0_DMA = (uint32_t)(uint8_t*)USBHS_EP0_Buf;
    USBHSD->UEP1_RX_DMA = (uint32_t)(uint8_t*)USBHS_EP1_Rx_Buf;
    USBHSD->UEP2_TX_DMA = (uint32_t)(uint8_t*)USBHS_EP2_Tx_Buf;
    USBHSD->UEP2_RX_DMA = (uint32_t)(uint8_t*)USBHS_EP2_Rx_Buf;
    USBHSD->UEP3_TX_DMA = (uint32_t)(uint8_t*)USBHS_EP3_Tx_Buf;

    USBHSD->UEP0_TX_LEN = 0;
    USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_ACK;
    USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK;

    USBHSD->UEP1_TX_LEN = 4;
    USBHSD->UEP1_TX_CTRL = USBHS_UEP_T_TOG_DATA0;
    USBHSD->UEP1_RX_CTRL = USBHS_UEP_R_TOG_DATA0;

    USBHSD->UEP2_TX_LEN = 0;
    USBHSD->UEP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
    USBHSD->UEP2_RX_CTRL = USBHS_UEP_R_RES_ACK;

    USBHSD->UEP3_TX_LEN = 0;
    USBHSD->UEP3_TX_CTRL = USBHS_UEP_T_RES_NAK;

    /* Clear End-points Busy Status */
    for (uint8_t i = 0; i < DEF_UEP_NUM; i++) {
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
void USBHS_Device_Init (FunctionalState sta) {
    if (sta) {
        USBHSD->CONTROL = USBHS_UC_CLR_ALL | USBHS_UC_RESET_SIE;
        Delay_Us (10);
        USBHSD->CONTROL &= ~USBHS_UC_RESET_SIE;
        USBHSD->HOST_CTRL = USBHS_UH_PHY_SUSPENDM;
        USBHSD->CONTROL = USBHS_UC_DMA_EN | USBHS_UC_INT_BUSY | USBHS_UC_SPEED_HIGH;
        USBHSD->INT_EN = USBHS_UIE_SETUP_ACT | USBHS_UIE_TRANSFER | USBHS_UIE_DETECT | USBHS_UIE_SUSPEND | USBHS_UIE_SOF_ACT;
        USBHS_Device_Endp_Init();
        USBHSD->CONTROL |= USBHS_UC_DEV_PU_EN;

        NVIC_InitTypeDef nvic = {
            .NVIC_IRQChannel = USBHS_IRQn,
            .NVIC_IRQChannelCmd = ENABLE,
            .NVIC_IRQChannelPreemptionPriority = 0,
            .NVIC_IRQChannelSubPriority = 0
        };
        NVIC_Init(&nvic);
    } else {
        USBHSD->CONTROL = USBHS_UC_CLR_ALL | USBHS_UC_RESET_SIE;
        Delay_Us (10);
        USBHSD->CONTROL &= ~USBHS_UC_RESET_SIE;
        NVIC_DisableIRQ (USBHS_IRQn);
    }
}

/*********************************************************************
 * @fn      USBHS_Endp_DataUp
 *
 * @brief   usbhs device data upload
 *          input: endp  - end-point numbers
 *                 *pubf - data buffer
 *                 len   - load data length
 *                 mod   - 0: DEF_UEP_DMA_LOAD 1: DEF_UEP_CPY_LOAD
 *
 * @return  none
 */
uint8_t USBHS_Endp_DataUp (uint8_t endp, uint8_t *pbuf, uint16_t len, uint8_t mod) {
    uint8_t endp_buf_mode, endp_en, endp_tx_ctrl;

    /* DMA config, endp_ctrl config, endp_len config */
    if ((endp >= DEF_UEP1) && (endp <= DEF_UEP15)) {
        endp_en = USBHSD->ENDP_CONFIG;
        if (endp_en & USBHSD_UEP_TX_EN (endp)) {
            if ((USBHS_Endp_Busy[endp] & DEF_UEP_BUSY) == 0x00) {
                endp_buf_mode = USBHSD->BUF_MODE;
                /* if end-point buffer mode is double buffer */
                if (endp_buf_mode & USBHSD_UEP_DOUBLE_BUF (endp)) {
                    /* end-point buffer mode is double buffer */
                    /* only end-point tx enable  */
                    if ((endp_en & USBHSD_UEP_RX_EN (endp)) == 0x00) {
                        endp_tx_ctrl = USBHSD_UEP_TXCTRL (endp);
                        if (mod == DEF_UEP_DMA_LOAD) {
                            if ((endp_tx_ctrl & USBHS_UEP_T_TOG_DATA1) == 0) {
                                /* use UEPn_TX_DMA */
                                USBHSD_UEP_TXDMA (endp) = (uint32_t)pbuf;
                            } else {
                                /* use UEPn_RX_DMA */
                                USBHSD_UEP_RXDMA (endp) = (uint32_t)pbuf;
                            }
                        } else if (mod == DEF_UEP_CPY_LOAD) {
                            if ((endp_tx_ctrl & USBHS_UEP_T_TOG_DATA1) == 0) {
                                /* use UEPn_TX_DMA */
                                memcpy (USBHSD_UEP_TXBUF (endp), pbuf, len);
                            } else {
                                /* use UEPn_RX_DMA */
                                memcpy (USBHSD_UEP_RXBUF (endp), pbuf, len);
                            }
                        } else {
                            return 1;
                        }
                    } else {
                        return 1;
                    }
                } else {
                    /* end-point buffer mode is single buffer */
                    if (mod == DEF_UEP_DMA_LOAD) {

                        USBHSD_UEP_TXDMA (endp) = (uint32_t)pbuf;
                    } else if (mod == DEF_UEP_CPY_LOAD) {
                        memcpy (USBHSD_UEP_TXBUF (endp), pbuf, len);
                    } else {
                        return 1;
                    }
                }
                /* Set end-point busy */
                USBHS_Endp_Busy[endp] |= DEF_UEP_BUSY;
                /* end-point n response tx ack */
                USBHSD_UEP_TLEN (endp) = len;
                USBHSD_UEP_TXCTRL (endp) = (USBHSD_UEP_TXCTRL (endp) &= ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
            } else {
                return 1;
            }
        } else {
            return 1;
        }
    } else {
        return 1;
    }

    return 0;
}

#define errflag _errflag
#define notsupport _errflag = 0xff
#define ssupport _errflag = 0

enum {
    UAC_CLASS_REQ_RECIVER_DEVICE = 0,
    UAC_CLASS_REQ_RECIVER_INTERFACE,
    UAC_CLASS_REQ_RECIVER_ENDPOINT,
    UAC_CLASS_REQ_RECIVER_OTHER,
};

enum {
    UAC_CLOCK_SOURCE_CONTROL_SELECT_UNDEFIED = 0,
    UAC_CLOCK_SOURCE_CONTROL_SELECT_SAMLPE_RATE,
    UAC_CLOCK_SOURCE_CONTROL_SELECT_CLOCK_VALID,
};

static const uint8_t com_cfg[] = {
    USB_DWORD(115200), 1, 0, 8, 1
};

static uint16_t len = 0;
static uint32_t sample_rate = 48000;
static uint8_t audio_stream_interface_work = 0;
static volatile uint8_t channel_mutes_ = 0;
static volatile int16_t channel_volumes_[3];
// uac setup,set something or send something to host
static uint8_t USBHS_UAC_Setup(uint8_t _errflag) {
    switch (USBHS_SetupReqType & USB_REQ_TYP_MASK) {
    case USB_REQ_TYP_CLASS: // 类请求
        switch (USBHS_SetupReqType & 0x1f) {
        case UAC_CLASS_REQ_RECIVER_DEVICE:
            errflag = 0xff;
            break;
        case UAC_CLASS_REQ_RECIVER_INTERFACE: {
            if (USBHS_SetupReqCode == CDC_SET_LINE_CODING) {
                ssupport;
            }
            else if (USBHS_SetupReqCode == CDC_SET_LINE_CTLSTE) {
                ssupport;
            }
            else if (USBHS_SetupReqCode == CDC_GET_LINE_CODING) {
                memcpy(USBHS_EP0_Buf, com_cfg, sizeof(com_cfg));
                len = 7;
            }
            else if ((USBHS_SetupReqIndex & 0xff) == 0) {
                // audio control
                switch (USBHS_SetupReqIndex >> 8) {
                case 0x04: { // feature unit
                    uint8_t channel = USBHS_SetupReqValue & 0xff;
                    switch (USBHS_SetupReqValue >> 8) {
                    case 0x01: // mute
                        if (USBHS_SetupReqType & 0x80) {
                            // get
                            USBHS_EP0_Buf[0] = (channel_mutes_ & (1 << channel)) ? 1 : 0;
                            len = 1;
                        }
                        else {
                            // set
                            ssupport;
                        }
                        break;
                    case 0x02:
                        // volume
                        switch (USBHS_SetupReqCode) {
                        case 0x1:
                            // curr
                            if (USBHS_SetupReqType & 0x80) {
                                // get
                                *(int16_t*)USBHS_EP0_Buf = channel_volumes_[channel];
                                len = 2;
                            }
                            else {
                                // set
                                ssupport;
                            }
                            break;
                        case 0x2:
                            // range
                            if (USBHS_SetupReqType & 0x80) {
                                // get
                                int16_t* p = (int16_t*)USBHS_EP0_Buf;
                                p[0] = 1;
                                p[1] = -32767;
                                p[2] = 0;
                                p[3] = 128;
                                len = 8;
                            }
                            else {
                                // set
                                notsupport;
                            }
                            break;
                        default:
                            notsupport;
                            break;
                        }
                        break;
                    default:
                        notsupport;
                        break;
                    }
                }
                break;
                case 0x3: { // clock source
                    switch (USBHS_SetupReqValue >> 8) {
                    case UAC_CLOCK_SOURCE_CONTROL_SELECT_SAMLPE_RATE: {
                        switch (USBHS_SetupReqCode) {
                        case 0x1: // curr
                            if (USBHS_SetupReqType & 0x80) {
                                // get
                                *(uint32_t*)USBHS_EP0_Buf = sample_rate;
                                len = 4;
                            }
                            else {
                                // set
                                ssupport;
                            }
                            break;
                        case 0x2: // range
                            if (USBHS_SetupReqType & 0x80) {
                                // get
                                static const uint32_t fs_array[] = {
                                    48000, 48000, 0,
                                    96000, 96000, 0,
                                    192000, 192000, 0,
                                };
                                *(uint16_t*)USBHS_EP0_Buf = sizeof(fs_array) / (sizeof(uint32_t) * 3);
                                memcpy(USBHS_EP0_Buf + 2, fs_array, sizeof(fs_array));
                                len = sizeof(fs_array) + 2;
                            }
                            else {
                                // set
                                notsupport;
                            }
                            break;
                        default:
                            errflag = 0xff;
                            break;
                        }
                    }
                        break;
                    case UAC_CLOCK_SOURCE_CONTROL_SELECT_CLOCK_VALID:
                        errflag = 0xff;
                        break;
                    default:
                        errflag = 0xff; 
                        break;
                    }
                }
                    break;
                default:
                    errflag = 0xff;
                    break;
                }
            }
            else if ((USBHS_SetupReqIndex & 0xff) == 1) {
                errflag = 0xff;
                // ssupport;
            }
        }
            break;
        case UAC_CLASS_REQ_RECIVER_ENDPOINT:
            errflag = 0xff;
            break;
        case UAC_CLASS_REQ_RECIVER_OTHER:
            errflag = 0xff;
            break;
        }
        break;
    default:
        errflag = 0xff;
        break;
    }
    return _errflag;
}

// uac set somethins
static uint8_t USBHS_UAC_RX(uint8_t _errflag) {
    switch (USBHS_SetupReqType & USB_REQ_TYP_MASK) {
    case USB_REQ_TYP_CLASS:
        switch (USBHS_SetupReqType & 0x1f) {
        case UAC_CLASS_REQ_RECIVER_DEVICE:
            errflag = 0xff;
            break;
        case UAC_CLASS_REQ_RECIVER_INTERFACE: {
            if (USBHS_SetupReqCode == CDC_SET_LINE_CODING) {
                ssupport;
            }
            else if (USBHS_SetupReqCode == CDC_SET_LINE_CTLSTE) {
                ssupport;
            }
            else if (USBHS_SetupReqCode == CDC_GET_LINE_CODING) {
                ssupport;
            }
            else if ((USBHS_SetupReqIndex & 0xff) == 0) {
                // audio control
                switch (USBHS_SetupReqIndex >> 8) {
                case 0x04: { // feature unit
                    uint8_t channel = USBHS_SetupReqValue & 0xff;
                    switch (USBHS_SetupReqValue >> 8) {
                    case 0x01: // mute
                        if (USBHS_SetupReqType & 0x80) {
                            // get
                        }
                        else {
                            // set
                            if (USBHS_EP0_Buf[0]) {
                                channel_mutes_ |= (1 << channel);
                            }
                            else {
                                channel_mutes_ &= ~(1 << channel);
                            }
                        }
                        break;
                    case 0x02:
                        // volume
                        switch (USBHS_SetupReqCode) {
                        case 0x1:
                            // curr
                            if (USBHS_SetupReqType & 0x80) {
                                // get
                            }
                            else {
                                // set
                                channel_volumes_[channel] = *(int16_t*)USBHS_EP0_Buf;
                                uint8_t vol = 0;
                                if (channel_volumes_[channel] == -32768) {
                                    vol = 0xff;
                                }
                                else {
                                    vol = (uint16_t)(-channel_volumes_[channel]) >> 7;
                                }
                                if (channel == 0) {
                                    Codec_SetVolume(eCodecChannel_Left, vol);
                                    Codec_SetVolume(eCodecChannel_Right, vol);
                                }
                                else if (channel == 1) {
                                    Codec_SetVolume(eCodecChannel_Left, vol);
                                }
                                else if (channel == 2) {
                                    Codec_SetVolume(eCodecChannel_Right, vol);
                                }
                            }
                            break;
                        case 0x2:
                            // range
                            if (USBHS_SetupReqType & 0x80) {
                                // get
                            }
                            else {
                                // set
                                notsupport;
                            }
                            break;
                        default:
                            notsupport;
                            break;
                        }
                        break;
                    default:
                        notsupport;
                        break;
                    }
                }
                break;
                
                case 0x3: { // clock source
                    switch (USBHS_SetupReqValue >> 8) {
                    case UAC_CLOCK_SOURCE_CONTROL_SELECT_SAMLPE_RATE: {
                        switch (USBHS_SetupReqCode) {
                        case 0x1: // curr
                            if (USBHS_SetupReqType & 0x80) {
                                // get
                                ssupport;
                            }
                            else {
                                // set
                                volatile uint32_t fs = *(uint32_t*)USBHS_EP0_Buf;
                                sample_rate = fs;
                                Codec_SetSampleRate(sample_rate);
                                ssupport;
                            }
                            break;
                        case 0x2: // range
                            if (USBHS_SetupReqType & 0x80) {
                                // get
                                ssupport;
                            }
                            else {
                                // set
                                notsupport;
                            }
                            break;
                        default:
                            errflag = 0xff;
                            break;
                        }
                    }
                        break;
                    case UAC_CLOCK_SOURCE_CONTROL_SELECT_CLOCK_VALID:
                        errflag = 0xff;
                        break;
                    default:
                        errflag = 0xff; 
                        break;
                    }
                }
                    break;
                default:
                    errflag = 0xff;
                    break;
                }
            }
            else if ((USBHS_SetupReqIndex & 0xff) == 1) {
                errflag = 0xff;
            }
        }
            break;
        case UAC_CLASS_REQ_RECIVER_ENDPOINT:
            errflag = 0xff;
            break;
        case UAC_CLASS_REQ_RECIVER_OTHER:
            errflag = 0xff;
            break;
        }
        break;
    default:
        errflag = 0xFF;
        break;
    }
    return _errflag;
}

static volatile uint32_t cdc_tx_len_ = 0;
static volatile uint32_t cdc_tx_ptr_ = 0;
static volatile bool cdc_tx_cplt = true;
static volatile bool has_zero_package = false;
uint32_t USBCDC_Write(const char* buf, uint32_t len) {
    cdc_tx_ptr_ = (uint32_t)buf;
    cdc_tx_len_ = len;
    cdc_tx_cplt = false;
    if (len % DEF_USB_EP2_HS_SIZE == 0) has_zero_package = true;
    uint32_t tx_len = len > DEF_USB_EP2_HS_SIZE ? DEF_USB_EP2_HS_SIZE : len;
    cdc_tx_ptr_ += tx_len;
    cdc_tx_len_ -= tx_len;
    USBHSD->UEP2_TX_LEN = tx_len;
    USBHSD->UEP2_TX_DMA = (uint32_t)buf;
    USBHSD->UEP2_TX_CTRL &= ~USBHS_UEP_T_RES_MASK;
    USBHSD->UEP2_TX_CTRL |= USBHS_UEP_T_RES_ACK;

    uint32_t start = Tick_GetTick();
    while (!cdc_tx_cplt && (Tick_GetTick() - start) < 10) {}

    return len;
}

void USBUAC_WriteFeedback(float local_fs) {
    if (audio_stream_interface_work == 0) return;
    // turn sample rate into data_rate per micro frame
    // 4(unused) 12.13 3(unused)
    local_fs /= 8000.0f;
    uint16_t intergal = (uint16_t)local_fs;
    local_fs -= intergal;
    uint16_t fraction = (uint16_t)(local_fs * 65536);
    USBHS_EP4_Tx_Buf[0] = fraction & 0xff;
    USBHS_EP4_Tx_Buf[1] = (fraction >> 8) & 0xff;
    USBHS_EP4_Tx_Buf[2] = intergal & 0xff;
    USBHS_EP4_Tx_Buf[3] = (intergal >> 8) & 0xff;
}

/*********************************************************************
 * @fn      USBHS_IRQHandler
 *
 * @brief   This function handles USBHS exception.
 *
 * @return  none
 */
void USBHS_IRQHandler (void) {
    len = 0;
    uint8_t intflag = USBHSD->INT_FG;
    uint8_t intst = USBHSD->INT_ST;
    uint8_t _errflag = 0;

    if (intflag & USBHS_UIF_TRANSFER) {
        switch (intst & USBHS_UIS_TOKEN_MASK) {
        /* data-in stage processing */
        case USBHS_UIS_TOKEN_IN:
            switch (intst & (USBHS_UIS_TOKEN_MASK | USBHS_UIS_ENDP_MASK)) {
            /* end-point 0 data in interrupt */
            case USBHS_UIS_TOKEN_IN | DEF_UEP0:
                if (USBHS_SetupReqLen == 0) {
                    USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
                }
                if ((USBHS_SetupReqType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) {
                    /* Non-standard request endpoint 0 Data upload */
                } else {
                    /* Standard request endpoint 0 Data upload */
                    switch (USBHS_SetupReqCode) {
                    case USB_GET_DESCRIPTOR:
                        len = USBHS_SetupReqLen >= DEF_USBD_UEP0_SIZE ? DEF_USBD_UEP0_SIZE : USBHS_SetupReqLen;
                        memcpy (USBHS_EP0_Buf, pUSBHS_Descr, len);
                        USBHS_SetupReqLen -= len;
                        pUSBHS_Descr += len;
                        USBHSD->UEP0_TX_LEN = len;
                        USBHSD->UEP0_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
                        break;

                    case USB_SET_ADDRESS:
                        USBHSD->DEV_AD = USBHS_DevAddr;
                        break;

                    default:
                        USBHSD->UEP0_TX_LEN = 0;
                        break;
                    }
                }

                /* test mode */
                if (USBHS_Test_Flag & 0x80) {
                    USB_TestMode_Deal();
                }
                break;

            // UAC反馈传输完成
            case USBHS_UIS_TOKEN_IN | DEF_UEP1:
                break;

            // CDC中断上传状态更改
            case USBHS_UIS_TOKEN_IN | DEF_UEP3:
                USBHSD->UEP3_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
                USBHSD->UEP3_TX_CTRL &= ~USBHS_UEP_T_RES_MASK;
                USBHSD->UEP3_TX_CTRL |= USBHS_UEP_T_RES_NAK;
                break;

            // CDC TX
            case USBHS_UIS_TOKEN_IN | DEF_UEP2:
            if (intst & USBHS_UIS_TOG_OK) {
                USBHSD->UEP2_TX_CTRL = (USBHSD->UEP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
                USBHSD->UEP2_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
                if (!cdc_tx_cplt) {
                    uint32_t tx_len = cdc_tx_len_ > DEF_USB_EP2_HS_SIZE ? DEF_USB_EP2_HS_SIZE : cdc_tx_len_;
                    USBHSD->UEP2_TX_LEN = tx_len;
                    USBHSD->UEP2_TX_DMA = cdc_tx_ptr_;
                    cdc_tx_ptr_ += tx_len;
                    cdc_tx_len_ -= tx_len;
                    if (tx_len == 0 && !has_zero_package) {
                        cdc_tx_cplt = true;
                    }
                    else {
                        if (tx_len == 0 && has_zero_package) {
                            has_zero_package = false;
                        }
                        USBHSD->UEP2_TX_CTRL &= ~USBHS_UEP_T_RES_MASK;
                        USBHSD->UEP2_TX_CTRL |= USBHS_UEP_T_RES_ACK;
                    }
                }
            }
                break;

            default:
                break;
            }
            break;

        /* data-out stage processing */
        case USBHS_UIS_TOKEN_OUT:
            switch (intst & (USBHS_UIS_TOKEN_MASK | USBHS_UIS_ENDP_MASK)) {
            /* end-point 0 data out interrupt */
            case USBHS_UIS_TOKEN_OUT | DEF_UEP0:
                len = USBHSD->RX_LEN;
                if (intst & USBHS_UIS_TOG_OK) {
                    /* if any processing about rx, set it here */
                    if ((USBHS_SetupReqType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) {
                        /* Non-standard request end-point 0 Data download */
                        _errflag = USBHS_UAC_RX(_errflag);
                    } else {
                        /* Standard request end-point 0 Data download */
                    }
                    USBHS_SetupReqLen -= len;
                    if (USBHS_SetupReqLen == 0) {
                        USBHSD->UEP0_TX_LEN = 0;
                        USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
                    }
                }
                break;

            /* end-point 1 data out interrupt */
            case USBHS_UIS_TOKEN_OUT | DEF_UEP1:
            if (intst & USBHS_UIS_TOG_OK) {
                Codec_WriteUACBuffer(USBHS_EP1_Rx_Buf, USBHSD->RX_LEN);
            }
                break;

            // ep2 rx: CDC串口接收
            case USBHS_UIS_TOKEN_OUT | DEF_UEP2:
            if (intst & USBHS_UIS_TOG_OK) {
                USBHSD->UEP2_RX_CTRL ^= USBHS_UEP_R_TOG_DATA1;
                USBHSD->UEP2_RX_CTRL = (USBHSD->UEP2_RX_CTRL & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_ACK;
            }
                break;

            default:
                break;
            }
            break;

        /* Sof pack processing */
        case USBHS_UIS_TOKEN_SOF:
                Codec_MeasureSampleRateAndReportFeedback();
            break;

        default:
            break;
        }
        USBHSD->INT_FG = USBHS_UIF_TRANSFER;
    } 
    else if (intflag & USBHS_UIF_SETUP_ACT) {
        USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_NAK;
        USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_NAK;

        /* Store All Setup Values */
        USBHS_SetupReqType = pUSBHS_SetupReqPak->bmRequestType;
        USBHS_SetupReqCode = pUSBHS_SetupReqPak->bRequest;
        USBHS_SetupReqLen = pUSBHS_SetupReqPak->wLength;
        USBHS_SetupReqValue = pUSBHS_SetupReqPak->wValue;
        USBHS_SetupReqIndex = pUSBHS_SetupReqPak->wIndex;

        len = 0;
        _errflag = 0;
        if ((USBHS_SetupReqType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) {
            /* usb non-standard request processing */
            _errflag = USBHS_UAC_Setup(_errflag);
            if (USBHS_SetupReqType & 0x80) { // in, or get something
                if (USBHS_SetupReqLen > len) {
                    USBHS_SetupReqLen = len;
                }
            } // else out, or set something do not change
        } else {
            /* usb standard request processing */
            switch (USBHS_SetupReqCode) {
            /* get device/configuration/string/report/... descriptors */
            case USB_GET_DESCRIPTOR:
                switch ((uint8_t)(USBHS_SetupReqValue >> 8)) {
                /* get usb device descriptor */
                case USB_DESCR_TYP_DEVICE:
                    pUSBHS_Descr = MyDevDescr;
                    len = DEF_USBD_DEVICE_DESC_LEN;
                    break;

                /* get usb configuration descriptor */
                case USB_DESCR_TYP_CONFIG:
                    /* Query current usb speed */
                    if ((USBHSD->SPEED_TYPE & USBHS_SPEED_TYPE_MASK) == USBHS_SPEED_HIGH) {
                        /* High speed mode */
                        USBHS_DevSpeed = USBHS_SPEED_HIGH;
                        USBHS_DevMaxPackLen = DEF_USBD_HS_PACK_SIZE;
                    } else {
                        /* Full speed mode */
                        USBHS_DevSpeed = USBHS_SPEED_FULL;
                        USBHS_DevMaxPackLen = DEF_USBD_FS_PACK_SIZE;
                    }

                    /* Load usb configuration descriptor by speed */
                    if (USBHS_DevSpeed == USBHS_SPEED_HIGH) {
                        /* High speed mode */
                        pUSBHS_Descr = MyCfgDescr_HS;
                        len = DEF_USBD_CONFIG_HS_DESC_LEN;
                    } else {
                        /* Full speed mode */
                        pUSBHS_Descr = MyCfgDescr_FS;
                        len = DEF_USBD_CONFIG_FS_DESC_LEN;
                    }
                    break;

                /* get usb string descriptor */
                case USB_DESCR_TYP_STRING:
                    switch ((uint8_t)(USBHS_SetupReqValue & 0xFF)) {
                    /* Descriptor 0, Language descriptor */
                    case DEF_STRING_DESC_LANG:
                        pUSBHS_Descr = MyLangDescr;
                        len = DEF_USBD_LANG_DESC_LEN;
                        break;

                    /* Descriptor 1, Manufacturers String descriptor */
                    case DEF_STRING_DESC_MANU:
                        pUSBHS_Descr = MyManuInfo;
                        len = DEF_USBD_MANU_DESC_LEN;
                        break;

                    /* Descriptor 2, Product String descriptor */
                    case DEF_STRING_DESC_PROD:
                        pUSBHS_Descr = MyProdInfo;
                        len = DEF_USBD_PROD_DESC_LEN;
                        break;

                    /* Descriptor 3, Serial-number String descriptor */
                    case DEF_STRING_DESC_SERN:
                    default:
                        pUSBHS_Descr = MySerNumInfo;
                        len = DEF_USBD_SN_DESC_LEN;
                        break;
                    }
                    break;

                /* get usb device qualify descriptor */
                case USB_DESCR_TYP_QUALIF:
                    pUSBHS_Descr = MyQuaDesc;
                    len = DEF_USBD_QUALFY_DESC_LEN;
                    break;

                /* get usb BOS descriptor */
                case USB_DESCR_TYP_BOS:
                    /* USB 2.00 DO NOT support BOS descriptor */
                    errflag = 0xFF;
                    break;

                /* get usb other-speed descriptor */
                case USB_DESCR_TYP_SPEED:
                    if (USBHS_DevSpeed == USBHS_SPEED_HIGH) {
                        /* High speed mode */
                        memcpy (&TAB_USB_HS_OSC_DESC[2], &MyCfgDescr_FS[2], DEF_USBD_CONFIG_FS_DESC_LEN - 2);
                        pUSBHS_Descr = (uint8_t *)&TAB_USB_HS_OSC_DESC[0];
                        len = DEF_USBD_CONFIG_FS_DESC_LEN;
                    } else if (USBHS_DevSpeed == USBHS_SPEED_FULL) {
                        /* Full speed mode */
                        memcpy (&TAB_USB_FS_OSC_DESC[2], &MyCfgDescr_HS[2], DEF_USBD_CONFIG_HS_DESC_LEN - 2);
                        pUSBHS_Descr = (uint8_t *)&TAB_USB_FS_OSC_DESC[0];
                        len = DEF_USBD_CONFIG_HS_DESC_LEN;
                    } else {
                        errflag = 0xFF;
                    }
                    break;

                default:
                    errflag = 0xFF;
                    break;
                }

                /* Copy Descriptors to Endp0 DMA buffer */
                if (USBHS_SetupReqLen > len) {
                    USBHS_SetupReqLen = len;
                }
                len = (USBHS_SetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBHS_SetupReqLen;
                memcpy (USBHS_EP0_Buf, pUSBHS_Descr, len);
                pUSBHS_Descr += len;
                break;

            /* Set usb address */
            case USB_SET_ADDRESS:
                USBHS_DevAddr = (uint16_t)(USBHS_SetupReqValue & 0xFF);
                break;

            /* Get usb configuration now set */
            case USB_GET_CONFIGURATION:
                USBHS_EP0_Buf[0] = USBHS_DevConfig;
                if (USBHS_SetupReqLen > 1) {
                    USBHS_SetupReqLen = 1;
                }
                break;

            /* Set usb configuration to use */
            case USB_SET_CONFIGURATION:
                USBHS_DevConfig = (uint8_t)(USBHS_SetupReqValue & 0xFF);
                USBHS_DevEnumStatus = 0x01;
                break;

            /* Clear or disable one usb feature */
            case USB_CLEAR_FEATURE:
                if ((USBHS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE) {
                    /* clear one device feature */
                    if ((uint8_t)(USBHS_SetupReqValue & 0xFF) == 0x01) {
                        /* clear usb sleep status, device not prepare to sleep */
                        USBHS_DevSleepStatus &= ~0x01;
                    } else {
                        errflag = 0xFF;
                    }
                } else if ((USBHS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) {
                    /* Set End-point Feature */
                    if ((uint8_t)(USBHS_SetupReqValue & 0xFF) == USB_REQ_FEAT_ENDP_HALT) {
                        /* Clear End-point Feature */
                        switch ((uint8_t)(USBHS_SetupReqIndex & 0xFF)) {
                        case (DEF_UEP1 | DEF_UEP_OUT):
                            /* Set End-point 1 OUT ACK */
                            USBHSD->UEP1_RX_CTRL = USBHS_UEP_R_TOG_DATA0;
                            break;

                        case DEF_UEP1 | DEF_UEP_IN:
                            USBHSD->UEP1_TX_CTRL = USBHS_UEP_T_TOG_DATA0;
                            break;

                        case (DEF_UEP2 | DEF_UEP_OUT):
                            USBHSD->UEP2_RX_CTRL = USBHS_UEP_R_RES_ACK;
                            break;

                        case (DEF_UEP2 | DEF_UEP_IN):
                            USBHSD->UEP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
                            cdc_tx_cplt = true;
                            has_zero_package = false;
                            break;

                        case (DEF_UEP3 | DEF_UEP_IN):
                            USBHSD->UEP3_TX_CTRL = USBHS_UEP_T_RES_NAK;
                            break;

                        default:
                            errflag = 0xFF;
                            break;
                        }
                    } else {
                        errflag = 0xFF;
                    }

                } else {
                    errflag = 0xFF;
                }
                break;

            /* set or enable one usb feature */
            case USB_SET_FEATURE:
                if ((USBHS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE) {
                    /* Set Device Feature */
                    if ((uint8_t)(USBHS_SetupReqValue & 0xFF) == USB_REQ_FEAT_REMOTE_WAKEUP) {
                        if (MyCfgDescr_FS[7] & 0x20) {
                            /* Set Wake-up flag, device prepare to sleep */
                            USBHS_DevSleepStatus |= 0x01;
                        } else {
                            errflag = 0xFF;
                        }
                    } else if ((uint8_t)(USBHS_SetupReqValue & 0xFF) == 0x02) {
                        /* test mode deal */
                        if ((USBHS_SetupReqIndex == 0x0100) ||
                            (USBHS_SetupReqIndex == 0x0200) ||
                            (USBHS_SetupReqIndex == 0x0300) ||
                            (USBHS_SetupReqIndex == 0x0400)) {
                            /* Set the flag and wait for the status to be uploaded before proceeding with the actual operation */
                            USBHS_Test_Flag |= 0x80;
                        }
                    } else {
                        errflag = 0xFF;
                    }
                } else if ((USBHS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) {
                    /* Set End-point Feature */
                    if ((uint8_t)(USBHS_SetupReqValue & 0xFF) == USB_REQ_FEAT_ENDP_HALT) {
                        /* Set end-points status stall */
                        switch ((uint8_t)(USBHS_SetupReqIndex & 0xFF)) {
                        case (DEF_UEP1 | DEF_UEP_OUT):
                            /* Set End-point 1 OUT STALL */
                            USBHSD->UEP1_RX_CTRL = (USBHSD->UEP1_RX_CTRL & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_STALL;
                            break;

                        default:
                            errflag = 0xFF;
                            break;
                        }
                    }
                }
                break;

            /* This request allows the host to select another setting for the specified interface  */
            case USB_GET_INTERFACE:
                USBHS_EP0_Buf[0] = audio_stream_interface_work;
                if (USBHS_SetupReqLen > 1) {
                    USBHS_SetupReqLen = 1;
                }
                break;

            case USB_SET_INTERFACE: {
                if (USBHS_DevEnumStatus != 1) {
                    notsupport;
                }
                else if (USBHS_SetupReqIndex != 1) {
                    notsupport;
                }
                else if (USBHS_SetupReqValue == 0) {
                    // mute
                    audio_stream_interface_work = 0;
                    Codec_Stop();
                }
                else {
                    // start
                    audio_stream_interface_work = 1;
                    Codec_Start();
                }
                if (_errflag != 0) {
                    // mute
                    audio_stream_interface_work = 0;
                    Codec_Stop();
                }
            }
                break;

            /* host get status of specified device/interface/end-points */
            case USB_GET_STATUS:
                USBHS_EP0_Buf[0] = 0x00;
                USBHS_EP0_Buf[1] = 0x00;
                if ((USBHS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) {
                    switch ((uint8_t)(USBHS_SetupReqIndex & 0xFF)) {
                    case (DEF_UEP_OUT | DEF_UEP1):
                        if (((USBHSD->UEP1_RX_CTRL) & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_STALL) {
                            USBHS_EP0_Buf[0] = 0x01;
                        }
                        break;

                    case (DEF_UEP_IN | DEF_UEP1):
                        if (((USBHSD->UEP1_TX_CTRL) & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_STALL) {
                            USBHS_EP0_Buf[0] = 0x01;
                        }
                        break;

                    case (DEF_UEP_OUT | DEF_UEP2):
                        if (((USBHSD->UEP2_RX_CTRL) & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_STALL) {
                            USBHS_EP0_Buf[0] = 0x01;
                        }
                        break;

                    case (DEF_UEP_IN | DEF_UEP2):
                        if (((USBHSD->UEP2_TX_CTRL) & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_STALL) {
                            USBHS_EP0_Buf[0] = 0x01;
                        }
                        break;

                    case (DEF_UEP_OUT | DEF_UEP3):
                        if (((USBHSD->UEP3_RX_CTRL) & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_STALL) {
                            USBHS_EP0_Buf[0] = 0x01;
                        }
                        break;

                    default:
                        errflag = 0xFF;
                        break;
                    }
                } else if ((USBHS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE) {
                    if (USBHS_DevSleepStatus & 0x01) {
                        USBHS_EP0_Buf[0] = 0x02;
                    }
                }

                if (USBHS_SetupReqLen > 2) {
                    USBHS_SetupReqLen = 2;
                }
                break;

            default:
                errflag = 0xFF;
                break;
            }
        }

        /* errflag = 0xFF means a request not support or some errors occurred, else correct */
        if (_errflag == 0xFF) {
            /* if one request not support, return stall */
            USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_STALL;
            USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_STALL;
        } else {
            /* end-point 0 data Tx/Rx */
            if (USBHS_SetupReqType & DEF_UEP_IN) {
                /* tx */
                len = (USBHS_SetupReqLen > DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBHS_SetupReqLen;
                USBHS_SetupReqLen -= len;
                USBHSD->UEP0_TX_LEN = len;
                USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
            } else {
                /* rx */
                if (USBHS_SetupReqLen == 0) {
                    USBHSD->UEP0_TX_LEN = 0;
                    USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
                } else {
                    USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
                }
            }
        }
        USBHSD->INT_FG = USBHS_UIF_SETUP_ACT;
        intflag &= ~USBHS_UIF_SETUP_ACT;
    }
    else if (intflag & USBHS_UIF_BUS_RST) {
        /* usb reset interrupt processing */
        USBHS_DevConfig = 0;
        USBHS_DevAddr = 0;
        USBHS_DevSleepStatus = 0;
        USBHS_DevEnumStatus = 0;

        cdc_tx_cplt = true;

        USBHSD->DEV_AD = 0;
        USBHS_Device_Endp_Init();
        USBHSD->INT_FG = USBHS_UIF_BUS_RST;
        intflag &= ~USBHS_UIF_BUS_RST;
    }
    else if (intflag & USBHS_UIF_SUSPEND) {
        USBHSD->INT_FG = USBHS_UIF_SUSPEND;
        intflag &= ~USBHS_UIF_SUSPEND;
        Delay_Us (10);
        /* usb suspend interrupt processing */
        if (USBHSD->MIS_ST & USBHS_UMS_SUSPEND) {
            USBHS_DevSleepStatus |= 0x02;
            if (USBHS_DevSleepStatus == 0x03) {
                /* Handling usb sleep here */
            }
        } else {
            USBHS_DevSleepStatus &= ~0x02;
        }
    }
    else {
        /* other interrupts */
        USBHSD->INT_FG = intflag;
        intflag = 0;
    }
}

/*********************************************************************
 * @fn      USBHS_Send_Resume
 *
 * @brief   USBHS device sends wake-up signal to host
 *
 * @return  none
 */
void USBHS_Send_Resume (void) {
}
