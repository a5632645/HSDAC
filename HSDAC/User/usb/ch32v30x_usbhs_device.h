/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v30x_usbhs_device.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2023/11/20
* Description        : USB2.0 High speed device operation related header files
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32V30X_USBHS_DEVICE_H__
#define __CH32V30X_USBHS_DEVICE_H__

/*******************************************************************************/
/* Header File */
#include "debug.h"
#include "string.h"
#include "ch32v30x_usb.h"
#include "usb_desc.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************/
/* Macro Definition */

#define DEF_UEP_IN                    0x80
#define DEF_UEP_OUT                   0x00
/* Endpoint Number */
#define DEF_UEP_BUSY                  0x01
#define DEF_UEP_FREE                  0x00
#define DEF_UEP_NUM                   16
#define DEF_UEP0                      0x00
#define DEF_UEP1                      0x01
#define DEF_UEP2                      0x02
#define DEF_UEP3                      0x03
#define DEF_UEP4                      0x04
#define DEF_UEP5                      0x05
#define DEF_UEP6                      0x06
#define DEF_UEP7                      0x07
#define DEF_UEP8                      0x08
#define DEF_UEP9                      0x09
#define DEF_UEP10                     0x0A
#define DEF_UEP11                     0x0B
#define DEF_UEP12                     0x0C
#define DEF_UEP13                     0x0D
#define DEF_UEP14                     0x0E
#define DEF_UEP15                     0x0F

#define USBHSD_UEP_CFG_BASE           0x40023410
#define USBHSD_UEP_BUF_MOD_BASE       0x40023418
#define USBHSD_UEP_RXDMA_BASE         0x40023420
#define USBHSD_UEP_TXDMA_BASE         0x4002345C
#define USBHSD_UEP_TXLEN_BASE         0x400234DC
#define USBHSD_UEP_TXCTL_BASE         0x400234DE
#define USBHSD_UEP_TX_EN( N )         ( (uint16_t)( 0x01 << N ) )
#define USBHSD_UEP_RX_EN( N )         ( (uint16_t)( 0x01 << ( N + 16 ) ) )
#define USBHSD_UEP_DOUBLE_BUF( N )    ( (uint16_t)( 0x01 << N ) )
#define DEF_UEP_DMA_LOAD              0 /* Direct the DMA address to the data to be processed */
#define DEF_UEP_CPY_LOAD              1 /* Use memcpy to move data to a buffer */
#define USBHSD_UEP_RXDMA( N )         ( *((volatile uint32_t *)( USBHSD_UEP_RXDMA_BASE + ( N - 1 ) * 0x04 ) ) )
#define USBHSD_UEP_RXBUF( N )         ( (uint8_t *)(*((volatile uint32_t *)( USBHSD_UEP_RXDMA_BASE + ( N - 1 ) * 0x04 ) ) ) + 0x20000000 )
#define USBHSD_UEP_TXCTRL( N )        ( *((volatile uint8_t *)( USBHSD_UEP_TXCTL_BASE + ( N - 1 ) * 0x04 ) ) )
#define USBHSD_UEP_TXDMA( N )         ( *((volatile uint32_t *)( USBHSD_UEP_TXDMA_BASE + ( N - 1 ) * 0x04 ) ) )
#define USBHSD_UEP_TXBUF( N )         ( (uint8_t *)(*((volatile uint32_t *)( USBHSD_UEP_TXDMA_BASE + ( N - 1 ) * 0x04 ) ) ) + 0x20000000 )
#define USBHSD_UEP_TLEN( N )          ( *((volatile uint16_t *)( USBHSD_UEP_TXLEN_BASE + ( N - 1 ) * 0x04 ) ) )

/* USB SPEED TYPE */
#define USBHS_SPEED_TYPE_MASK         ((uint8_t)(0x03))
#define USBHS_SPEED_LOW               ((uint8_t)(0x02))
#define USBHS_SPEED_FULL              ((uint8_t)(0x00))
#define USBHS_SPEED_HIGH              ((uint8_t)(0x01))

/* Test mode */
#define TEST_MASK       0x03
#define TEST_SE0        0x00
#define TEST_PACKET     0x01
#define TEST_J          0x02
#define TEST_K          0x03

extern void USBHS_RCC_Init( void );
extern void USBHS_Device_Init ( FunctionalState sta );

#ifdef __cplusplus
}
#endif

#endif

