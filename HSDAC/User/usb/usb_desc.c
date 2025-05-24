/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_desc.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/20
 * Description        : usb device descriptor,configuration descriptor,
 *                      string descriptors and other descriptors.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "usb_desc.h"

#define USB_WORD(X) (uint8_t)(X & 0xff), (uint8_t)(X >> 8)

/* Device Descriptor */
const uint8_t  MyDevDescr[ ] =
{
    0x12,       // bLength
    0x01,       // bDescriptorType (Device)
    0x00, 0x02, // bcdUSB 2.00
    0x02,       // bDeviceClass
    0x00,       // bDeviceSubClass
    0x00,       // bDeviceProtocol
    DEF_USBD_UEP0_SIZE,   // bMaxPacketSize0 64
    USB_WORD(DEF_USB_VID),  // idVendor 0x1A86
    USB_WORD(DEF_USB_PID),  // idProduct 0x5537
    USB_WORD(DEF_IC_PRG_VER), // bcdDevice 0.01
    0x01,       // iManufacturer (String Index)
    0x02,       // iProduct (String Index)
    0x03,       // iSerialNumber (String Index)
    0x01,       // bNumConfigurations 1
};

/* USBÅäÖÃÃèÊö·û(¸ßËÙ) */
const uint8_t  MyCfgDescr_HS[ ] =
{
    0x09,        // bLength
    0x02,        // bDescriptorType (Configuration)
    USB_WORD(67),// wTotalLength
    0x02,        // bNumInterfaces
    0x01,        // bConfigurationValue
    0x00,        // iConfiguration (String Index)
    0x80,        // bmAttributes
    0x32,        // bMaxPower (2mA per unit)

    0x09,        // bLength
    0x04,        // bDescriptorType (Interface)
    0x00,        // bInterfaceNumber
    0x00,        // bAlternateSetting
    0x01,        // bNumEndpoints
    0x02,        // bInterfaceClass
    0x02,        // bInterfaceSubClass
    0x01,        // bInterfaceProtocol
    0x00,        // iInterface (String Index)

    0x05,
    0x24,
    0x00,
    0x10,
    0x01,

    /* Length/management descriptor (data class interface 1) */
    0x05, 0x24, 0x01, 0x00, 0x01,
    0x04, 0x24, 0x02, 0x02,
    0x05, 0x24, 0x06, 0x00, 0x01,

    /* Interrupt upload endpoint descriptor */
    // EP1 IN, ÖÐ¶Ï´«Êä
    0x07, 0x05, 0x81, 0x03, (uint8_t)DEF_USB_EP1_HS_SIZE, (uint8_t)(DEF_USB_EP1_HS_SIZE >> 8), 0x01,

    /* Interface 1 (data interface) descriptor */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00,

    0x07,        // bLength
    0x05,        // bDescriptorType (Endpoint)
    0x82,        // bEndpointAddress (EP2 IN)
    0x02,        // bmAttributes (Bulk)
    (uint8_t)DEF_USB_EP1_HS_SIZE, (uint8_t)( DEF_USB_EP1_HS_SIZE >> 8 ), // wMaxPacketSize
    0x00,        // bInterval 0 (unit depends on device speed)

    0x07,        // bLength
    0x05,        // bDescriptorType (Endpoint)
    0x02,        // bEndpointAddress (EP2 OUT)
    0x02,        // bmAttributes (Bulk)
    (uint8_t)DEF_USB_EP1_HS_SIZE, (uint8_t)( DEF_USB_EP1_HS_SIZE >> 8 ), // wMaxPacketSize
    0x00,        // bInterval 0 (unit depends on device speed)
};

/* Language Descriptor */
const uint8_t  MyLangDescr[] =
{
    0x04, 0x03, 0x09, 0x04
};

/* Manufacturer Descriptor */
const uint8_t  MyManuInfo[] =
{
    0x0E, 0x03, 'w', 0, 'c', 0, 'h', 0, '.', 0, 'c', 0, 'n', 0
};

/* Product Information */
const uint8_t  MyProdInfo[] =
{
    18,   // length
    0x03, // type (string)
    USB_WORD('C'), USB_WORD('H'), USB_WORD('3'), USB_WORD('2'), USB_WORD(' '), USB_WORD('C'), USB_WORD('D'), USB_WORD('C')
};

/* Serial Number Information */
const uint8_t  MySerNumInfo[] =
{
    20,
    0x03,
    USB_WORD('2'), USB_WORD('0'), USB_WORD('2'), USB_WORD('5'), USB_WORD('-'), USB_WORD('5'), USB_WORD('-'), USB_WORD('2'), USB_WORD('4')
};

/* Device Qualified Descriptor */
const uint8_t MyQuaDesc[ ] =
{
    0x0A, 0x06, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0x40, 0x01, 0x00,
};

/* Device BOS Descriptor */
const uint8_t MyBOSDesc[ ] =
{
    0x05, 0x0F, 0x0C, 0x00, 0x01,
    0x07, 0x10, 0x02, 0x02, 0x00, 0x00, 0x00,
};

/* USB Full-Speed Mode, Other speed configuration Descriptor */
uint8_t TAB_USB_FS_OSC_DESC[ sizeof(MyCfgDescr_HS) ] =
{
    /* Other parts are copied through the program */
    0x09, 0x07,
};

