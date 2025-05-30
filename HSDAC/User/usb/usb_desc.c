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

/* Device Descriptor */
const uint8_t MyDevDescr[] =
{
    0x12,                    // bLength
    0x01,                    // bDescriptorType (Device)
    0x00, 0x02,              // bcdUSB 2.00
    0xEF,                    // bDeviceClass
    0x02,                    // bDeviceSubClass
    0x01,                    // bDeviceProtocol
    DEF_USBD_UEP0_SIZE,      // bMaxPacketSize0 64
    USB_WORD (DEF_USB_VID),  // idVendor 0x1A86
    USB_WORD (DEF_USB_PID),  // idProduct 0x5537
    DEF_IC_PRG_VER, 0x00,    // bcdDevice 0.01
    0x01,                    // iManufacturer (String Index)
    0x02,                    // iProduct (String Index)
    0x03,                    // iSerialNumber (String Index)
    0x01,                    // bNumConfigurations 1
};

/* USB≈‰÷√√Ë ˆ∑˚(∏ﬂÀŸ) */
const uint8_t MyCfgDescr_HS[] =
{
    0x09,           // bLength
    0x02,           // bDescriptorType (Configuration)
    USB_WORD(193),  // wTotalLength
    0x04,           // bNumInterfaces
    0x01,           // bConfigurationValue
    0x00,           // iConfiguration (String Index)
    0x80,           // bmAttributes
    250,            // bMaxPower 100mA

    // interface association descriptor
    8,              // length
    0x0b,           // desc type (INTERFACE_ASSOCIATION)
    0x00,           // first interface
    0x02,           // interface count
    0x01,           // function class (AUDIO)
    0x00,           // function sub class (UNDEFIEND)
    0x20,           // function protocool (AF_VER_2)
    0x00,           // function string id

    // Audio Control Interface Descriptor (Audio Control Interface)
    0x09,           // bLength
    0x04,           // bDescriptorType (Interface)
    0x00,           // bInterfaceNumber
    0x00,           // bAlternateSetting
    0x00,           // bNumEndpoints
    0x01,           // bInterfaceClass (Audio)
    0x01,           // bInterfaceSubClass (Audio Control)
    0x20,           // bInterfaceProtocol
    0x00,           // iInterface

    // ---------- start of audio function ----------
    // Audio Control Interface Header Descriptor
    0x09,           // bLength
    0x24,           // bDescriptorType (CS Interface)
    0x01,           // bDescriptorSubtype (Header)
    0x00, 0x02,     // bcdADC (Audio Device Class Specification Version)
    0x01,           // bCatalog
    USB_WORD(46),   // wTotalLength (Length of this descriptor plus all following descriptors in the control interface)
    0x00,           // controls, all not support

    // Clock Dsec
    8,              // length
    0x24,           // desc type (CS_INTERFACE)
    0x0A,           // desc subtype (CLOCK_SOURCE)
    0x03,           // clock id
    0x02,           // clock attribute (internal variable clock, no sof sync)
    0x03,           // controls (frequency w/r, no valid control)
    0x00,           // associated terminal
    0x00,           // string id

    // Input Terminal
    17,             // length
    0x24,           // desc type (CS_INTERFACE)
    0x02,           // desc subtype (INPUT_TERMINAL)
    0x01,           // terminal id
    0x01, 0x01,     // terminal type (USB streaming)
    0x00,           // associated terminal
    0x03,           // clock source id
    0x02,           // num channels
    USB_DWORD(0x03),// channel config
    0x00, 0x00,     // control, all not support
    0x00,           // channel string id
    0x00,           // terminal string id 

    // Output Terminal
    12,             // length
    0x24,           // desc type (CS_INTERFACE)
    0x03,           // desc subtype (OUTPUT_TERMINAL)
    0x02,           // terminal id
    0x01, 0x03,     // terminal type (speakers)
    0x00,           // associated terminal
    0x01,           // input unit id
    0x03,           // clock source id
    0x00, 0x00,     // controls, all not support
    0x00,           // terminal string id

    // No feature unit
    // ---------- end of audio function ----------


    // Audio Stream Interface
    // Audio Streaming Interface Descriptor (Alternate Setting 0)
    0x09,  // bLength
    0x04,  // bDescriptorType (Interface)
    0x01,  // bInterfaceNumber (Audio Streaming Interface)
    0x00,  // bAlternateSetting (Alternate Setting 0)
    0x00,  // bNumEndpoints (No endpoints in this setting)
    0x01,  // bInterfaceClass (Audio)
    0x02,  // bInterfaceSubClass (Audio Streaming)
    0x20,  // bInterfaceProtocol
    0x00,  // iInterface (No interface string)

    // Audio Streaming Interface Descriptor (Alternate Setting 1)
    0x09,  // bLength
    0x04,  // bDescriptorType (Interface)
    0x01,  // bInterfaceNumber
    0x01,  // bAlternateSetting (Alternate Setting 1)
    0x01,  // bNumEndpoints (1 endpoint)
    0x01,  // bInterfaceClass (Audio)
    0x02,  // bInterfaceSubClass (Audio Streaming)
    0x20,  // bInterfaceProtocol
    0x00,  // iInterface (No interface string)

    // audio streaming terminal link desc
    16,             // length
    0x24,           // type (CS_INTERFACE)
    0x01,           // subtype (AS_GENERAL)
    0x01,           // terminal link
    0x00,           // no control support
    0x01,           // format type (FORMAT-1)
    USB_DWORD(0x1), // bmFormats
    0x02,           // num channels
    USB_DWORD(0x3), // channel config
    0x00,           // channel name string id

    // Audio Streaming Format Type Descriptor
    0x06,              // bLength
    0x24,              // bDescriptorType (CS Interface)
    0x02,              // bDescriptorSubtype (Format Type)
    0x01,              // bFormatType (Type I - PCM)
    0x04,              // bSubslotsize
    32,                // bBitResolution

    // Audio Stream Endpoint
    // Audio Streaming Endpoint Descriptor (ISO Data Endpoint)
    0x07,        // bLength
    0x05,        // bDescriptorType (Endpoint)
    0x01,        // bEndpointAddress (OUT endpoint)
    0x09,        // bmAttributes (Isochronous)
    USB_WORD(DEF_USB_EP1_HS_SIZE),  // wMaxPacketSize (192 bytes)
    0x01,        // bInterval (1 frame)

    // Audio Streaming Endpoint Descriptor (General Audio)
    0x08,        // bLength
    0x25,        // bDescriptorType (CS Endpoint)
    0x01,        // bDescriptorSubtype (General)
    0x00,        // bmAttributes (Sampling Frequency Control)
    0x00,        // controls
    0x00,        // bLockDelayUnits
    0x00, 0x00,  // wLockDelay (1 ms)


    // ---------- usb cdc as debug ----------
    // interface association descriptor
    8,              // length
    0x0b,           // desc type (INTERFACE_ASSOCIATION)
    0x02,           // first interface
    0x02,           // interface count
    0x02,           // function class (CDC)
    0x02,           // function sub class (ACM)
    0x01,           // function protocool (AT commands)
    0x00,           // function string id

    // usb cdc interface desc
    9,           // length
    0x04,        // desc tpye (INTERFACE)
    0x02,        // interface number
    0x00,        // alter settings
    0x01,        // num endpoints
    0x02,        // interface class
    0x02,        // interface sub class
    0x01,        // interface protocol
    0x00,        // interface string id

    /* Functional Descriptors */
    0x05,       // length
    0x24,       // functional desc
    0x00,       // header functional desc
    0x10, 0x01, // cdc version 1.10

    /* Length/management descriptor (data class interface 1) */
    0x05,       // length
    0x24,       // functional desc
    0x01,       // subtype
    0x00,       // capabilities
    0x03,       // data interface

    0x04,       // length
    0x24,       // functional desc
    0x02,       // sub type (CALL MANAGERMANT)
    0x02,       // capabilities

    0x05,       // length
    0x24,       // functional desc
    0x06,       // sub type
    0x02,       // master interface (COMMUICATION)
    0x03,       // slave interface (DATA CLASS)

    /* Interrupt upload endpoint descriptor */
    0x07,       // length
    0x05,       // desc type (ENDPOINT)
    0x83,       // address (EP3 IN)
    0x03,       // attribute
    USB_WORD(DEF_USB_EP3_HS_SIZE),
    0x01,

    /* Interface 1 (data interface) descriptor */
    0x09,       // length
    0x04,       // desc type (INTERFACE)
    0x03,       // interface num
    0x00,       // alter settings
    0x02,       // num endpoints
    0x0a,       // class
    0x00,       // subclass
    0x00,       // protocol
    0x00,       // string id

    /* Endpoint descriptor */
    0x07,
    0x05,
    0x02,
    0x02,
    USB_WORD(DEF_USB_EP2_HS_SIZE),
    0x00,

    /* Endpoint descriptor */
    0x07,
    0x05,
    0x82,
    0x02,
    USB_WORD(DEF_USB_EP2_HS_SIZE),
    0x00,
};

/* Configuration Descriptor */
const uint8_t MyCfgDescr_FS[] =
{
    0x09,           // bLength
    0x02,           // bDescriptorType (Configuration)
    USB_WORD(127),  // wTotalLength
    0x02,           // bNumInterfaces 2
    0x01,           // bConfigurationValue
    0x00,           // iConfiguration (String Index)
    0x80,           // bmAttributes
    0x32,           // bMaxPower 100mA

    // interface association descriptor
    8,              // length
    0x0b,           // desc type (INTERFACE_ASSOCIATION)
    0x00,           // first interface
    0x02,           // interface count
    0x01,           // function class (AUDIO)
    0x00,           // function sub class (UNDEFIEND)
    0x20,           // function protocool (AF_VER_2)
    0x00,           // function string id

    // Audio Control Interface Descriptor (Audio Control Interface)
    0x09,           // bLength
    0x04,           // bDescriptorType (Interface)
    0x00,           // bInterfaceNumber
    0x00,           // bAlternateSetting
    0x00,           // bNumEndpoints
    0x01,           // bInterfaceClass (Audio)
    0x01,           // bInterfaceSubClass (Audio Control)
    0x20,           // bInterfaceProtocol
    0x00,           // iInterface

    // ---------- start of audio function ----------
    // Audio Control Interface Header Descriptor
    0x09,           // bLength
    0x24,           // bDescriptorType (CS Interface)
    0x01,           // bDescriptorSubtype (Header)
    0x00, 0x02,     // bcdADC (Audio Device Class Specification Version)
    0x01,           // bCatalog
    USB_WORD(46),   // wTotalLength (Length of this descriptor plus all following descriptors in the control interface)
    0x00,           // controls, all not support

    // Clock Dsec
    8,              // length
    0x24,           // desc type (CS_INTERFACE)
    0x0A,           // desc subtype (CLOCK_SOURCE)
    0x03,           // clock id
    0x02,           // clock attribute (internal variable clock, no sof sync)
    0x03,           // controls (frequency w/r, no valid control)
    0x00,           // associated terminal
    0x00,           // string id

    // Input Terminal
    17,             // length
    0x24,           // desc type (CS_INTERFACE)
    0x02,           // desc subtype (INPUT_TERMINAL)
    0x01,           // terminal id
    0x01, 0x01,     // terminal type (USB streaming)
    0x00,           // associated terminal
    0x03,           // clock source id
    0x02,           // num channels
    USB_DWORD(0x03),// channel config
    0x00, 0x00,     // control, all not support
    0x00,           // channel string id
    0x00,           // terminal string id 

    // Output Terminal
    12,             // length
    0x24,           // desc type (CS_INTERFACE)
    0x03,           // desc subtype (OUTPUT_TERMINAL)
    0x02,           // terminal id
    0x01, 0x03,     // terminal type (speakers)
    0x00,           // associated terminal
    0x01,           // input unit id
    0x03,           // clock source id
    0x00, 0x00,     // controls, all not support
    0x00,           // terminal string id

    // No feature unit
    // ---------- end of audio function ----------


    // Audio Stream Interface
    // Audio Streaming Interface Descriptor (Alternate Setting 0)
    0x09,  // bLength
    0x04,  // bDescriptorType (Interface)
    0x01,  // bInterfaceNumber (Audio Streaming Interface)
    0x00,  // bAlternateSetting (Alternate Setting 0)
    0x00,  // bNumEndpoints (No endpoints in this setting)
    0x01,  // bInterfaceClass (Audio)
    0x02,  // bInterfaceSubClass (Audio Streaming)
    0x20,  // bInterfaceProtocol
    0x00,  // iInterface (No interface string)

    // Audio Streaming Interface Descriptor (Alternate Setting 1)
    0x09,  // bLength
    0x04,  // bDescriptorType (Interface)
    0x01,  // bInterfaceNumber
    0x01,  // bAlternateSetting (Alternate Setting 1)
    0x01,  // bNumEndpoints (1 endpoint)
    0x01,  // bInterfaceClass (Audio)
    0x02,  // bInterfaceSubClass (Audio Streaming)
    0x20,  // bInterfaceProtocol
    0x00,  // iInterface (No interface string)

    // audio streaming terminal link desc
    16,             // length
    0x24,           // type (CS_INTERFACE)
    0x01,           // subtype (AS_GENERAL)
    0x01,           // terminal link
    0x00,           // no control support
    0x01,           // format type (FORMAT-1)
    USB_DWORD(0x1), // bmFormats
    0x02,           // num channels
    USB_DWORD(0x3), // channel config
    0x00,           // channel name string id

    // Audio Streaming Format Type Descriptor
    0x06,              // bLength
    0x24,              // bDescriptorType (CS Interface)
    0x02,              // bDescriptorSubtype (Format Type)
    0x01,              // bFormatType (Type I - PCM)
    0x04,              // bSubslotsize
    32,                // bBitResolution

    // Audio Stream Endpoint
    // Audio Streaming Endpoint Descriptor (ISO Data Endpoint)
    0x07,        // bLength
    0x05,        // bDescriptorType (Endpoint)
    0x03,        // bEndpointAddress (OUT endpoint)
    0x09,        // bmAttributes (Isochronous)
    USB_WORD(DEF_USB_EP3_FS_SIZE),  // wMaxPacketSize (192 bytes)
    0x01,        // bInterval (1 frame)

    // Audio Streaming Endpoint Descriptor (General Audio)
    0x08,        // bLength
    0x25,        // bDescriptorType (CS Endpoint)
    0x01,        // bDescriptorSubtype (General)
    0x00,        // bmAttributes (Sampling Frequency Control)
    0x00,        // controls
    0x00,        // bLockDelayUnits
    0x00, 0x00,  // wLockDelay (1 ms)
};

/* Language Descriptor */
const uint8_t MyLangDescr[] =
{
    0x04, 0x03, 0x09, 0x04
};

/* Manufacturer Descriptor */
const uint8_t MyManuInfo[] =
{
    14, 0x03, 'w', 0, 'c', 0, 'h', 0, '.', 0, 'c', 0, 'n', 0
};

#define UC(X) USB_WORD(X)
/* Product Information */
const uint8_t MyProdInfo[] =
{
    18, 0x03, UC('C'), UC('H'), UC('3'), UC('2'), UC('-'), UC('U'), UC('A'), UC('C')
};

/* Serial Number Information */
const uint8_t MySerNumInfo[] =
{
    20, 0x03, UC('2'), UC('0'), UC('2'), UC('5'), UC('-'), UC('5'), UC('-'), UC('2'), UC('8')
};

/* Device Qualified Descriptor */
const uint8_t MyQuaDesc[] =
{
    0x0A,
    0x06,
    0x00, 0x02,
    0xEF,
    0x02,
    0x01,
    0x40,
    0x01,
    0x00,
};

/* Device BOS Descriptor */
const uint8_t MyBOSDesc[] =
{
    0x05,
    0x0F,
    0x0C,
    0x00,
    0x01,
    0x07,
    0x10,
    0x02,
    0x02,
    0x00,
    0x00,
    0x00,
};

/* USB Full-Speed Mode, Other speed configuration Descriptor */
uint8_t TAB_USB_FS_OSC_DESC[sizeof (MyCfgDescr_HS)] =
{
    /* Other parts are copied through the program */
    0x09,
    0x07,
};

/* USB High-Speed Mode, Other speed configuration Descriptor */
uint8_t TAB_USB_HS_OSC_DESC[sizeof (MyCfgDescr_FS)] =
{
    /* Other parts are copied through the program */
    0x09,
    0x07,
};
