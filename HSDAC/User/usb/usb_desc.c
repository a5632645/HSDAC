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
#include "usb_audio_v1_0.h"
#include "usb_audio_v2_0.h"
#include "usb_chapter_9.h"
#include "usbd_audio_core.h"
#include "usbd_def.h"

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
    USB_WORD(DEF_USB_EP3_HS_SIZE),  // wMaxPacketSize (192 bytes)
    0x01,        // bInterval (1 frame)

    // Audio Streaming Endpoint Descriptor (General Audio)
    0x08,        // bLength
    0x25,        // bDescriptorType (CS Endpoint)
    0x01,        // bDescriptorSubtype (General)
    0x00,        // bmAttributes (Sampling Frequency Control)
    0x00,        // controls
    0x00,        // bLockDelayUnits
    0x01, 0x00,  // wLockDelay (1 ms)
};

const uint8_t MyCfgDescr_HS2[] = 
{
    /* Configuration descriptor 
     * The interface descriptor is immediately after the descriptor. The interface 
     * descriptor indicates the type of interface and the number of corresponding endpoints */
    0x09,                                           /* bLength */
    USB_DESC_TYPE_CONFIGURATION,                    /* bDescriptorType */
    0x86,              /* wTotalLength  226 bytes + 59 cdc*/
    0x00,
    0x02,  /* bNumInterfaces */
    0x01,                                           /* bConfigurationValue */
    0x00,                            /* iConfiguration */
    0x80,                                           /* bmAttributes: Bus Powered according to user configuration */
    USBD_MAX_POWER,                                 /* bMaxPower = 100 mA */
    /* 09 byte */
    
    /* Interface Association descriptor 
     * USB composite devices are generally implemented with Interface Association Descriptor (IAD), 
     * which is to add IAD descriptor before the interface to be merged */
    USB_AUDIO_IAD_DESC_SIZE,                /* bLength */
    USB_DESC_TYPE_IAD,                      /* bDescriptorType */
    AUDIO_FIRST_INTERFACE_NUM,              /* bFirstInterface */
    AUDIO_INTERFACES_COUNT,                 /* bInterfaceCount */
    USB_DEVICE_CLASS_AUDIO,                 /* bFunctionClass */
    USB_AUDIO_V2_FUNCTION_SUBCLASS_UNDEFINED, /* bFunctionSubClass */
    USB_AUDIO_V2_AF_VERSION_02_00,          /* bFunctionProtocol */
    0x00,                                   /* iFunction - USB Audio 2.0 */
    /* 08 byte */

    /* Interface descriptor */
    AUDIO_INTERFACE_DESC_SIZE,              /* bLength */
    USB_DESC_TYPE_INTERFACE,                /* bDescriptorType */
    AUDIO_CONTROL_INTERFACE_NUM,            /* bInterfaceNumber */
    0x00,                                   /* bAlternateSetting */
    0x00,                                   /* bNumEndpoints */
    USB_AUDIO_V2_CLASS_CODE,                /* bInterfaceClass */
    USB_AUDIO_V2_AUDIOCONTROL,              /* bInterfaceSubClass */
    USB_AUDIO_V2_IP_VERSION_02_00,          /* bInterfaceProtocol */
    0x00,                                   /* iInterface - Topology Control */
    /* 09 byte */

    /* AudioControl Interface Descriptor
     * The audio device interface header should contain a class-specific AC interface header descriptor,
     * which is used to define other functional ports of the interface.
     * Class-specific AC interface header descriptor (AC interface header)
     *******************************************************************/
    AUDIO_INTERFACE_DESC_SIZE,              /* bLength */
    USB_AUDIO_V2_CS_INTERFACE,              /* bDescriptorType */
    USB_AUDIO_V2_HEADER,                    /* bDescriptorSubtype */
    0x00,0x02,          /* 2.00 */          /* bcdADC */
    USB_AUDIO_V2_DESKTOP_SPEAKER,                    /* bCategory */
    0x2E,0x00,                              /* wTotalLength  */
    0x00,                                   /* bmControls */
    /* 09 byte */

    /* AudioControl Interface Descriptor
     * Audio device frequency description Clock source for OUT traffic */
    0x08,                                   /* bLength */
    USB_AUDIO_V2_CS_INTERFACE,              /* bDescriptorType */
    USB_AUDIO_V2_CLOCK_SOURCE,              /* bDescriptorSubtype */
    AUDIO_SCLOCK_TERMINAL_OUTPUT,           /* bClockID */
    0x00,                                   /* bmAttributes - 0 - External clock cource, 1 - Internal fixed clock, 3 - Internal programmable clock */
    0x03,                                   /* bmControls - Clock Frequency Control (read/write) */
    0x00,                                   /* bAssocTerminal - Constant corresponding output port ID */
    0x00,                                   /* iClockSource -  */
    /* 08 byte */

    /* AudioControl Interface Descriptor
     * USB Input Terminal for OUTPUT Descriptor */
    0x11,                                   /* bLength */
    USB_AUDIO_V2_CS_INTERFACE,              /* bDescriptorType */
    USB_AUDIO_V2_INPUT_TERMINAL,            /* bDescriptorSubtype */
    AUDIO_IN_TERMINAL_OUTPUT,               /* bTerminalID */
    0x01,0x01,                              /* wTerminalType AUDIO_TERMINAL_USB_STREAMING  0x0101 */
    0x00,                                   /* bAssocTerminal */
    AUDIO_SCLOCK_TERMINAL_OUTPUT,           /* bCSourceID */
    0x02,                                   /* bNrChannels */
    0x00,0x00,0x00,0x00,                    /* bmChannelConfig 0x00000000  all channels control */
    0x00,0x00,                              /* bmControls - 0x0003 Copy Protect Control (read/write) */
    0x00,                                   /* iChannelNames */
    0x00,                                   /* iTerminal - USBH Out */
    /* 17 byte */

    
    /* AudioControl Interface Descriptor
     * USB Output Terminal for OUTPUT Descriptor */
    0x0C,                                   /* bLength */
    USB_AUDIO_V2_CS_INTERFACE,              /* bDescriptorType */
    USB_AUDIO_V2_OUTPUT_TERMINAL,           /* bDescriptorSubtype */
    AUDIO_OUT_TERMINAL_OUTPUT,              /* bTerminalID */
    0x02,0x06,                              /* wTerminalType - A generic digital audio interface */
    0x00,                                   /* bAssocTerminal */
    AUDIO_IN_TERMINAL_OUTPUT,               /* bSourceID */
    AUDIO_SCLOCK_TERMINAL_OUTPUT,           /* bCSourceID */
    0x00,0x00,                              /* bmControls - 0x0003 Copy Protect Control (read/write) */
    0x00,                                   /* iTerminal - USBD In */
    /* 12 byte */

    /* Interface Descriptor - Audio Streaming OUT Interface - Alt0 */
    AUDIO_INTERFACE_DESC_SIZE,              /* bLength */
    USB_DESC_TYPE_INTERFACE,                /* bDescriptorType */
    AUDIO_STREAMING_OUTPUT_INTERFACE_NUM,   /* bInterfaceNumber */
    USB_AUDIO_OUTPUT_ALTSET_OFF,            /* bAlternateSetting */
    0x00,                                   /* bNumEndpoints */
    USB_AUDIO_V2_CLASS_CODE,                /* bInterfaceClass */
    USB_AUDIO_V2_AUDIOSTREAMING,            /* bInterfaceSubClass */
    USB_AUDIO_V2_IP_VERSION_02_00,          /* bInterfaceProtocol */
    0x00,                                   /* iInterface - Playback Inactive */
    /* 09 byte */
    
    /* Interface Descriptor - Audio Streaming OUT Interface - Alt1 */
    AUDIO_INTERFACE_DESC_SIZE,              /* bLength */
    USB_DESC_TYPE_INTERFACE,                /* bDescriptorType */
    AUDIO_STREAMING_OUTPUT_INTERFACE_NUM,   /* bInterfaceNumber */
    USB_AUDIO_OUTPUT_ALTSET_2CH_ON,         /* bAlternateSetting */
    0x02,                                   /* bNumEndpoints */
    USB_AUDIO_V2_CLASS_CODE,                /* bInterfaceClass */
    USB_AUDIO_V2_AUDIOSTREAMING,            /* bInterfaceSubClass */
    USB_AUDIO_V2_IP_VERSION_02_00,          /* bInterfaceProtocol */
    0x00,                                   /* iInterface - Playback Active */
    /* 09 byte */

    /* AudioStreaming Interface Descriptor - Audio Stream OUT Interface Desc */
    /* Interface 1, Alternate Setting 1                                           */
    0x10,                                   /* bLength */
    USB_AUDIO_V2_CS_INTERFACE,              /* bDescriptorType */
    USB_AUDIO_V2_AS_GENERAL,                /* bDescriptorSubtype */
    AUDIO_IN_TERMINAL_OUTPUT,               /* bTerminalLink */
    0x00,                                   /* bmControls */
    USB_AUDIO_V2_FORMAT_TYPE_I,             /* bFormatType */
    0x01,0x00,0x00,0x00,                    /* bmFormats D0 - PCM */
    0x02,                                   /* bNrChannels */
    0x00,0x00,0x00,0x00,                    /* bmChannelConfig 0x00000000 driver choose channels */
    0x00,                                   /* iChannelNames */
    /* 16 byte */

    /* AudioStreaming Interface Descriptor - Audio USB_OUT Format */
    0x6,                                    /* bLength */
    USB_AUDIO_V2_CS_INTERFACE,              /* bDescriptorType */
    AUDIO_STREAMING_FORMAT_TYPE,            /* bDescriptorSubtype */
    USB_AUDIO_V2_FORMAT_TYPE_I,             /* bFormatType */
    0x04,                                   /* bSubslotSize :  4 Bytes per frame (32bits) */
    32,                                     /* bBitResolution (32-bits per sample) */
    /* 06 byte */ 

    /* Endpoint Descriptor - STD AS ISO OUT Endpoint */
    0x07,                                   /* bLength */
    USB_DESC_TYPE_ENDPOINT,                 /* bDescriptorType */
    AUDIO_OUT_EP,                           /* bEndpointAddress 1 out endpoint */
    0x05,                                   /* bmAttributes Asynchronous Isochronous mode */
    0x00,0x04,                              /* wMaxPacketSize in Bytes 1024 per micro-frame */
    0x01,                                   /* bInterval (Frames/Subframes) 2^(1-1)=1 */
    /* 07 byte */
    
    /* AudioControl Endpoint Descriptor - CS AS ISO OUT Endpoint */
    0x08,                                   /* bLength */
    USB_AUDIO_V2_CS_ENDPOINT,               /* bDescriptorType */
    USB_AUDIO_V2_EP_GENERAL,                /* bDescriptorSubtype */
    0x00,                                   /* bmAttributes */
    0x00,                                   /* bmControls */
    0x00,                                   /* bLockDelayUnits - Undefined */
    0x00,0x00,                              /* wLockDelay */
    /* 08 byte */
    
    /* Endpoint Descriptor - STD AS ISO Feedback Endpoint */
    0x07,                                   /* bLength */
    USB_DESC_TYPE_ENDPOINT,                 /* bDescriptorType */
    AUDIO_FB_EP,                            /* bEndpointAddress 1 int endpoint */
    0x11,                                   /* bmAttributes FeedBack Isochronous mode */
    0x04,0x00,                              /* wMaxPacketSize in Bytes 4 per micro-frame */
    0x03,                                   /* bInterval (Frames/Subframes) 2^(3-1)=4 */
    /* 07 byte */
};

const uint8_t MyCfgDescr_FS2[] = 
{
    /* Configuration descriptor 
     * The interface descriptor is immediately after the descriptor. The interface 
     * descriptor indicates the type of interface and the number of corresponding endpoints */
    0x09,                                           /* bLength */
    USB_DESC_TYPE_CONFIGURATION,                    /* bDescriptorType */
    0x86,              /* wTotalLength  226 bytes + 59 cdc*/
    0x00,
    0x02,  /* bNumInterfaces */
    0x01,                                           /* bConfigurationValue */
    USBD_IDX_CONFIG_STR,                            /* iConfiguration */
    0x80,                                           /* bmAttributes: Bus Powered according to user configuration */
    USBD_MAX_POWER,                                 /* bMaxPower = 100 mA */
    /* 09 byte */
    
    /* Interface Association descriptor 
     * USB composite devices are generally implemented with Interface Association Descriptor (IAD), 
     * which is to add IAD descriptor before the interface to be merged */
    USB_AUDIO_IAD_DESC_SIZE,                /* bLength */
    USB_DESC_TYPE_IAD,                      /* bDescriptorType */
    AUDIO_FIRST_INTERFACE_NUM,              /* bFirstInterface */
    AUDIO_INTERFACES_COUNT,                 /* bInterfaceCount */
    USB_DEVICE_CLASS_AUDIO,                 /* bFunctionClass */
    USB_AUDIO_V2_FUNCTION_SUBCLASS_UNDEFINED, /* bFunctionSubClass */
    USB_AUDIO_V2_AF_VERSION_02_00,          /* bFunctionProtocol */
    0x05,                                   /* iFunction - USB Audio 2.0 */
    /* 08 byte */

    /* Interface descriptor */
    AUDIO_INTERFACE_DESC_SIZE,              /* bLength */
    USB_DESC_TYPE_INTERFACE,                /* bDescriptorType */
    AUDIO_CONTROL_INTERFACE_NUM,            /* bInterfaceNumber */
    0x00,                                   /* bAlternateSetting */
    0x00,                                   /* bNumEndpoints */
    USB_AUDIO_V2_CLASS_CODE,                /* bInterfaceClass */
    USB_AUDIO_V2_AUDIOCONTROL,              /* bInterfaceSubClass */
    USB_AUDIO_V2_IP_VERSION_02_00,          /* bInterfaceProtocol */
    0x06,                                   /* iInterface - Topology Control */
    /* 09 byte */

    /* AudioControl Interface Descriptor
     * The audio device interface header should contain a class-specific AC interface header descriptor,
     * which is used to define other functional ports of the interface.
     * Class-specific AC interface header descriptor (AC interface header)
     *******************************************************************/
    AUDIO_INTERFACE_DESC_SIZE,              /* bLength */
    USB_AUDIO_V2_CS_INTERFACE,              /* bDescriptorType */
    USB_AUDIO_V2_HEADER,                    /* bDescriptorSubtype */
    0x00,0x02,          /* 2.00 */          /* bcdADC */
    USB_AUDIO_V2_DESKTOP_SPEAKER,                    /* bCategory */
    0x2E,0x00,                              /* wTotalLength  */
    0x00,                                   /* bmControls */
    /* 09 byte */

    /* AudioControl Interface Descriptor
     * Audio device frequency description Clock source for OUT traffic */
    0x08,                                   /* bLength */
    USB_AUDIO_V2_CS_INTERFACE,              /* bDescriptorType */
    USB_AUDIO_V2_CLOCK_SOURCE,              /* bDescriptorSubtype */
    AUDIO_SCLOCK_TERMINAL_OUTPUT,           /* bClockID */
    0x00,                                   /* bmAttributes - 0 - External clock cource, 1 - Internal fixed clock, 3 - Internal programmable clock */
    0x03,                                   /* bmControls - Clock Frequency Control (read/write) */
    0x00,                                   /* bAssocTerminal - Constant corresponding output port ID */
    0x00,                                   /* iClockSource -  */
    /* 08 byte */

    /* AudioControl Interface Descriptor
     * USB Input Terminal for OUTPUT Descriptor */
    0x11,                                   /* bLength */
    USB_AUDIO_V2_CS_INTERFACE,              /* bDescriptorType */
    USB_AUDIO_V2_INPUT_TERMINAL,            /* bDescriptorSubtype */
    AUDIO_IN_TERMINAL_OUTPUT,               /* bTerminalID */
    0x01,0x01,                              /* wTerminalType AUDIO_TERMINAL_USB_STREAMING  0x0101 */
    0x00,                                   /* bAssocTerminal */
    AUDIO_SCLOCK_TERMINAL_OUTPUT,           /* bCSourceID */
    0x02,                                   /* bNrChannels */
    0x00,0x00,0x00,0x00,                    /* bmChannelConfig 0x00000000  all channels control */
    0x00,0x00,                              /* bmControls - 0x0003 Copy Protect Control (read/write) */
    0x00,                                   /* iChannelNames */
    0x00,                                   /* iTerminal - USBH Out */
    /* 17 byte */

    
    /* AudioControl Interface Descriptor
     * USB Output Terminal for OUTPUT Descriptor */
    0x0C,                                   /* bLength */
    USB_AUDIO_V2_CS_INTERFACE,              /* bDescriptorType */
    USB_AUDIO_V2_OUTPUT_TERMINAL,           /* bDescriptorSubtype */
    AUDIO_OUT_TERMINAL_OUTPUT,              /* bTerminalID */
    0x02,0x06,                              /* wTerminalType - A generic digital audio interface */
    0x00,                                   /* bAssocTerminal */
    AUDIO_IN_TERMINAL_OUTPUT,               /* bSourceID */
    AUDIO_SCLOCK_TERMINAL_OUTPUT,           /* bCSourceID */
    0x00,0x00,                              /* bmControls - 0x0003 Copy Protect Control (read/write) */
    0x00,                                   /* iTerminal - USBD In */
    /* 12 byte */

    /* Interface Descriptor - Audio Streaming OUT Interface - Alt0 */
    AUDIO_INTERFACE_DESC_SIZE,              /* bLength */
    USB_DESC_TYPE_INTERFACE,                /* bDescriptorType */
    AUDIO_STREAMING_OUTPUT_INTERFACE_NUM,   /* bInterfaceNumber */
    USB_AUDIO_OUTPUT_ALTSET_OFF,            /* bAlternateSetting */
    0x00,                                   /* bNumEndpoints */
    USB_AUDIO_V2_CLASS_CODE,                /* bInterfaceClass */
    USB_AUDIO_V2_AUDIOSTREAMING,            /* bInterfaceSubClass */
    USB_AUDIO_V2_IP_VERSION_02_00,          /* bInterfaceProtocol */
    0x00,                                   /* iInterface - Playback Inactive */
    /* 09 byte */
    
    /* Interface Descriptor - Audio Streaming OUT Interface - Alt1 */
    AUDIO_INTERFACE_DESC_SIZE,              /* bLength */
    USB_DESC_TYPE_INTERFACE,                /* bDescriptorType */
    AUDIO_STREAMING_OUTPUT_INTERFACE_NUM,   /* bInterfaceNumber */
    USB_AUDIO_OUTPUT_ALTSET_2CH_ON,         /* bAlternateSetting */
    0x02,                                   /* bNumEndpoints */
    USB_AUDIO_V2_CLASS_CODE,                /* bInterfaceClass */
    USB_AUDIO_V2_AUDIOSTREAMING,            /* bInterfaceSubClass */
    USB_AUDIO_V2_IP_VERSION_02_00,          /* bInterfaceProtocol */
    0x00,                                   /* iInterface - Playback Active */
    /* 09 byte */

    /* AudioStreaming Interface Descriptor - Audio Stream OUT Interface Desc */
    /* Interface 1, Alternate Setting 1                                           */
    0x10,                                   /* bLength */
    USB_AUDIO_V2_CS_INTERFACE,              /* bDescriptorType */
    USB_AUDIO_V2_AS_GENERAL,                /* bDescriptorSubtype */
    AUDIO_IN_TERMINAL_OUTPUT,               /* bTerminalLink */
    0x00,                                   /* bmControls */
    USB_AUDIO_V2_FORMAT_TYPE_I,             /* bFormatType */
    0x01,0x00,0x00,0x00,                    /* bmFormats D0 - PCM */
    0x02,                                   /* bNrChannels */
    0x00,0x00,0x00,0x00,                    /* bmChannelConfig 0x00000000 driver choose channels */
    0x00,                                   /* iChannelNames */
    /* 16 byte */

    /* AudioStreaming Interface Descriptor - Audio USB_OUT Format */
    0x6,                                    /* bLength */
    USB_AUDIO_V2_CS_INTERFACE,              /* bDescriptorType */
    AUDIO_STREAMING_FORMAT_TYPE,            /* bDescriptorSubtype */
    USB_AUDIO_V2_FORMAT_TYPE_I,             /* bFormatType */
    0x04,                                   /* bSubslotSize :  4 Bytes per frame (32bits) */
    32,                                     /* bBitResolution (32-bits per sample) */
    /* 06 byte */ 

    /* Endpoint Descriptor - STD AS ISO OUT Endpoint */
    0x07,                                   /* bLength */
    USB_DESC_TYPE_ENDPOINT,                 /* bDescriptorType */
    AUDIO_OUT_EP,                           /* bEndpointAddress 1 out endpoint */
    0x05,                                   /* bmAttributes Asynchronous Isochronous mode */
    USB_WORD(1023),                              /* wMaxPacketSize in Bytes 1024 per micro-frame */
    0x01,                                   /* bInterval (Frames/Subframes) 2^(1-1)=1 */
    /* 07 byte */
    
    /* AudioControl Endpoint Descriptor - CS AS ISO OUT Endpoint */
    0x08,                                   /* bLength */
    USB_AUDIO_V2_CS_ENDPOINT,               /* bDescriptorType */
    USB_AUDIO_V2_EP_GENERAL,                /* bDescriptorSubtype */
    0x00,                                   /* bmAttributes */
    0x00,                                   /* bmControls */
    0x00,                                   /* bLockDelayUnits - Undefined */
    0x00,0x00,                              /* wLockDelay */
    /* 08 byte */
    
    /* Endpoint Descriptor - STD AS ISO Feedback Endpoint */
    0x07,                                   /* bLength */
    USB_DESC_TYPE_ENDPOINT,                 /* bDescriptorType */
    AUDIO_FB_EP,                            /* bEndpointAddress 1 int endpoint */
    0x11,                                   /* bmAttributes FeedBack Isochronous mode */
    0x04,0x00,                              /* wMaxPacketSize in Bytes 4 per micro-frame */
    0x03,                                   /* bInterval (Frames/Subframes) 2^(3-1)=4 */
    /* 07 byte */
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
    0x02,           // clock attribute (internal fixed clock, no sof sync)
    0x03,           // controls (no frequency control, no valid control)
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
    0x01, 0x00,  // wLockDelay (1 ms)
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
    20, 0x03, UC('2'), UC('0'), UC('2'), UC('5'), UC('-'), UC('5'), UC('-'), UC('2'), UC('7')
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
