#include "usb_desc.h"

/* Device Descriptor */
const uint8_t MyDevDescr[] =
{
    0x12,                    // bLength
    0x01,                    // bDescriptorType (Device)
    0x00, 0x02,              // bcdUSB 2.00
    0x00,                    // bDeviceClass
    0x00,                    // bDeviceSubClass
    0x00,                    // bDeviceProtocol
    DEF_USBD_UEP0_SIZE,      // bMaxPacketSize0 64
    USB_WORD (DEF_USB_VID),  // idVendor 0x1A86
    USB_WORD (DEF_USB_PID),  // idProduct 0x5537
    DEF_IC_PRG_VER, 0x00,    // bcdDevice 0.01
    0x01,                    // iManufacturer (String Index)
    0x02,                    // iProduct (String Index)
    0x03,                    // iSerialNumber (String Index)
    0x01,                    // bNumConfigurations 1
};

/* HID Report Descriptor */
const uint8_t MyHIDReportDesc_HS[] =
{
    0x06, 0x00, 0xff,       // Usage Page (Vendor Defined)
    0x09, 0x01,             // Usage (Vendor Usage 1)
    0xA1, 0x01,             // Collection (Application)
    0x09, 0x02,             //   usage 2
    0x15, 0x00,             //   Logical Minimum (0)
    0x25, 0xFF,             //   Logical Maximum (255)
    0x75, 32,               //   Report Size (32)
    0x95, 65,               //   Report Count (65)
    0x91, 0x02,             //   Output (Data, Variable, Absolute)
    0x09, 0x03,             //   usage 3
    0x15, 0x00,             //   Logical Minimum (0)
    0x25, 0xFF,             //   Logical Maximum (255)
    0x75, 0x08,             //   Report Size (8)
    0x95, 0x1,             //   Report Count (1)
    0x81, 0x02,             //   Input (Data, Variable, Absolute)
    0xC0                    // End Collection
};

/* Configuration Descriptor */
const uint8_t MyCfgDescr_FS[] =
{
    0x09,           // bLength
    0x02,           // bDescriptorType (Configuration)
    USB_WORD(9),    // wTotalLength
    0x00,           // bNumInterfaces
    0x01,           // bConfigurationValue
    0x00,           // iConfiguration (String Index)
    0x80,           // bmAttributes
    250,            // bMaxPower 100mA
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
    40, 0x03,
    UC('C'), UC('H'), UC('3'), UC('2'), UC('-'), UC('U'), UC('A'), UC('C'), UC('-'),
    UC('B'), UC('O'), UC('O'), UC('T'), UC('L'), UC('O'), UC('A'), UC('D'), UC('E'), UC('R')
};

/* Serial Number Information */
const uint8_t MySerNumInfo[] =
{
    20, 0x03, UC('2'), UC('0'), UC('2'), UC('5'), UC('-'), UC('6'), UC('-'), UC('1'), UC('8')
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
uint8_t TAB_USB_FS_OSC_DESC[41] =
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
