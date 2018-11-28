
#include "NUC121.h"
#include "HID_FIDO_and_MassStorage.h"
#include "descriptors.h"
/******************************************************************************//**
 * @file     descriptors.c
 * @version  V3.00
 * @brief    USBD descriptors
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
 
const uint8_t HID_DeviceReportDescriptor[] =
{
    HID_UsagePage ( FIDO_USAGE_PAGE ),
    HID_Usage ( FIDO_USAGE_CTAPHID ),
    HID_Collection ( HID_Application ),
    HID_Usage ( FIDO_USAGE_DATA_IN ),
    HID_LogicalMin ( 0 ),
    HID_LogicalMaxS ( 0xff ),
    HID_ReportSize ( 8 ),
    HID_ReportCount ( HID_INPUT_REPORT_BYTES ),
    HID_Input ( HID_Data | HID_Absolute | HID_Variable ),
    HID_Usage ( FIDO_USAGE_DATA_OUT ),
    HID_LogicalMin ( 0 ),
    HID_LogicalMaxS ( 0xff ),
    HID_ReportSize ( 8 ),
    HID_ReportCount ( HID_OUTPUT_REPORT_BYTES ),
    HID_Output ( HID_Data | HID_Absolute | HID_Variable ),
    HID_EndCollection
};

/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
const uint8_t gu8DeviceDescriptor[] =
{
    LEN_DEVICE,         /* bLength */
    DESC_DEVICE,        /* bDescriptorType */
#ifdef SUPPORT_LPM
    WBVAL(0x0201),      /* bcdUSB => 0x0201 to support LPM */
#else
    WBVAL(0x0110),      /* bcdUSB */
#endif
    0x00,               /* bDeviceClass */
    0x00,               /* bDeviceSubClass */
    0x00,               /* bDeviceProtocol */
    EP0_MAX_PKT_SIZE,   /* bMaxPacketSize0 */
    WBVAL(USBD_VID),    /* idVendor */
    WBVAL(USBD_PID),    /* idProduct */
    0x00, 0x00,         /* bcdDevice */
    0x01,               /* iManufacture */
    0x02,               /* iProduct */
    0x03,               /* iSerialNumber */
    0x01                /* bNumConfigurations */
};

/*!<USB Configure Descriptor */
const uint8_t gu8ConfigDescriptor[] =
{
    LEN_CONFIG,                                                  /* bLength */
    DESC_CONFIG,                                                 /* bDescriptorType */
    WBVAL(CONFIG_DESCRIPTOR_TOTAL_LENGTH),                       /* wTotalLength */
    CONFIG_INTERFACES_NUM,                                       /* bNumInterfaces */
    0x01,                                                        /* bConfigurationValue */
    0x00,                                                        /* iConfiguration */
    0x80 | (USBD_SELF_POWERED << 6) | (USBD_REMOTE_WAKEUP << 5), /* bmAttributes */
    USBD_MAX_POWER,                                              /* MaxPower */

    /* I/F descr: HID */
    LEN_INTERFACE,  /* bLength */
    DESC_INTERFACE, /* bDescriptorType */
    0x00,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x02,           /* bNumEndpoints */
    0x03,           /* bInterfaceClass */
    0x00,           /* bInterfaceSubClass */
    0x00,           /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* HID Descriptor */
    LEN_HID,                                   /* Size of this descriptor in UINT8s. */
    DESC_HID,                                  /* HID descriptor type. */
    WBVAL(0x0001),                             /* HID Class Spec. release number. 00.01 */
    0x00,                                      /* H/W target country. */
    0x01,                                      /* Number of HID class descriptors to follow. */
    DESC_HID_RPT,                              /* Descriptor type. */
    WBVAL(sizeof(HID_DeviceReportDescriptor)), /* Total length of report descriptor. */

    /* EP Descriptor: interrupt in. */
    LEN_ENDPOINT,                       /* bLength */
    DESC_ENDPOINT,                      /* bDescriptorType */
    (EP_INPUT | HID_IN_EP_NUM),         /* bEndpointAddress */
    EP_INT,                             /* bmAttributes */
    WBVAL(EP2_MAX_PKT_SIZE),            /* wMaxPacketSize */
    HID_DEFAULT_INT_IN_INTERVAL,        /* bInterval */

    /* EP Descriptor: interrupt out. */
    LEN_ENDPOINT,                       /* bLength */
    DESC_ENDPOINT,                      /* bDescriptorType */
    (EP_OUTPUT | HID_OUT_EP_NUM),       /* bEndpointAddress */
    EP_INT,                             /* bmAttributes */
    WBVAL(EP3_MAX_PKT_SIZE),            /* wMaxPacketSize */
    HID_DEFAULT_INT_IN_INTERVAL         /* bInterval */

#ifndef NO_MASS_STORAGE
    /* MSC Descriptor */
    /* const BYTE cbyInterfaceDescriptor[LEN_INTERFACE] = */
    ,LEN_INTERFACE, /* bLength */
    DESC_INTERFACE, /* bDescriptorType */
    0x01,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x02,           /* bNumEndpoints */
    0x08,           /* bInterfaceClass */
    0x06,           /* bInterfaceSubClass */
    0x50,           /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* const BYTE cbyEndpointDescriptor1[LEN_ENDPOINT] = */
    LEN_ENDPOINT,                       /* bLength */
    DESC_ENDPOINT,                      /* bDescriptorType */
    (EP_INPUT | MSC_IN_EP_NUM),         /* bEndpointAddress */
    EP_BULK,                            /* bmAttributes */
    WBVAL(EP4_MAX_PKT_SIZE),            /* wMaxPacketSize */
    0x00,                               /* bInterval */

    /* const BYTE cbyEndpointDescriptor2[LEN_ENDPOINT] = */
    LEN_ENDPOINT,                       /* bLength */
    DESC_ENDPOINT,                      /* bDescriptorType */
    (EP_OUTPUT | MSC_OUT_EP_NUM),       /* bEndpointAddress */
    EP_BULK,                            /* bmAttributes */
    WBVAL(EP5_MAX_PKT_SIZE),            /* wMaxPacketSize */
    0x00                                /* bInterval */
#endif

#ifndef NO_BULK_TRANSFER
    /* BULK Descriptor */
    /* const BYTE cbyInterfaceDescriptor[LEN_INTERFACE] = */
    ,LEN_INTERFACE, /* bLength */
    DESC_INTERFACE, /* bDescriptorType */
    0x02,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x02,           /* bNumEndpoints */
    0xFF,           /* bInterfaceClass */
    0xFF,           /* bInterfaceSubClass */
    0xFF,           /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* const BYTE cbyEndpointDescriptor1[LEN_ENDPOINT] = */
    LEN_ENDPOINT,                       /* bLength */
    DESC_ENDPOINT,                      /* bDescriptorType */
    (EP_INPUT | BULK_IN_EP_NUM),        /* bEndpointAddress */
    EP_BULK,                            /* bmAttributes */
    EP6_MAX_PKT_SIZE, 0x00,             /* wMaxPacketSize */
    0x00,                               /* bInterval */

    /* const BYTE cbyEndpointDescriptor2[LEN_ENDPOINT] = */
    LEN_ENDPOINT,                       /* bLength */
    DESC_ENDPOINT,                      /* bDescriptorType */
    (EP_OUTPUT | BULK_OUT_EP_NUM),      /* bEndpointAddress */
    EP_BULK,                            /* bmAttributes */
    EP7_MAX_PKT_SIZE, 0x00,             /* wMaxPacketSize */
    0x00                                /* bInterval */
#endif
};

#ifdef SUPPORT_LPM
/*!<USB BOS Descriptor */
const uint8_t gu8BosDescriptor[] =
{
    LEN_BOS,                         /* bLength */
    DESC_BOS,                        /* bDescriptorType */
    WBVAL(LEN_BOS + LEN_DEVCAP),     /* wTotalLength */
    0x01,                            /* bNumDevcieCaps */
    LEN_DEVCAP,                      /* bLength */
    DESC_DEVCAP,                     /* bDescriptorType */
    0x02,                            /* bDevCapabilityType, 0x02 is USB 2.0 Extension */
    0x06, 0x04, 0x00, 0x00           /* bmAttributs, 32 bits                                              */
    /*                                  bit 0 : Reserved. Must 0.                                         */
    /*                                  bit 1 : 1 to support LPM.                                         */
    /*                                  bit 2 : 1 to support BSL & Alternat HIRD                          */
    /*                                  bit 3 : 1 to recommend Baseline BESL                              */
    /*                                  bit 4 : 1 to recommand Deep BESL                                  */
    /*                                  bit 11:8 : Recommend Baseline BESL value. Ignore by bit3 is zero. */
    /*                                  bit 15:12 : Recommend Deep BESL value. Ignore by bit4 is zero.    */
    /*                                  bit 31:16 : Reserved. Must 0.                                     */
};
#endif


/*!<USB Language String Descriptor */
const uint8_t gu8StringLang[4] =
{
    4,              /* bLength */
    DESC_STRING,    /* bDescriptorType */
    0x09, 0x04
};

//!<USB Vendor String Descriptor
const uint8_t gu8VendorStringDesc[] =
{
#if ( AUTHENTICATOR_VENDOR == AUTHENTICATOR_VENDOR_EWBM )
    10,           /* bLength          */
    DESC_STRING,  /* bDescriptorType  */
    'e', 0,
    'W', 0,
    'B', 0,
    'M', 0
#elif ( AUTHENTICATOR_VENDOR == AUTHENTICATOR_VENDOR_RAON )
    22,           /* bLength          */
    DESC_STRING,  /* bDescriptorType  */
    'R', 0,
    'A', 0,
    'O', 0,
    'N', 0,
    'S', 0,
    'E', 0,
    'C', 0,
    'U', 0,
    'R', 0,
    'E', 0
#endif
};


//!<USB Product String Descriptor
const uint8_t gu8ProductStringDesc[] =
{
#if ( AUTHENTICATOR_VENDOR == AUTHENTICATOR_VENDOR_EWBM )
    20,             /* bLength          */
    DESC_STRING,    /* bDescriptorType  */
    'e', 0,
    'W', 0,
    'B', 0,
    'M', 0,
    ' ', 0,
    'F', 0,
    'I', 0,
    'D', 0,
    'O', 0,
#elif ( AUTHENTICATOR_VENDOR == AUTHENTICATOR_VENDOR_RAON )
    82,             /* bLength          */
    DESC_STRING,    /* bDescriptorType  */
    'T', 0,
    'o', 0,
    'u', 0,
    'c', 0,
    'h', 0,
    'E', 0,
    'n', 0,
    ' ', 0,
    'O', 0,
    'n', 0,
    'e', 0,
    'P', 0,
    'a', 0,
    's', 0,
    's', 0,
    ' ', 0,
    'F', 0,
    'I', 0,
    'D', 0,
    'O', 0,
    '2', 0,
    ' ', 0,
    'A', 0,
    'u', 0,
    't', 0,
    'h', 0,
    'e', 0,
    'n', 0,
    't', 0,
    'i', 0,
    'c', 0,
    'a', 0,
    't', 0,
    'o', 0,
    'r', 0,
    ' ', 0,
    'v', 0,
    '1', 0,
    '.', 0,
    '0', 0
#endif
};

uint8_t gu8StringSerial[26] =
{
    26,             // bLength
    DESC_STRING,    // bDescriptorType
    'A', 0,
    '0', 0,
    '0', 0,
    '0', 0,
    '0', 0,
    '0', 0,
    '0', 0,
    '0', 0,
    '0', 0,
    '0', 0,
    '0', 0,
    '0', 0
};

const uint8_t *gpu8UsbString[4] =
{
    gu8StringLang,
    gu8VendorStringDesc,
    gu8ProductStringDesc,
    gu8StringSerial
};

const uint8_t *gpu8UsbHidReport[3] =
{
    HID_DeviceReportDescriptor,
    NULL,
    NULL
};

const uint32_t gu32UsbHidReportLen[3] =
{
    sizeof(HID_DeviceReportDescriptor),
    0,
    0
};

const uint32_t gu32ConfigHidDescIdx[3] =
{
    (LEN_CONFIG + LEN_INTERFACE),
    0,
    0
};

const S_USBD_INFO_T gsInfo =
{
    gu8DeviceDescriptor,
    gu8ConfigDescriptor,
    gpu8UsbString,
    gpu8UsbHidReport,
    gu32UsbHidReportLen,
    gu32ConfigHidDescIdx,
#ifdef SUPPORT_LPM
    gu8BosDescriptor
#endif
};
