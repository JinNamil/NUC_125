/******************************************************************************
 * @file     HID_Transfer_and_Keyboard.h
 * @brief    NUC121 series USB composite device header file
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_HID_MASS_H__
#define __USBD_HID_MASS_H__

#include <stdint.h>

/* Define the vendor name */
#define AUTHENTICATOR_VENDOR_EWBM    (1)
#define AUTHENTICATOR_VENDOR_RAON    (2)
#define AUTHENTICATOR_VENDOR         (AUTHENTICATOR_VENDOR_EWBM)

/* Define the vendor id and product id */
#define USBD_VID            (0xE383)
#define USBD_PID            (0x0007)

/*!<Define HID Class Specific Request */
#define GET_REPORT          (0x01)
#define GET_IDLE            (0x02)
#define GET_PROTOCOL        (0x03)
#define SET_REPORT          (0x09)
#define SET_IDLE            (0x0A)
#define SET_PROTOCOL        (0x0B)

/*!<USB HID Interface Class protocol */
#define HID_NONE            (0x00)
#define HID_KEYBOARD        (0x01)
#define HID_MOUSE           (0x02)

/*!<USB HID Class Report Type */
#define HID_RPT_TYPE_INPUT      (0x01)
#define HID_RPT_TYPE_OUTPUT     (0x02)
#define HID_RPT_TYPE_FEATURE    (0x03)

/*!<Define Mass Storage Class Specific Request */
#define BULK_ONLY_MASS_STORAGE_RESET    (0xFF)
#define GET_MAX_LUN                     (0xFE)

/*!<Define Mass Storage Signature */
#define CBW_SIGNATURE       (0x43425355)
#define CSW_SIGNATURE       (0x53425355)

/*!<Define Mass Storage UFI Command */
#define UFI_TEST_UNIT_READY                     (0x00)
#define UFI_REQUEST_SENSE                       (0x03)
#define UFI_INQUIRY                             (0x12)
#define UFI_MODE_SELECT_6                       (0x15)
#define UFI_MODE_SENSE_6                        (0x1A)
#define UFI_START_STOP                          (0x1B)
#define UFI_PREVENT_ALLOW_MEDIUM_REMOVAL        (0x1E)
#define UFI_READ_FORMAT_CAPACITY                (0x23)
#define UFI_READ_CAPACITY                       (0x25)
#define UFI_READ_10                             (0x28)
#define UFI_READ_12                             (0xA8)
#define UFI_WRITE_10                            (0x2A)
#define UFI_WRITE_12                            (0xAA)
#define UFI_VERIFY_10                           (0x2F)
#define UFI_MODE_SELECT_10                      (0x55)
#define UFI_MODE_SENSE_10                       (0x5A)
#define UFI_READ_CAPACITY_16                    (0x9E)

/*-----------------------------------------*/
#define BULK_CBW    (0x00)
#define BULK_IN     (0x01)
#define BULK_OUT    (0x02)
#define BULK_CSW    (0x04)
#define BULK_NORMAL (0xFF)

/*-------------------------------------------------------------*/
/* Define EP maximum packet size */
#define EP0_MAX_PKT_SIZE    (8)
#define EP1_MAX_PKT_SIZE    (8)
#define EP2_MAX_PKT_SIZE    (64)
#define EP3_MAX_PKT_SIZE    (64)
#define EP4_MAX_PKT_SIZE    (64)
#define EP5_MAX_PKT_SIZE    (64)
#define EP6_MAX_PKT_SIZE    (64)
#define EP7_MAX_PKT_SIZE    (64)

#define SETUP_BUF_BASE      (0)
#define SETUP_BUF_LEN       (8)
#define EP0_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP0_BUF_LEN         (EP0_MAX_PKT_SIZE)
#define EP1_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP1_BUF_LEN         (EP1_MAX_PKT_SIZE)
#define EP2_BUF_BASE        (EP1_BUF_BASE + EP1_BUF_LEN)
#define EP2_BUF_LEN         (EP2_MAX_PKT_SIZE)
#define EP3_BUF_BASE        (EP2_BUF_BASE + EP2_BUF_LEN)
#define EP3_BUF_LEN         (EP3_MAX_PKT_SIZE)
#define EP4_BUF_BASE        (EP3_BUF_BASE + EP3_BUF_LEN)
#define EP4_BUF_LEN         (EP4_MAX_PKT_SIZE)
#define EP5_BUF_BASE        (EP4_BUF_BASE + EP4_BUF_LEN)
#define EP5_BUF_LEN         (EP5_MAX_PKT_SIZE)
#define EP6_BUF_BASE        (EP5_BUF_BASE + EP5_BUF_LEN)
#define EP6_BUF_LEN         (EP6_MAX_PKT_SIZE)
#define EP7_BUF_BASE        (EP6_BUF_BASE + EP6_BUF_LEN)
#define EP7_BUF_LEN         (EP7_MAX_PKT_SIZE)

/* Define the EP numbers */
#define HID_IN_EP_NUM       (0x02)
#define HID_OUT_EP_NUM      (0x03)
#define MSC_IN_EP_NUM       (0x04)
#define MSC_OUT_EP_NUM      (0x05)
#define BULK_IN_EP_NUM      (0x06)
#define BULK_OUT_EP_NUM     (0x07)

/* Define Descriptor information */
#define HID_DEFAULT_INT_IN_INTERVAL     (4)
#define USBD_SELF_POWERED               (0)
#define USBD_REMOTE_WAKEUP              (0)
#define USBD_MAX_POWER                  (50)  /* The unit is in 2mA. ex: 50 * 2mA = 100mA */

/** Configuration Descriptor Data */
#if defined(NO_MASS_STORAGE) && defined(NO_BULK_TRANSFER)
    #define CONFIG_DESCRIPTOR_TOTAL_LENGTH    (LEN_CONFIG+LEN_INTERFACE+LEN_HID+LEN_ENDPOINT+LEN_ENDPOINT)
    #define CONFIG_INTERFACES_NUM             (0x01)
#elif !defined(NO_MASS_STORAGE) && !defined(NO_BULK_TRANSFER)
    #define CONFIG_DESCRIPTOR_TOTAL_LENGTH    (LEN_CONFIG+LEN_INTERFACE+LEN_HID+LEN_ENDPOINT+LEN_ENDPOINT+LEN_INTERFACE+LEN_ENDPOINT+LEN_ENDPOINT+LEN_INTERFACE+LEN_ENDPOINT+LEN_ENDPOINT)
    #define CONFIG_INTERFACES_NUM             (0x03)
#else
    #define CONFIG_DESCRIPTOR_TOTAL_LENGTH    (LEN_CONFIG+LEN_INTERFACE+LEN_HID+LEN_ENDPOINT+LEN_ENDPOINT+LEN_INTERFACE+LEN_ENDPOINT+LEN_ENDPOINT)
    #define CONFIG_INTERFACES_NUM             (0x02)
#endif

/* HIRC trim setting:
 *    HIRC trim reference clock is from USB SOF (Start-Of-Frame) packet.
 *    HIRC trim operation is keep going if clock is inaccuracy.
 *    HIRC Trim retry count limitation is 512 loops.
 *    Trim value calculation is based on average difference in 4 clocks of reference clock.
 *    Enable HIRC auto trim function and trim HIRC to 48 MHz.
 */
#define DEFAULT_HIRC_TRIM_SETTING    ((0x1ul<<SYS_IRCTCTL_REFCKSEL_Pos)| (0x0ul<<SYS_IRCTCTL_CESTOPEN_Pos)| (0x3ul<<SYS_IRCTCTL_RETRYCNT_Pos)| (0x0ul<<SYS_IRCTCTL_LOOPSEL_Pos) | (0x2ul<<SYS_IRCTCTL_FREQSEL_Pos))

static __INLINE uint32_t get_be32(uint8_t *buf)
{
    return ((uint32_t) buf[0] << 24) | ((uint32_t) buf[1] << 16) | ((uint32_t) buf[2] << 8)  | ((uint32_t) buf[3]);
}

#ifndef NO_MASS_STORAGE
/******************************************************************************/
/*                USBD Mass Storage Structure                                 */
/******************************************************************************/
/** @addtogroup USBD_Mass_Exported_Struct  USBD Mass Exported Struct
  USBD Mass Specific Struct
  @{
*/

/*!<USB Mass Storage Class - Command Block Wrapper Structure */
struct CBW {
    uint32_t  dCBWSignature;
    uint32_t  dCBWTag;
    uint32_t  dCBWDataTransferLength;
    uint8_t   bmCBWFlags;
    uint8_t   bCBWLUN;
    uint8_t   bCBWCBLength;
    uint8_t   u8OPCode;
    uint8_t   u8LUN;
    uint8_t   au8Data[14];
};

/*!<USB Mass Storage Class - Command Status Wrapper Structure */
struct CSW {
    uint32_t  dCSWSignature;
    uint32_t  dCSWTag;
    uint32_t  dCSWDataResidue;
    uint8_t   bCSWStatus;
};

/*-------------------------------------------------------------*/
#define MASS_BUFFER_SIZE    256 /* Mass Storage command buffer size */
#define STORAGE_BUFFER_SIZE 512 /* Data transfer buffer size in 512 bytes alignment */
#define UDC_SECTOR_SIZE     512 /* logic sector size */

extern uint32_t MassBlock[];
extern uint32_t Storage_Block[];

#define MassCMD_BUF        ((uint32_t)&MassBlock[0])
#define STORAGE_DATA_BUF   ((uint32_t)&Storage_Block[0])
#endif

/*-------------------------------------------------------------*/
void EP2_Handler(void);
void EP3_Handler(void);

#ifndef NO_MASS_STORAGE
void EP4_Handler(void);
void EP5_Handler(void);
#endif

#ifndef NOT_USE_BULK_TRASFER
void EP6_Handler(void);
void EP7_Handler(void);
#endif

void HID_MSC_Init(void);
void HID_MSC_ClassRequest(void);
void HID_MSC_ProcessCmd(void);

void HID_AckCmd(void);

#ifndef NO_BULK_TRANSFER
void BULK_AckCmd(void);
#endif

#ifndef NO_MASS_STORAGE
void MSC_RequestSense(void);
void MSC_ReadFormatCapacity(void);
void MSC_Read(void);
void MSC_ReadCapacity(void);
void MSC_ReadCapacity16(void);
void MSC_Write(void);
void MSC_ModeSense10(void);
void MSC_ReadTrig(void);
void MSC_SetConfig(void);
void MSC_ReadMedia(uint32_t addr, uint32_t size, uint32_t buffer);
void MSC_WriteMedia(uint32_t addr, uint32_t size, uint32_t buffer);
void MSC_AckCmd(void);
#endif

#endif  /* __USBD_HID_H_ */

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
