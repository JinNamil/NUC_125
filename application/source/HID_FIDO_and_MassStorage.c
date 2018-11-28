/*!<Includes */
#include <stdio.h>
#include <string.h>
#include "NUC121.h"

#ifndef NO_MASS_STORAGE
#include "SPIFlash.h"
#endif

#include "USPIComm.h"
#include "HID_FIDO_and_MassStorage.h"

#if 0
    #define DBG_PRINTF      printf
#else
    #define DBG_PRINTF(...)
#endif

#define HID_PACKET              (0x80)
#define HID_CTAP_PING           (HID_PACKET | 0x01)
#define HID_CTAP_MSG            (HID_PACKET | 0x03)
#define HID_CTAP_LOCK           (HID_PACKET | 0x04)
#define HID_CTAP_INIT           (HID_PACKET | 0x06)
#define HID_CTAP_WINK           (HID_PACKET | 0x08)
#define HID_CTAP_CBOR           (HID_PACKET | 0x10)
#define HID_CTAP_CANCEL         (HID_PACKET | 0x11)
#define HID_CTAP_KEEPALIVE      (HID_PACKET | 0x3B)
#define HID_CTAP_ERROR          (HID_PACKET | 0x3F)

#define HID_USER_UAF            (HID_PACKET | 0x41)
#define HID_USER_FINGERPRINT    (HID_PACKET | 0x51)
#define HID_USER_UTIL           (HID_PACKET | 0x52)
#define HID_USER_BULK           (HID_PACKET | 0x53)

/*--------------------------------------------------------------------------*/
/* Global variables for Control Pipe */

uint8_t volatile g_u8EP2Ready = 1;
uint8_t volatile g_u8EP3Ready = 0;

#ifndef NO_MASS_STORAGE
uint8_t volatile g_u8EP4Ready = 0;
uint8_t volatile g_u8EP5Ready = 0;
#endif

#ifndef NO_BULK_TRANSFER
uint8_t volatile g_u8EP6Ready = 0;
uint8_t volatile g_u8EP7Ready = 0;
#endif

uint8_t volatile g_u8BP14Ready = 0;

uint8_t commBuf[64] = {0};

#ifndef NO_MASS_STORAGE
/* USB flow control variables */
int32_t  g_TotalSectors = 0;
uint8_t  g_u8BulkState;
uint8_t  g_u8Prevent = 0;
uint8_t  g_u8Size;
uint8_t  g_au8SenseKey[4];
uint32_t g_u32SPIFlashStartAddr;
uint32_t g_u32Address;
uint32_t g_u32Length;
uint32_t g_u32LbaAddress;
uint32_t g_u32BytesInStorageBuf;
uint32_t g_u32BulkBuf0, g_u32BulkBuf1;

/* CBW/CSW variables */
struct CBW g_sCBW;
struct CSW g_sCSW;

uint32_t MassBlock[MASS_BUFFER_SIZE/4];
uint32_t Storage_Block[STORAGE_BUFFER_SIZE/4];
#endif


/*--------------------------------------------------------------------------*/
#ifndef NO_MASS_STORAGE
uint8_t g_au8InquiryID[36] =
{
    0x00,                   /* Peripheral Device Type */
    0x80,                   /* RMB */
    0x00,                   /* ISO/ECMA, ANSI Version */
    0x00,                   /* Response Data Format */
    0x1F, 0x00, 0x00, 0x00, /* Additional Length */

    /* Vendor Identification */
    0, 0, 0, 0, 0, 0, 0, 0,

    /* Product Identification */
    'U', 'S', 'B', ' ', 'M', 'a', 's', 's', ' ', 'S', 't', 'o', 'r', 'a', 'g', 'e',

    /* Product Revision */
    '0', '.', '0', '1'
};

// code = 5Ah, Mode Sense
static uint8_t g_au8ModePage_01[12] =
{
    0x01,
    0x0A,
    0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00
};

static uint8_t g_au8ModePage_05[32] =
{
    0x05,
    0x1E,
    0x13, 0x88, 0x08, 0x20, 0x02, 0x00, 0x01, 0xF4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x68, 0x00, 0x00
};

static uint8_t g_au8ModePage_1B[12] =
{
    0x1B,
    0x0A,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint8_t g_au8ModePage_1C[8] =
{
    0x1C,
    0x06,
    0x00, 0x05, 0x00, 0x00, 0x00, 0x00
};
#endif

void GPAB_IRQHandler(void)
{
    if ( GPIO_GET_INT_FLAG(PB, BIT14) )
    {
        GPIO_CLR_INT_FLAG(PB, BIT14);
        g_u8BP14Ready = 1;
    }
    else
    {
        PA->INTSRC = PA->INTSRC;
        PB->INTSRC = PB->INTSRC;
    }
}

void USBD_IRQHandler(void)
{
    uint32_t u32IntSts = USBD_GET_INT_FLAG();
    uint32_t u32State = USBD_GET_BUS_STATE();

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_FLDET)
    {
        // Floating detect
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);

        if (USBD_IS_ATTACHED())
        {
            /* USB Plug In */
            USBD_ENABLE_USB();

            /*Enable HIRC tirm*/
            SYS->IRCTCTL = DEFAULT_HIRC_TRIM_SETTING;
        }
        else
        {
            /* USB Un-plug */
            USBD_DISABLE_USB();

            /*Disable HIRC tirm*/
            SYS->IRCTCTL = DEFAULT_HIRC_TRIM_SETTING & (~SYS_IRCTCTL_FREQSEL_Msk);
        }
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_BUS)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);

        if (u32State & USBD_STATE_USBRST)
        {
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();

            /*Enable HIRC tirm*/
            SYS->IRCTCTL = DEFAULT_HIRC_TRIM_SETTING;
        }

        if (u32State & USBD_STATE_SUSPEND)
        {
            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();

            /*Disable HIRC tirm*/
            SYS->IRCTCTL = DEFAULT_HIRC_TRIM_SETTING & (~SYS_IRCTCTL_FREQSEL_Msk);
        }

        if (u32State & USBD_STATE_RESUME)
        {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();

            /*Enable HIRC tirm*/
            SYS->IRCTCTL = DEFAULT_HIRC_TRIM_SETTING;
        }

#ifdef SUPPORT_LPM
        if (u32State & USBD_ATTR_L1SUSPEND_Msk)
        {
            /*
               TODO: Implement LPM SUSPEND flag here.
                     Remember to implement the power-saving function in main loop.
            */
        }

        if (u32State & USBD_ATTR_L1RESUME_Msk)
        {
            /*
               TODO: Implement LPM RESUME flag here.
            */
        }
#endif
    }

    if (u32IntSts & USBD_INTSTS_NEVWKIF_Msk)
    {
        /*Clear no-event wake up interrupt */
        USBD_CLR_INT_FLAG(USBD_INTSTS_NEVWKIF_Msk);
        /*
           TODO: Implement the function that will be executed when device is woken by non-USB event.
        */
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_USB)
    {
        // EP events
        if (u32IntSts & USBD_INTSTS_EP0)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);
            // control IN
            USBD_CtrlIn();
        }

        if (u32IntSts & USBD_INTSTS_EP1)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);

            // control OUT
            USBD_CtrlOut();
        }

        if (u32IntSts & USBD_INTSTS_EP2)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);
            EP2_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
            EP3_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP4)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);
#ifndef NO_MASS_STORAGE
            EP4_Handler();
#endif
        }

        if (u32IntSts & USBD_INTSTS_EP5)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);
#ifndef NO_MASS_STORAGE
            EP5_Handler();
#endif
        }

        if (u32IntSts & USBD_INTSTS_EP6)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);
#ifndef NO_BULK_TRANSFER
            EP6_Handler();
#endif
        }

        if (u32IntSts & USBD_INTSTS_EP7)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
#ifndef NO_BULK_TRANSFER
            EP7_Handler();
#endif
        }

        // USB event
        if (u32IntSts & USBD_INTSTS_SETUP)
        {
            // Setup packet
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);

            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);

            USBD_ProcessSetupPacket();
        }
    }
}

void EP2_Handler(void)
{
    g_u8EP2Ready = 1;
}

void EP3_Handler(void)
{
    g_u8EP3Ready = 1;
    memset(commBuf, 0x0, EP3_MAX_PKT_SIZE);
    USBD_MemCopy(commBuf, (uint8_t*)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3)), EP3_MAX_PKT_SIZE);
    USPICommWrite(commBuf, EP3_MAX_PKT_SIZE);
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
}

#ifndef NO_MASS_STORAGE
void EP4_Handler(void)
{
    g_u8EP4Ready = 1;
    MSC_AckCmd();
}

void EP5_Handler(void)
{
    g_u8EP5Ready = 1;
}
#endif

#ifndef NO_BULK_TRANSFER
void EP6_Handler(void)
{
    g_u8EP6Ready = 1;
}

void EP7_Handler(void)
{	
    g_u8EP7Ready = 1;
    memset(commBuf, 0x0, EP7_MAX_PKT_SIZE);
    USBD_MemCopy(commBuf, (uint8_t*)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP7)), EP7_MAX_PKT_SIZE);
    USPICommWrite(commBuf, EP7_MAX_PKT_SIZE);
    USBD_SET_PAYLOAD_LEN(EP7, EP7_MAX_PKT_SIZE);
}
#endif

void HID_MSC_Init(void)
{
    int32_t i;
    uint8_t *pu8;
    uint8_t pSerial[] = __TIME__;

    /* Init setup packet buffer */
    /* Buffer range for SETUP packet -> [0 ~ 0x7] */
    USBD->STBUFSEG = SETUP_BUF_BASE;

    /*****************************************************/
    /* EP0 ==> control IN endpoint, address 0 */
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    /* Buffer range for EP0 */
    USBD_SET_EP_BUF_ADDR(EP0, EP0_BUF_BASE);

    /* EP1 ==> control OUT endpoint, address 0 */
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    /* Buffer range for EP1 */
    USBD_SET_EP_BUF_ADDR(EP1, EP1_BUF_BASE);

    /*****************************************************/
    /* EP2 ==> Interrupt IN endpoint, address 1 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | HID_IN_EP_NUM);
    /* Buffer range for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /* EP3 ==> Interrupt OUT endpoint, address 2 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | HID_OUT_EP_NUM);
    /* Buffer range for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);
    /* trigger to receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

#ifndef NO_MASS_STORAGE
    /*****************************************************/
    /* EP4 ==> Bulk IN endpoint, address 3 */
    USBD_CONFIG_EP(EP4, USBD_CFG_EPMODE_IN | MSC_IN_EP_NUM);
    /* Buffer range for EP4 */
    USBD_SET_EP_BUF_ADDR(EP4, EP4_BUF_BASE);

    /* EP5 ==> Bulk Out endpoint, address 4 */
    USBD_CONFIG_EP(EP5, USBD_CFG_EPMODE_OUT | MSC_OUT_EP_NUM);
    /* Buffer range for EP5 */
    USBD_SET_EP_BUF_ADDR(EP5, EP5_BUF_BASE);
    /* trigger to receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP5, EP5_MAX_PKT_SIZE);

    /*****************************************************/
    g_u32BulkBuf0 = EP5_BUF_BASE;
    g_u32BulkBuf1 = EP4_BUF_BASE;

    g_sCSW.dCSWSignature = CSW_SIGNATURE;
    g_TotalSectors = SPI_FLASH_STORAGE_SIZE / UDC_SECTOR_SIZE;
#endif

#ifndef NO_BULK_TRANSFER
    /*****************************************************/
    /* EP4 ==> Bulk IN endpoint, address 3 */
    USBD_CONFIG_EP(EP6, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM);
    /* Buffer range for EP4 */
    USBD_SET_EP_BUF_ADDR(EP6, EP6_BUF_BASE);

    /* EP5 ==> Bulk Out endpoint, address 4 */
    USBD_CONFIG_EP(EP7, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM);
    /* Buffer range for EP5 */
    USBD_SET_EP_BUF_ADDR(EP7, EP7_BUF_BASE);
    /* trigger to receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP7, EP7_MAX_PKT_SIZE);
#endif

    /*
     * Generate Mass-Storage Device serial number To compliant USB-IF MSC test, we must enable serial string descriptor.
     * However, window may fail to recognize the devices if PID/VID and serial number are all the same when plug them to Windows at the sample time.
     * Therefore, we must generate different serial number for each device to avoid conflict when plug more then 2 MassStorage devices to Windows at the same time.
     *
     * NOTE: We use compiler predefine macro "__TIME__" to generate different number for serial at each build but each device here for a demo.
     * User must change it to make sure all serial number is different between each device.
     */
    pu8 = (uint8_t *)gsInfo.gu8StringDesc[3];

    for (i = 0; i < 8; i++)
    {
        pu8[pu8[0] - 16 + i*2] = pSerial[i];
    }
}

void HID_MSC_ClassRequest(void)
{
    uint8_t buf[8];

    USBD_GetSetupPacket(buf);

    /* request data transfer direction */
    if (buf[0] & EP_INPUT)
    {
        // Device to host
        switch (buf[1])
        {
            case GET_MAX_LUN:
                {
                    /* Check interface number with cfg descriptor and check wValue = 0, wLength = 1 */
                    if ((buf[4] == gsInfo.gu8ConfigDesc[LEN_CONFIG + 2]) && (buf[2] + buf[3] + buf[6] + buf[7] == 1))
                    {
                        M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = 0;
                        /* Data stage */
                        USBD_SET_DATA1(EP0);
                        USBD_SET_PAYLOAD_LEN(EP0, 1);
                        /* Status stage */
                        USBD_PrepareCtrlOut(0, 0);
                    }
                    else
                    {
                        USBD_SET_EP_STALL(EP1); // Stall when wrong parameter
                    }
                }
                break;

            default:
                {
                    /* Setup error, stall the device */
                    USBD_SetStall(0);
                }
                break;
        }
    }
    else
    {
        // Host to device
        switch (buf[1])
        {
            case SET_REPORT:
                {
                    if (buf[3] == 3)
                    {
                        /* Request Type = Feature */
                        USBD_SET_DATA1(EP1);
                        USBD_SET_PAYLOAD_LEN(EP1, 0);
                    }
                }
                break;

            case SET_IDLE:
                {
                    /* Status stage */
                    USBD_SET_DATA1(EP0);
                    USBD_SET_PAYLOAD_LEN(EP0, 0);
                }
                break;

            case BULK_ONLY_MASS_STORAGE_RESET:
                {
                    /* Check interface number with cfg descriptor and check wValue = 0, wLength = 0 */
                    if ((buf[4] == gsInfo.gu8ConfigDesc[LEN_CONFIG + 2]) && (buf[2] + buf[3] + buf[6] + buf[7] == 0))
                    {
#ifndef NO_MASS_STORAGE
                        g_u32Length = 0; // Reset all read/write data transfer
                        USBD_LockEpStall(0);

                        /* Clear ready */
                        USBD->EP[EP4].CFGP |= USBD_CFGP_CLRRDY_Msk;
                        USBD->EP[EP5].CFGP |= USBD_CFGP_CLRRDY_Msk;

                        /* Prepare to receive the CBW */

                        g_u8EP5Ready = 0;
                        g_u8BulkState = BULK_CBW;

                        USBD_SET_DATA1(EP5);
                        USBD_SET_EP_BUF_ADDR(EP5, g_u32BulkBuf0);
                        USBD_SET_PAYLOAD_LEN(EP5, 31);
#endif
                    }
                    else
                    {
                        /* Stall when wrong parameter */
                        USBD_SET_EP_STALL(EP1);
                    }

                    /* Status stage */
                    USBD_SET_DATA1(EP0);
                    USBD_SET_PAYLOAD_LEN(EP0, 0);
                }
                break;

            default:
                {
                    // Stall
                    /* Setup error, stall the device */
                    USBD_SetStall(0);
                }
                break;
        }
    }
}

#ifndef NO_MASS_STORAGE
void MSC_RequestSense(void)
{
    uint8_t senseData[20] = {0,};

    if ( g_u8Prevent )
    {
        g_u8Prevent = 0;
        senseData[0] = 0x70;//(0x70&0x7F); /* Error Code (70h) */
    }
    else
    {
        senseData[0] = 0xf0;//(0xF0&0x80); /* Valid */
    }

    senseData[2]  = (g_au8SenseKey[0]&0x0F);
    // Information
    //senseData[3]  = Information
    //senseData[4]  = Information
    //senseData[5]  = Information
    //senseData[6]  = Information
    senseData[7]  = 0x0a; /* Additional Sense Length (10) */
    // Reserved
    //senseData[8]  = Reserved
    //senseData[9]  = Reserved
    //senseData[10] = Reserved
    //senseData[11] = Reserved
    senseData[12] = g_au8SenseKey[1]; /* Additional Sense Code (Mandatory) */
    senseData[13] = g_au8SenseKey[2]; /* Additional Sense Code Qualifier (Mandatory) */
    // Reserved
    //senseData[14] = Reserved
    //senseData[15] = Reserved
    //senseData[16] = Reserved
    //senseData[17] = Reserved
    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4)), senseData, 20);

    g_au8SenseKey[0] = 0;
    g_au8SenseKey[1] = 0;
    g_au8SenseKey[2] = 0;
}
#endif

#ifndef NO_MASS_STORAGE
void MSC_ReadFormatCapacity(void)
{
    uint8_t *pu8Desc;

    pu8Desc = (uint8_t *)MassCMD_BUF;
    memset(pu8Desc, 0, 36);

    /*---------- Capacity List Header ----------*/
    // Capacity List Length
    pu8Desc[3]  = 0x10;

    /*---------- Current/Maximum Capacity Descriptor ----------*/
    // Number of blocks (MSB first)
    pu8Desc[4]  = _GET_BYTE3(g_TotalSectors);
    pu8Desc[5]  = _GET_BYTE2(g_TotalSectors);
    pu8Desc[6]  = _GET_BYTE1(g_TotalSectors);
    pu8Desc[7]  = _GET_BYTE0(g_TotalSectors);

    // Descriptor Code:
    // 01b = Unformatted Media - Maximum formattable capacity for this cartridge
    // 10b = Formatted Media - Current media capacity
    // 11b = No Cartridge in Drive - Maximum formattable capacity for any cartridge
    pu8Desc[8]  = 0x02;


    // Block Length. Fixed to be 512 (MSB first)
    pu8Desc[9]  = _GET_BYTE2(512);
    pu8Desc[10] = _GET_BYTE1(512);
    pu8Desc[11] = _GET_BYTE0(512);

    /*---------- Formattable Capacity Descriptor ----------*/
    // Number of Blocks
    pu8Desc[12] = _GET_BYTE3(g_TotalSectors);
    pu8Desc[13] = _GET_BYTE2(g_TotalSectors);
    pu8Desc[14] = _GET_BYTE1(g_TotalSectors);
    pu8Desc[15] = _GET_BYTE0(g_TotalSectors);

    // Block Length. Fixed to be 512 (MSB first)
    pu8Desc[17] = _GET_BYTE2(512);
    pu8Desc[18] = _GET_BYTE1(512);
    pu8Desc[19] = _GET_BYTE0(512);
}
#endif

#ifndef NO_MASS_STORAGE
void MSC_Read(void)
{
    uint32_t u32Len = 0;

    if (USBD_GET_EP_BUF_ADDR(EP4) == g_u32BulkBuf1)
    {
        USBD_SET_EP_BUF_ADDR(EP4, g_u32BulkBuf0);
    }
    else
    {
        USBD_SET_EP_BUF_ADDR(EP4, g_u32BulkBuf1);
    }

    /* Trigger to send out the data packet */
    USBD_SET_PAYLOAD_LEN(EP4, g_u8Size);

    g_u32Length -= g_u8Size;
    g_u32BytesInStorageBuf -= g_u8Size;

    if (g_u32Length)
    {
        if (g_u32BytesInStorageBuf)
        {
            /* Prepare next data packet */
            g_u8Size = EP4_MAX_PKT_SIZE;

            if (g_u8Size > g_u32Length)
            {
                g_u8Size = g_u32Length;
            }

            if (USBD_GET_EP_BUF_ADDR(EP4) == g_u32BulkBuf1)
            {
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0), (uint8_t *)g_u32Address, g_u8Size);
            }
            else
            {
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);
            }

            g_u32Address += g_u8Size;
        }
        else
        {
            u32Len = g_u32Length;

            if (u32Len > STORAGE_BUFFER_SIZE)
            {
                u32Len = STORAGE_BUFFER_SIZE;
            }

            MSC_ReadMedia(g_u32LbaAddress, u32Len, STORAGE_DATA_BUF);
            g_u32BytesInStorageBuf = u32Len;
            g_u32LbaAddress += u32Len;
            g_u32Address = STORAGE_DATA_BUF;

            /* Prepare next data packet */
            g_u8Size = EP4_MAX_PKT_SIZE;

            if (g_u8Size > g_u32Length)
                g_u8Size = g_u32Length;

            if (USBD_GET_EP_BUF_ADDR(EP4) == g_u32BulkBuf1)
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0), (uint8_t *)g_u32Address, g_u8Size);
            else
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);

            g_u32Address += g_u8Size;
        }
    }
}
#endif

#ifndef NO_MASS_STORAGE
void MSC_ReadTrig(void)
{
    uint32_t u32Len;

    if (g_u32Length)
    {
        if (g_u32BytesInStorageBuf)
        {
            /* Prepare next data packet */
            g_u8Size = EP4_MAX_PKT_SIZE;

            if (g_u8Size > g_u32Length)
            {
                g_u8Size = g_u32Length;
            }

            if (USBD_GET_EP_BUF_ADDR(EP4) == g_u32BulkBuf1)
            {
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0), (uint8_t *)g_u32Address, g_u8Size);
            }
            else
            {
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);
            }

            g_u32Address += g_u8Size;
        }
        else
        {
            u32Len = g_u32Length;

            if (u32Len > STORAGE_BUFFER_SIZE)
            {
                u32Len = STORAGE_BUFFER_SIZE;
            }

            MSC_ReadMedia(g_u32LbaAddress, u32Len, STORAGE_DATA_BUF);
            g_u32BytesInStorageBuf = u32Len;
            g_u32LbaAddress += u32Len;
            g_u32Address = STORAGE_DATA_BUF;

            /* Prepare next data packet */
            g_u8Size = EP4_MAX_PKT_SIZE;

            if (g_u8Size > g_u32Length)
            {
                g_u8Size = g_u32Length;
            }

            if (USBD_GET_EP_BUF_ADDR(EP4) == g_u32BulkBuf1)
            {
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0), (uint8_t *)g_u32Address, g_u8Size);
            }
            else
            {
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);
            }

            g_u32Address += g_u8Size;
        }

        /* DATA0/DATA1 Toggle */
        if (USBD_GET_EP_BUF_ADDR(EP4) == g_u32BulkBuf1)
        {
            USBD_SET_EP_BUF_ADDR(EP4, g_u32BulkBuf0);
        }
        else
        {
            USBD_SET_EP_BUF_ADDR(EP4, g_u32BulkBuf1);
        }

        /* Trigger to send out the data packet */
        USBD_SET_PAYLOAD_LEN(EP4, g_u8Size);

        g_u32Length -= g_u8Size;
        g_u32BytesInStorageBuf -= g_u8Size;

    }
    else
    {
        USBD_SET_PAYLOAD_LEN(EP4, 0);
    }
}
#endif

#ifndef NO_MASS_STORAGE
void MSC_ReadCapacity(void)
{
    uint32_t sectorAddr;

    memset((uint8_t *)MassCMD_BUF, 0, 36);

    sectorAddr = g_TotalSectors - 1;
    *((uint8_t *)(MassCMD_BUF+0)) = *((uint8_t *)&sectorAddr+3);
    *((uint8_t *)(MassCMD_BUF+1)) = *((uint8_t *)&sectorAddr+2);
    *((uint8_t *)(MassCMD_BUF+2)) = *((uint8_t *)&sectorAddr+1);
    *((uint8_t *)(MassCMD_BUF+3)) = *((uint8_t *)&sectorAddr+0);
    *((uint8_t *)(MassCMD_BUF+6)) = 0x02;
}
#endif

#ifndef NO_MASS_STORAGE
void MSC_ReadCapacity16(void)
{
    uint32_t sectorAddr;

    memset((uint8_t *)MassCMD_BUF, 0, 36);

    sectorAddr = g_TotalSectors - 1;
    *((uint8_t *)(MassCMD_BUF+0))  = 0;
    *((uint8_t *)(MassCMD_BUF+1))  = 0;
    *((uint8_t *)(MassCMD_BUF+2))  = 0;
    *((uint8_t *)(MassCMD_BUF+3))  = 0;
    *((uint8_t *)(MassCMD_BUF+4))  = *((uint8_t *)&sectorAddr+3);
    *((uint8_t *)(MassCMD_BUF+5))  = *((uint8_t *)&sectorAddr+2);
    *((uint8_t *)(MassCMD_BUF+6))  = *((uint8_t *)&sectorAddr+1);
    *((uint8_t *)(MassCMD_BUF+7))  = *((uint8_t *)&sectorAddr+0);
    *((uint8_t *)(MassCMD_BUF+10)) = 0x02;
}
#endif

#ifndef NO_MASS_STORAGE
void MSC_ModeSense10(void)
{
    uint8_t i, j;
    uint8_t NumHead, NumSector;
    uint16_t NumCyl = 0;

    /* Clear the command buffer */
    *((uint32_t *)MassCMD_BUF) = 0;
    *((uint32_t *)MassCMD_BUF + 1) = 0;

    switch (g_sCBW.au8Data[0])
    {
        case 0x01:
            {
                *((uint8_t *)MassCMD_BUF) = 19;
                i = 8;
                for (j = 0; j < 12; j++, i++)
                {
                    *((uint8_t *)(MassCMD_BUF+i)) = g_au8ModePage_01[j];
                }
            }
            break;

        case 0x05:
            {
                *((uint8_t *)MassCMD_BUF) = 39;
                i = 8;
                for (j = 0; j < 32; j++, i++)
                {
                    *((uint8_t *)(MassCMD_BUF+i)) = g_au8ModePage_05[j];
                }

                NumHead = 2;
                NumSector = 64;
                NumCyl = g_TotalSectors / 128;

                *((uint8_t *)(MassCMD_BUF+12)) = NumHead;
                *((uint8_t *)(MassCMD_BUF+13)) = NumSector;
                *((uint8_t *)(MassCMD_BUF+16)) = (uint8_t)(NumCyl >> 8);
                *((uint8_t *)(MassCMD_BUF+17)) = (uint8_t)(NumCyl & 0x00ff);
            }
            break;

        case 0x1B:
            {
                *((uint8_t *)MassCMD_BUF) = 19;
                i = 8;
                for (j = 0; j < 12; j++, i++)
                {
                    *((uint8_t *)(MassCMD_BUF+i)) = g_au8ModePage_1B[j];
                }
            }
            break;

        case 0x1C:
            {
                *((uint8_t *)MassCMD_BUF) = 15;
                i = 8;
                for (j = 0; j < 8; j++, i++)
                {
                    *((uint8_t *)(MassCMD_BUF+i)) = g_au8ModePage_1C[j];
                }
            }
            break;

        case 0x3F:
            {
                *((uint8_t *)MassCMD_BUF) = 0x47;
                i = 8;
                for (j = 0; j < 12; j++, i++)
                {
                    *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_01[j];
                }
                for (j = 0; j < 32; j++, i++)
                {
                    *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_05[j];
                }
                for (j = 0; j < 12; j++, i++)
                {
                    *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_1B[j];
                }
                for (j = 0; j < 8; j++, i++)
                {
                    *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_1C[j];
                }

                NumHead = 2;
                NumSector = 64;
                NumCyl = g_TotalSectors / 128;

                *((uint8_t *)(MassCMD_BUF+24)) = NumHead;
                *((uint8_t *)(MassCMD_BUF+25)) = NumSector;
                *((uint8_t *)(MassCMD_BUF+28)) = (uint8_t)(NumCyl >> 8);
                *((uint8_t *)(MassCMD_BUF+29)) = (uint8_t)(NumCyl & 0x00ff);
            }
            break;

        default:
            {
                g_au8SenseKey[0] = 0x05;
                g_au8SenseKey[1] = 0x24;
                g_au8SenseKey[2] = 0x00;
            }
    }
}
#endif

#ifndef NO_MASS_STORAGE
void MSC_Write(void)
{
    uint32_t lba = 0;
    uint32_t len = 0;

    if (g_u32Length > EP5_MAX_PKT_SIZE)
    {
        if (USBD_GET_EP_BUF_ADDR(EP5) == g_u32BulkBuf0)
        {
            USBD_SET_EP_BUF_ADDR(EP5, g_u32BulkBuf1);
            USBD_SET_PAYLOAD_LEN(EP5, EP5_MAX_PKT_SIZE);
            USBD_MemCopy((uint8_t *)g_u32Address, (uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0), EP5_MAX_PKT_SIZE);
        }
        else
        {
            USBD_SET_EP_BUF_ADDR(EP5, g_u32BulkBuf0);
            USBD_SET_PAYLOAD_LEN(EP5, EP5_MAX_PKT_SIZE);
            USBD_MemCopy((uint8_t *)g_u32Address, (uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), EP5_MAX_PKT_SIZE);
        }

        g_u32Address += EP5_MAX_PKT_SIZE;
        g_u32Length  -= EP5_MAX_PKT_SIZE;

        /* Buffer full. Writer it to storage first. */
        if (g_u32Address >= (STORAGE_DATA_BUF + STORAGE_BUFFER_SIZE))
        {
            MSC_WriteMedia(g_u32SPIFlashStartAddr, STORAGE_BUFFER_SIZE, STORAGE_DATA_BUF);

            g_u32Address = STORAGE_DATA_BUF;
            g_u32SPIFlashStartAddr += STORAGE_BUFFER_SIZE;
        }
    }
    else
    {
        if (USBD_GET_EP_BUF_ADDR(EP5) == g_u32BulkBuf0)
        {
            USBD_MemCopy((uint8_t *)g_u32Address, (uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0), g_u32Length);
        }
        else
        {
            USBD_MemCopy((uint8_t *)g_u32Address, (uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), g_u32Length);
        }

        g_u32Address += g_u32Length;
        g_u32Length  = 0;


        if ((g_sCBW.u8OPCode == UFI_WRITE_10) || (g_sCBW.u8OPCode == UFI_WRITE_12))
        {
            lba = get_be32(&g_sCBW.au8Data[0]);
            len = g_sCBW.dCBWDataTransferLength;

            len = lba * UDC_SECTOR_SIZE + g_sCBW.dCBWDataTransferLength - g_u32SPIFlashStartAddr;

            if (len)
            {
                MSC_WriteMedia(g_u32SPIFlashStartAddr, len, STORAGE_DATA_BUF);
            }
        }

        g_u8BulkState = BULK_IN;
        MSC_AckCmd();
    }
}
#endif

void HID_MSC_ProcessCmd(void)
{
    if ( g_u8BP14Ready )
    {
#ifndef NO_BULK_TRANSFER
        if (( g_u8EP2Ready ) || ( g_u8EP6Ready ))
        {
            g_u8BP14Ready = 0;
            USPICommRead(commBuf, 64);

            if ( commBuf[4] == HID_USER_BULK )
            {
                int dataSize = (commBuf[5]<<8) | commBuf[6];
                int blockCnt = dataSize / 64;
                if (( dataSize % 64) > 0 )
                    blockCnt += 1;

                while ( blockCnt-- )
                {
                    USPICommRead(commBuf, 64);
                    BULK_AckCmd();
                }
            }
            else
            {
                HID_AckCmd();
            }
        }
#else
        if ( g_u8EP2Ready )
        {
            g_u8BP14Ready = 0;
            USPICommRead(commBuf, 64);
            HID_AckCmd();
        }
#endif
    }

#ifndef NO_MASS_STORAGE
    if ( g_u8EP5Ready )
    {
        int32_t  i = 0;
        g_u8EP5Ready = 0;

        if (g_u8BulkState == BULK_CBW)
        {
            uint8_t u8Len = USBD_GET_PAYLOAD_LEN(EP5);

            if (u8Len > 31)
            {
                u8Len = 31;
            }

            /* Check Signature & length of CBW */
            /* Bulk Out buffer */
            if ((*(uint32_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0) != CBW_SIGNATURE)/* || (u8Len != 31)*/)
            {

                USBD_SET_EP_STALL(EP4);
                USBD_SET_EP_STALL(EP5);
                USBD_LockEpStall(1<<EP5);

                g_u8BulkState = BULK_CBW;
                DBG_PRINTF("CBW signature fail. stall bulk out pipe\n");
                return;

            }

            /* Get the CBW */
            for (i = 0; i < u8Len; i++)
            {
                *((uint8_t *)(&g_sCBW.dCBWSignature) + i) = *(uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0 + i);
            }


            /* Prepare to echo the tag from CBW to CSW */
            g_sCSW.dCSWTag = g_sCBW.dCBWTag;

            /* Parse Op-Code of CBW */
            switch (g_sCBW.u8OPCode)
            {
                case UFI_PREVENT_ALLOW_MEDIUM_REMOVAL:
                    {
                        if (g_sCBW.au8Data[2] & 0x01)
                        {
                            g_au8SenseKey[0] = 0x05;  //INVALID COMMAND
                            g_au8SenseKey[1] = 0x24;
                            g_au8SenseKey[2] = 0;
                            g_u8Prevent = 1;
                        }
                        else
                        {
                            g_u8Prevent = 0;
                        }

                        g_u8BulkState = BULK_IN;
                        MSC_AckCmd();
                    }
                    return;

                case UFI_VERIFY_10:
                case UFI_START_STOP:
                case UFI_TEST_UNIT_READY:
                    {
                        DBG_PRINTF("Test Unit\n");
                        g_u8BulkState = BULK_IN;
                        MSC_AckCmd();
                    }
                    return;

                case UFI_REQUEST_SENSE:
                    {
                        uint32_t u32Len;

                        u32Len = g_sCBW.dCBWDataTransferLength;

                        if (u32Len > 18)
                        {
                            u32Len = 18;
                        }

                        if (u32Len)
                        {
                            if (g_sCBW.dCBWDataTransferLength > u32Len)
                            {
                                /* Expecting a STALL after data phase completes with a zero-length or short packet */
                                USBD_SET_EP_STALL(EP4);
                                USBD_SET_EP_STALL(EP5);
                                USBD_LockEpStall((1<<EP4) | (1<<EP5));
                                return;
                            }

                            MSC_RequestSense();
                            g_u8BulkState = BULK_IN;
                            USBD_SET_PAYLOAD_LEN(EP4, u32Len);
                        }
                        else
                        {
                            /* Just skip data phase if zero data transfer length */
                            g_u8BulkState = BULK_IN;
                            MSC_AckCmd();
                        }
                    }
                    return;

                case UFI_READ_FORMAT_CAPACITY:
                    {
                        g_u32Length = g_sCBW.dCBWDataTransferLength;

                        /* format capacity descriptor length is fixed to be 12 bytes */
                        if (g_u32Length > 20)
                        {
                            g_u32Length = 20;
                        }

                        g_u32Address = MassCMD_BUF;
                        MSC_ReadFormatCapacity();
                        g_u8BulkState = BULK_IN;

                        if (g_u32Length > 0)
                        {
                            if (g_u32Length > EP4_MAX_PKT_SIZE)
                            {
                                g_u8Size = EP4_MAX_PKT_SIZE;
                            }
                            else
                            {
                                g_u8Size = g_u32Length;
                            }

                            /* Bulk IN buffer */
                            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);
                            g_u32BytesInStorageBuf = g_u8Size;

                            g_u32Address += g_u8Size;
                            USBD_SET_EP_BUF_ADDR(EP4, g_u32BulkBuf0);
                            MSC_Read();
                        }
                    }
                    return;

                case UFI_READ_CAPACITY:
                case UFI_READ_CAPACITY_16:
                    {
                        g_u32Length = g_sCBW.dCBWDataTransferLength;

                        if (g_u32Length > 36)
                        {
                            g_u32Length = 36;
                        }

                        g_u32Address = MassCMD_BUF;

                        if (g_sCBW.u8OPCode == UFI_READ_CAPACITY)
                        {
                            MSC_ReadCapacity();
                        }
                        else
                        {
                            MSC_ReadCapacity16();
                        }

                        g_u8BulkState = BULK_IN;

                        if (g_u32Length > 0)
                        {
                            if (g_u32Length > EP4_MAX_PKT_SIZE)
                            {
                                g_u8Size = EP4_MAX_PKT_SIZE;
                            }
                            else
                            {
                                g_u8Size = g_u32Length;
                            }

                            /* Bulk IN buffer */
                            USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);
                            g_u32BytesInStorageBuf = g_u8Size;

                            g_u32Address += g_u8Size;
                            USBD_SET_EP_BUF_ADDR(EP4, g_u32BulkBuf0);
                            MSC_Read();
                        }
                    }
                    return;

                case UFI_MODE_SELECT_10:
                    {
                        g_u32Length = g_sCBW.dCBWDataTransferLength;
                        g_u32Address = MassCMD_BUF;

                        if (g_u32Length > 0)
                        {
                            USBD_SET_PAYLOAD_LEN(EP5, EP5_MAX_PKT_SIZE);
                            g_u8BulkState = BULK_OUT;
                        }
                    }
                    return;

                case UFI_MODE_SENSE_10:
                    {
                        if (g_u32Length == 0)
                        {
                            g_u32Length = g_sCBW.dCBWDataTransferLength;
                            g_u32Address = MassCMD_BUF;
                        }

                        MSC_ModeSense10();
                        g_u8BulkState = BULK_IN;

                        if (g_u32Length > 0)
                        {
                            if (g_u32Length > EP4_MAX_PKT_SIZE)
                            {
                                g_u8Size = EP4_MAX_PKT_SIZE;
                            }
                            else
                            {
                                g_u8Size = g_u32Length;
                            }

                            /* Bulk IN buffer */
                            USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);

                            g_u32Address += g_u8Size;

                            USBD_SET_EP_BUF_ADDR(EP4, g_u32BulkBuf0);
                            MSC_Read();
                        }
                    }
                    return;

                case UFI_INQUIRY:
                    {
                        uint32_t u32Len;
                        uint8_t u8PageCode;


                        u32Len = g_sCBW.dCBWDataTransferLength;

                        /* Limit length */
                        if (u32Len > 36)
                        {
                            u32Len = 36;
                        }

                        u8PageCode = g_sCBW.au8Data[0];


                        g_u8BulkState = BULK_IN;

                        if (u32Len)
                        {
                            /* u8PageCode should be zero */
                            if (u8PageCode)
                            {
                                /* Expecting a STALL after data phase completes with a zero-length or short packet */
                                //USBD_SET_EP_STALL(EP0);
                                USBD_SET_EP_STALL(EP5);
                                USBD_LockEpStall(1<<EP5);

                                DBG_PRINTF("INQUIRY page code = %d", u8PageCode);
                            }

                            /* Bulk IN buffer */
                            USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_au8InquiryID, u32Len);
                            USBD_SET_PAYLOAD_LEN(EP4, u32Len);

                            DBG_PRINTF("Inquiry, len %d\n", u32Len);
                        }
                        else
                        {
                            /* Next is status phase if zero data length in data phase */
                            MSC_AckCmd();
                        }
                    }
                    return;

                case UFI_READ_10:
                case UFI_READ_12:
                    {
                        /* Check if it is a new transfer */
                        if (g_u32Length == 0)
                        {
                            /* Prepare the data for Bulk IN transfer */

                            /* Get LBA address */
                            g_u32Address = get_be32(&g_sCBW.au8Data[0]);
                            g_u32LbaAddress = g_u32Address * UDC_SECTOR_SIZE;
                            g_u32Length = g_sCBW.dCBWDataTransferLength;
                            g_u32BytesInStorageBuf = g_u32Length;

                            DBG_PRINTF("Read addr=0x%x, len=0x%x\n", g_u32LbaAddress, g_u32Length);

                            /* Error check  */
                            if ((g_u32LbaAddress > SPI_FLASH_STORAGE_SIZE) || (g_u32LbaAddress + g_u32Length > SPI_FLASH_STORAGE_SIZE))
                            {

                                USBD_SET_EP_STALL(EP4);
                                USBD_SET_EP_STALL(EP5);
                                USBD_LockEpStall((1<<EP4) | (1<<EP5));

                                DBG_PRINTF("Stall ep2, ep3. addr=0x%x, len=0x%x\n", g_u32LbaAddress, g_u32Length);

                                return;
                            }


                            i = g_u32Length;

                            if (i > STORAGE_BUFFER_SIZE)
                                i = STORAGE_BUFFER_SIZE;

                            MSC_ReadMedia(g_u32Address * UDC_SECTOR_SIZE, i, STORAGE_DATA_BUF);
                            g_u32BytesInStorageBuf = i;
                            g_u32LbaAddress += i;
                        }

                        g_u32Address = STORAGE_DATA_BUF;

                        /* Indicate the next packet should be Bulk IN Data packet */
                        g_u8BulkState = BULK_IN;

                        if (g_u32BytesInStorageBuf > 0) {
                            /* Set the packet size */
                            if (g_u32BytesInStorageBuf > EP4_MAX_PKT_SIZE)
                                g_u8Size = EP4_MAX_PKT_SIZE;
                            else
                                g_u8Size = g_u32BytesInStorageBuf;

                            /* Prepare the first data packet (DATA1) */
                            /* Bulk IN buffer */
                            USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);
                            g_u32Address += g_u8Size;

                            /* kick - start */
                            USBD_SET_EP_BUF_ADDR(EP4, g_u32BulkBuf1);
                            /* Trigger to send out the data packet */
                            USBD_SET_PAYLOAD_LEN(EP4, g_u8Size);
                            g_u32Length -= g_u8Size;
                            g_u32BytesInStorageBuf -= g_u8Size;
                        }
                    }
                    return;

                case UFI_WRITE_10:
                case UFI_WRITE_12:
                    {
                        if (g_u32Length == 0)
                        {
                            g_u32Length = g_sCBW.dCBWDataTransferLength;
                            g_u32Address = STORAGE_DATA_BUF;
                            g_u32SPIFlashStartAddr = get_be32(&g_sCBW.au8Data[0]) * UDC_SECTOR_SIZE;
                        }

                        DBG_PRINTF("Write 0x%x  0x%x\n", g_u32Address, g_u32Length);

                        if ((g_u32Length > 0))
                        {
                            USBD_SET_PAYLOAD_LEN(EP5, EP5_MAX_PKT_SIZE);
                            g_u8BulkState = BULK_OUT;
                        }
                    }
                    return;

                case UFI_MODE_SENSE_6:
                    {
                        uint32_t u32Data = 0x3;
                        g_u8BulkState = BULK_IN;
                        USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)u32Data, 4);
                        USBD_SET_PAYLOAD_LEN(EP4, 4);
                    }
                    return;

                case UFI_MODE_SELECT_6:
                    {
                        g_u32Length = g_sCBW.dCBWDataTransferLength;
                        g_u32Address = MassCMD_BUF;

                        if (g_u32Length > 0)
                        {
                            USBD_SET_PAYLOAD_LEN(EP5, EP5_MAX_PKT_SIZE);
                            g_u8BulkState = BULK_OUT;
                        }
                    }
                    return;

                default:
                    {
                        /* Just stall for unknown command */
                        //USBD_SET_EP_STALL(EP4);
                        //USBD_SET_EP_STALL(EP5);
                        //USBD_LockEpStall((1<<EP4) | (1<<EP5));
                        /* Unknow command */
                        DBG_PRINTF("Unknow cmd 0x%x\n", g_sCBW.u8OPCode);

                        if (g_sCBW.bmCBWFlags & 0x80)
                        {
                            USBD_SET_PAYLOAD_LEN(EP4, 4);
                        }

                        g_u8BulkState = BULK_IN;
                        MSC_AckCmd();
                    }
                    return;
            }
        }
        else if (g_u8BulkState == BULK_OUT)
        {
            switch (g_sCBW.u8OPCode)
            {
                case UFI_WRITE_10:
                case UFI_WRITE_12:
                case UFI_MODE_SELECT_10:
                    {
                        MSC_Write();
                    }
                    return;

                default:
                    {
                        /* Bulk-out of unkonwn command. Just dorp them. */
                        if (g_u32Length > EP5_MAX_PKT_SIZE)
                        {
                            USBD_SET_EP_BUF_ADDR(EP5, g_u32BulkBuf0);
                            USBD_SET_PAYLOAD_LEN(EP5, EP5_MAX_PKT_SIZE);
                            g_u32Length -= EP5_MAX_PKT_SIZE;
                        }
                        else
                        {
                            g_u32Length = 0;
                            g_u8BulkState = BULK_IN;
                            MSC_AckCmd();
                        }
                    }
                    break;
            }
        }
    }
#endif
}

void HID_AckCmd(void)
{
    uint8_t *ptr = NULL;
    ptr = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2));
    USBD_MemCopy(ptr, commBuf, EP2_MAX_PKT_SIZE);
    USBD_SET_PAYLOAD_LEN(EP2, EP2_MAX_PKT_SIZE);
    g_u8EP2Ready = 0;
}

#ifndef NO_BULK_TRANSFER
void BULK_AckCmd(void)
{
    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP6)), commBuf, EP6_MAX_PKT_SIZE);
    USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
    g_u8EP6Ready = 0;
}
#endif

#ifndef NO_MASS_STORAGE
void MSC_AckCmd(void)
{
    /* Bulk IN */
    int32_t volatile idx;

    if (g_u8BulkState == BULK_CSW)
    {
        /* Prepare to receive the CBW */
        g_u8BulkState = BULK_CBW;

        USBD_SET_EP_BUF_ADDR(EP5, g_u32BulkBuf0);
        USBD_SET_PAYLOAD_LEN(EP5, 31);

        DBG_PRINTF("CSW ack\n");
    }
    else if (g_u8BulkState == BULK_IN)
    {
        switch (g_sCBW.u8OPCode)
        {
            case UFI_READ_FORMAT_CAPACITY:
            case UFI_READ_CAPACITY:
            case UFI_READ_CAPACITY_16:
            case UFI_MODE_SENSE_10:
                {
                    if (g_u32Length > 0)
                    {
                        MSC_Read();
                        return;
                    }

                    if (g_sCBW.dCBWDataTransferLength > 36)
                    {
                        g_sCSW.dCSWDataResidue = g_sCBW.dCBWDataTransferLength - 36;
                    }
                    else
                    {
                        g_sCSW.dCSWDataResidue = 0;
                    }

                    g_sCSW.bCSWStatus = 0;
                }
                break;

            case UFI_READ_10:
            case UFI_READ_12:
                {
                    if (g_u32Length > 0)
                    {
                        MSC_ReadTrig();
                        return;
                    }
                }
                // <- CHOI WANTAEK: why no break???????

            case UFI_REQUEST_SENSE:
            case UFI_INQUIRY:
                {
                    if (g_sCBW.dCBWDataTransferLength > 36)
                    {
                        // Stall EP2 after short packet
                        //USBD_SET_EP_STALL(EP2);

                        g_sCSW.dCSWDataResidue = g_sCBW.dCBWDataTransferLength - 36;
                        g_sCSW.bCSWStatus = 0;
                        DBG_PRINTF("Inquiry size > 36\n");
                    }
                    else
                    {
                        g_sCSW.dCSWDataResidue = 0;
                        g_sCSW.bCSWStatus = 0;
                        DBG_PRINTF("Inquiry ack, %x\n", USBD->EP[4].CFGP);
                    }
                }
                break;

        case UFI_PREVENT_ALLOW_MEDIUM_REMOVAL:
        case UFI_VERIFY_10:
        case UFI_START_STOP:
        case UFI_WRITE_10:
        case UFI_WRITE_12:
            {
                int32_t tmp;

                tmp = g_sCBW.dCBWDataTransferLength - STORAGE_BUFFER_SIZE;

                if (tmp < 0)
                {
                    tmp = 0;
                }

                g_sCSW.dCSWDataResidue = tmp;
                g_sCSW.bCSWStatus = 0;
            }
            break;

        case UFI_TEST_UNIT_READY:
            {
                g_sCSW.dCSWDataResidue = 0;
                g_sCSW.bCSWStatus = 0;

            }
            break;

        case UFI_MODE_SENSE_6:
            {
                g_sCSW.dCSWDataResidue = g_sCBW.dCBWDataTransferLength - 4;
                g_sCSW.bCSWStatus = 0;
            }
            break;

        default:
            {
                // Unknown command
                //USBD_SET_EP_STALL(EP4);
                //USBD_SET_EP_STALL(EP5);
                //USBD_LockEpStall(1<<EP5);
                g_sCSW.dCSWDataResidue = g_sCBW.dCBWDataTransferLength;
                g_sCSW.bCSWStatus = 1; // return command failed
            }
            break;
        }

        /* Return the CSW */
        USBD_SET_EP_BUF_ADDR(EP4, g_u32BulkBuf1);

        /* Bulk IN buffer */
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)&g_sCSW.dCSWSignature, 16);

        g_u8BulkState = BULK_CSW;
        USBD_SET_PAYLOAD_LEN(EP4, 13);

        DBG_PRINTF("Prepare CSW\n");
    }
    else
    {
        // This should be a DATA phase error.
        USBD_SET_EP_STALL(EP4);
        USBD_SET_EP_STALL(EP5);
        USBD_LockEpStall((1<<EP4) | (1<<EP5));

        DBG_PRINTF("Unexpected IN ack\n");
    }
}

void MSC_ReadMedia(uint32_t addr, uint32_t size, uint32_t buffer)
{
    SPIFlashRead(addr, size, buffer);
}

void MSC_WriteMedia(uint32_t addr, uint32_t size, uint32_t buffer)
{
    SPIFlashWrite(addr, size, buffer);
}

void MSC_SetConfig(void)
{
    // Clear stall status and ready
    USBD->EP[4].CFGP = 1;
    USBD->EP[5].CFGP = 1;
    /*****************************************************/
    /* EP4 ==> Bulk IN endpoint, address 3 */
    USBD_CONFIG_EP(EP4, USBD_CFG_EPMODE_IN | MSC_IN_EP_NUM);
    /* Buffer range for EP4 */
    USBD_SET_EP_BUF_ADDR(EP4, EP4_BUF_BASE);

    /* EP5 ==> Bulk Out endpoint, address 4 */
    USBD_CONFIG_EP(EP5, USBD_CFG_EPMODE_OUT | MSC_OUT_EP_NUM);
    /* Buffer range for EP5 */
    USBD_SET_EP_BUF_ADDR(EP5, EP5_BUF_BASE);

    /* trigger to receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP5, EP5_MAX_PKT_SIZE);

    USBD_LockEpStall(0);
    g_u8BulkState = BULK_CBW;

    DBG_PRINTF("Set config\n");
}
#endif
