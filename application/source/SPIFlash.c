#include <stdio.h>
#include <stdint.h>
#include "NUC121.h"
#include "SPIFlash.h"

#if 0
#include <stdio.h>
    #define DBG_PRINTF      printf
#else
    #define DBG_PRINTF(...)
#endif

#define SPI_Flash                   (SPI0)

#define DRVSPIFLASH_PAGE_SIZE       (256)
#define DRVSPIFLASH_SECTOR_SIZE     (4096)

uint32_t g_sectorBuf[DRVSPIFLASH_SECTOR_SIZE/4];

///* SPI Flash Status */
#define DRVSPIFLASH_SPR             (0x80)    /* Status Register Protect      */
#define DRVSPIFLASH_R               (0x40)    /* Reserved Bit                 */
#define DRVSPIFLASH_TB              (0x20)    /* Top / Bottom Block Protect   */
#define DRVSPIFLASH_BP2             (0x10)    /* Block Protect Bit 2          */
#define DRVSPIFLASH_BP1             (0x08)    /* Block Protect Bit 1          */
#define DRVSPIFLASH_BP0             (0x04)    /* Block Protect Bit 0          */
#define DRVSPIFLASH_WEL             (0x02)    /* Write Enable Latch           */
#define DRVSPIFLASH_BUSY            (0x01)    /* BUSY                         */


///* SPI Flash Command */
#define DRVSPIFLASH_ZERO            (0x00)
#define DRVSPIFLASH_DUMMY           (0xFF)
#define DRVSPIFLASH_WRITE_ENABLE    (0x06)
#define DRVSPIFLASH_WRITE_DISABLE   (0x04)
#define DRVSPIFLASH_READ_STATUS     (0x05)
#define DRVSPIFLASH_WRITE_STATUS    (0x01)
#define DRVSPIFLASH_FAST_READ       (0x0B)
#define DRVSPIFLASH_FAST_RD_DUAL    (0x3B)
#define DRVSPIFLASH_PAGE_PROGRAM    (0x02)
#define DRVSPIFLASH_BLOCK_ERASE     (0xD8)
#define DRVSPIFLASH_SECTOR_ERASE    (0x20)
#define DRVSPIFLASH_CHIP_ERASE      (0xC7)
#define DRVSPIFLASH_POWER_DOWN      (0xB9)
#define DRVSPIFLASH_RELEASE_PD_ID   (0xAB)
#define DRVSPIFLASH_FAST_READ_PARA  (0x5B)
#define DRVSPIFLASH_PROGRAM_PARA    (0x52)
#define DRVSPIFLASH_ERASE_PARA      (0xD5)
#define DRVSPIFLASH_DEVICE_ID       (0x90)
#define DRVSPIFLASH_JEDEC_ID        (0x9F)

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define Check_SPI_BUSY              while(SPI_IS_BUSY(SPI_Flash))
#define SPI_SetBitLength(n)         SPI_SET_DATA_WIDTH(SPI_Flash,8*(n))
#define SPI_WRITE(n)                SPI_WRITE_TX(SPI_Flash, n)
#define SPI_READ                    SPI_READ_RX(SPI_Flash)
#define SS_HIGH                     SPI_SET_SS_HIGH(SPI_Flash)
#define SS_LOW                      SPI_SET_SS_LOW(SPI_Flash)
#define SPI_GO                      //SPI_TRIGGER(SPI_Flash)
///////////////////////////////////////////////////////////////////////////////////

static void WINBOND25X16A_ReadDummy(void)
{
    uint32_t u32Dummy = SPI_READ;
}

static void WINBOND25X16A_ReadDataByByte(uint8_t* pu8Data)
{
    uint32_t u32Data;
    SPI_SetBitLength(1);
    SPI_WRITE(0xff);
    SPI_GO;
    Check_SPI_BUSY;
    u32Data=SPI_READ;
    *pu8Data = u32Data & 0xFF;
}

static void WINBOND25X16A_ReadDataByWord(uint8_t * pu8Data)
{
    uint32_t u32Data;
    SPI_SetBitLength(4);
    SPI_WRITE(0xffffffff);
    SPI_GO;
    Check_SPI_BUSY;
    u32Data=SPI_READ;
    pu8Data[0] = (u32Data >> 24) & 0xFF;
    pu8Data[1] = (u32Data >> 16) & 0xFF;
    pu8Data[2] = (u32Data >>  8) & 0xFF;
    pu8Data[3] = u32Data & 0xFF;
}

static void WINBOND25X16A_ReadData(uint8_t* pu8Data, uint32_t u32CmdSize)
{
    uint32_t u32Data;
    SPI_SetBitLength(u32CmdSize);
    SPI_WRITE(DRVSPIFLASH_DUMMY);
    SPI_GO;
    Check_SPI_BUSY;
    u32Data=SPI_READ;
    while (u32CmdSize--)
    {
        pu8Data[u32CmdSize] = (u32Data>>(u32CmdSize*8)) & 0xFF;
    }
}

static void WINBOND25X16A_SendCommandData(uint32_t u32Cmd, uint32_t u32CmdSize)
{
    SPI_SetBitLength(u32CmdSize);
    SPI_WRITE(u32Cmd);
    SPI_GO;
    Check_SPI_BUSY;
    WINBOND25X16A_ReadDummy();
}

static void WINBOND25X16A_GetStatus(uint8_t * pu8Status)
{
    SS_LOW;
    WINBOND25X16A_SendCommandData(DRVSPIFLASH_READ_STATUS, 0x1);
    WINBOND25X16A_ReadDataByByte(pu8Status);
    SS_HIGH;
}


static uint8_t WINBOND25X16A_WaitIdle(void)
{
    uint8_t u8Status = 0;

    do
    {
        WINBOND25X16A_GetStatus(&u8Status);
    }
    while ((u8Status & DRVSPIFLASH_BUSY));

    return u8Status;
}


static void WINBOND25X16A_EnableWrite(uint8_t bEnable)
{
    SS_LOW;

    if (bEnable)
    {
        WINBOND25X16A_SendCommandData(DRVSPIFLASH_WRITE_ENABLE, 0x1);
    }
    else
    {
        WINBOND25X16A_SendCommandData(DRVSPIFLASH_WRITE_DISABLE, 0x1);
    }

    SS_HIGH;;
    WINBOND25X16A_WaitIdle();
}

static void WINBOND25X16A_EraseSector(uint32_t u32StartSector)
{
    uint32_t u32StartAddr;
    uint8_t  u8Status = 0;

    WINBOND25X16A_EnableWrite(TRUE);
    WINBOND25X16A_WaitIdle();
    SS_LOW;
    u32StartAddr = u32StartSector & 0x0FFFF000;
    WINBOND25X16A_SendCommandData(((DRVSPIFLASH_SECTOR_ERASE<<24) | u32StartAddr), 0x4);
    SS_HIGH;

    do
    {
        WINBOND25X16A_GetStatus(&u8Status);
    } while ((u8Status & DRVSPIFLASH_WEL));

}

static void WINBOND25X16A_ProgramPage(uint32_t u32StartPage, uint8_t * pu8Data)
{
    uint32_t u32StartAddr;
    uint32_t u32LoopCounter;
    uint32_t u32Data;
    uint8_t  u8Status;

    WINBOND25X16A_EnableWrite(TRUE);

    u8Status = WINBOND25X16A_WaitIdle();

    if ( u8Status & DRVSPIFLASH_WEL )
    {
        SS_LOW;
        u32StartAddr = u32StartPage & 0x0FFFFF00;
        WINBOND25X16A_SendCommandData(((DRVSPIFLASH_PAGE_PROGRAM<<24) | u32StartAddr), 0x4);

        for (u32LoopCounter = 0; u32LoopCounter < DRVSPIFLASH_PAGE_SIZE; u32LoopCounter += 4)
        {
            u32Data = ((((pu8Data[u32LoopCounter]<<24) | (pu8Data[u32LoopCounter + 1]<<16)) |
            (pu8Data[u32LoopCounter + 2]<<8)) + pu8Data[u32LoopCounter + 3]);

            WINBOND25X16A_SendCommandData(u32Data, 0x4);
        }

        SS_HIGH;
        WINBOND25X16A_WaitIdle();
        pu8Data += DRVSPIFLASH_PAGE_SIZE;
    }
}

static void WINBOND25X16A_ReadPage(uint8_t u8ReadMode, uint32_t u32StartPage, uint8_t* pu8Data)
{
    uint32_t u32StartAddr = u32StartPage&0x0FFFFF00;
    uint32_t u32DataSize  = DRVSPIFLASH_PAGE_SIZE;
    uint32_t u32LoopCounter;

    SS_LOW;
    if ( u8ReadMode == 0 )
    {
        WINBOND25X16A_SendCommandData(((DRVSPIFLASH_FAST_READ<<24)|u32StartAddr), 0x4);
    }
    else
    {
        WINBOND25X16A_SendCommandData(((DRVSPIFLASH_FAST_RD_DUAL<<24)|u32StartAddr), 0x4);
    }

    WINBOND25X16A_SendCommandData(DRVSPIFLASH_DUMMY, 0x1);

    for ( u32LoopCounter = 0; u32LoopCounter < u32DataSize; u32LoopCounter += 4 )
    {
        WINBOND25X16A_ReadDataByWord(&pu8Data[u32LoopCounter]);
    }
    SS_HIGH;
}

void SPIFlashRead(uint32_t addr, uint32_t size, uint32_t buffer)
{
    int32_t len = (int32_t)size;
    
    /* This is low level read function of USB Mass Storage */
    while ( len >= DRVSPIFLASH_PAGE_SIZE )
    {
        WINBOND25X16A_ReadPage(0, addr, (uint8_t *)buffer);
        addr   += DRVSPIFLASH_PAGE_SIZE;
        buffer += DRVSPIFLASH_PAGE_SIZE;
        len    -= DRVSPIFLASH_PAGE_SIZE;
    }
}

void SPIFlashWrite(uint32_t addr, uint32_t size, uint32_t buffer)
{
    /* This is low level write function of USB Mass Storage */
    int32_t len, i, offset;
    uint32_t *pu32;
    uint32_t alignAddr;

    len = (int32_t)size;

    if (len == DRVSPIFLASH_SECTOR_SIZE && ((addr & (DRVSPIFLASH_SECTOR_SIZE-1)) == 0))
    {        
        /* one-sector length & the start address is sector-alignment */
        WINBOND25X16A_EraseSector(addr);
        
        //WINBOND25X16A_EnableWrite(g_SpiPort, g_SlaveSel, TRUE);

        while ( len >= DRVSPIFLASH_PAGE_SIZE )
        {
            WINBOND25X16A_ProgramPage( addr, (uint8_t *)buffer);
            len    -= DRVSPIFLASH_PAGE_SIZE;
            buffer += DRVSPIFLASH_PAGE_SIZE;
            addr   += DRVSPIFLASH_PAGE_SIZE;
        }
    }
    else
    {        
        do
        {
            /* alignAddr: sector address */
            alignAddr = addr & 0x1FFFF000;

            /* Get the sector offset*/
            offset = (addr & (DRVSPIFLASH_SECTOR_SIZE-1));

            if (offset || (size < DRVSPIFLASH_SECTOR_SIZE))
            {
                /* if the start address is not sector-alignment or the size is less than one sector, */
                /* read back the data of the destination sector to g_sectorBuf[].                    */
                SPIFlashRead(alignAddr, DRVSPIFLASH_SECTOR_SIZE, (uint32_t)&g_sectorBuf[0]);
            }

            /* Update the data */
            pu32 = (uint32_t *)buffer;
            len = DRVSPIFLASH_SECTOR_SIZE - offset; /* len: the byte count between the start address and the end of a sector. */
            if (size < len) /* check if the range of data arrive at the end of a sector. */
                len = size; /* Not arrive at the end of a sector. "len" indicate the actual byte count of data. */

            for ( i = 0; i < len/4; i++ )
            {
                g_sectorBuf[offset/4 + i] = pu32[i];
            }
            
            WINBOND25X16A_EraseSector(alignAddr);

            //WINBOND25X16A_EnableWrite(g_SpiPort, g_SlaveSel, TRUE);

            for ( i = 0; i < 16; i++) /* one sector (16 pages) */
            {
                WINBOND25X16A_ProgramPage(alignAddr+(i<<8), (uint8_t *)g_sectorBuf+(i<<8));
            }

            size -= len;
            addr += len;
            buffer += len;

        } 
        while ( size > 0 );
    }
}

void SPIFlashChipErase(void)
{
    WINBOND25X16A_EnableWrite(TRUE);
    SS_LOW;
    WINBOND25X16A_SendCommandData(DRVSPIFLASH_CHIP_ERASE, 0x1);
    SS_HIGH;
    WINBOND25X16A_WaitIdle();
}

uint32_t SPIFlashReadMidDid(void)
{
    uint32_t au32DestinationData = 0;

    SS_LOW;
    WINBOND25X16A_SendCommandData(DRVSPIFLASH_DEVICE_ID, 0x1);
    WINBOND25X16A_ReadData((uint8_t*)&au32DestinationData, 0x3);
    au32DestinationData = 0;
    WINBOND25X16A_ReadData((uint8_t*)&au32DestinationData, 0x2);
    SS_HIGH;
    
    return au32DestinationData;
}

uint32_t SPIFlashReadJedecID(void)
{
    uint32_t au32DestinationData = 0;

    SS_LOW;
    WINBOND25X16A_SendCommandData(DRVSPIFLASH_JEDEC_ID, 0x1);
    WINBOND25X16A_ReadData((uint8_t*)&au32DestinationData, 0x3);
    SS_HIGH;
    
    return au32DestinationData;
}

void SPIFlashTest(void)
{
    uint32_t addr = 0;
    uint32_t data = 0;
    
    SPIFlashChipErase();
    
    while(addr < 0x800)
    {
        data = 0;
        SPIFlashRead(addr, 4, (uint32_t)&data);
        if (data != 0)
        {
            DBG_PRINTF("0x%08x(0x%08X)\n", addr, data);
        }
   
        data = addr&0xFF;
        SPIFlashWrite(addr, 4, data);
        
        SPIFlashRead(addr, 4, (uint32_t)&data);
        DBG_PRINTF("0x%08x(0x%08X)\n", addr, data);
        
        addr += 4;
    }
}
