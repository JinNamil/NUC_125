#include <stdio.h>
#include <stdint.h>
#include "NUC121.h"
#include "USPIComm.h"

#if 1
    #define DBG_PRINTF      printf
#else
    #define DBG_PRINTF(...)
#endif

#define SPI_Comm    (USPI0)

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define Check_SPI_BUSY              while(USPI_IS_BUSY(SPI_Comm))
#define SPI_WRITE(n)                USPI_WRITE_TX(SPI_Comm, n)
#define SPI_READ                    USPI_READ_RX(SPI_Comm)

/////////////////////////////////////////////////////////////////////////////////////
static void __io_delay(uint32_t delay)
{
    volatile uint32_t i;
    for ( i=0; i<=delay; i++ )
    {
        __NOP();
    }
}

void USPICommWrite(uint8_t* writeBuffer, uint32_t WriteSize)
{
    int i = 0;
    volatile uint32_t dummy = 0;
#ifdef NO_SPI_16_BIT
	  uint8_t* writeBufferP = (uint8_t*)writeBuffer;
	  int writeSizeP = WriteSize;
#else
    uint16_t* writeBufferP = (uint16_t*)writeBuffer;
	  int writeSizeP = WriteSize / 2;
#endif

    DBG_PRINTF("%s", __FUNCTION__);
    
    for (i = 0; i < writeSizeP; i++)
    {
        SPI_WRITE(writeBufferP[i]);
        Check_SPI_BUSY;
        dummy = SPI_READ;
        __io_delay(60);
    }
}

void USPICommRead(uint8_t* readBuffer, uint32_t readSize)
{
    uint32_t i = 0;
	  
#ifdef NO_SPI_16_BIT
	  uint8_t* readBufferP = (uint8_t*)readBuffer;
	  int readSizeP = readSize;
	  uint8_t dummy = 0xFF;
#else
    uint16_t* readBufferP = (uint16_t*)readBuffer;
	  int readSizeP = readSize / 2;
	  uint16_t dummy = 0xFFFF;
#endif
	      
    for (i = 0; i < readSizeP; i++)
    {
        SPI_WRITE(dummy);
        Check_SPI_BUSY;
        readBufferP[i] = (SPI_READ & dummy);
        __io_delay(60);
//		__io_delay(50000);
    }
	//DBG_PRINTF("\n");
    
//    DBG_PRINTF("%s", __FUNCTION__);
}
