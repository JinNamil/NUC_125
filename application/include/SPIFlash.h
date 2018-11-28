#ifndef __SPI_FLASH_H__
#define __SPI_FLASH_H__

#define SPI_FLASH_STORAGE_SIZE   (0x01000000)  /* SPI FLASH storage size. 16 MB */ 

void SPIFlashRead(uint32_t addr, uint32_t size, uint32_t buffer);
void SPIFlashWrite(uint32_t addr, uint32_t size, uint32_t buffer);
void SPIFlashChipErase(void);
uint32_t SPIFlashReadMidDid(void);
uint32_t SPIFlashReadJedecID(void);

void SPIFlashTest(void);

#endif
