#ifndef __USPI_COMM_H__
#define __USPI_COMM_H__

#include <stdint.h>

void USPICommWrite(uint8_t* writeBuffer, uint32_t WriteSize);
void USPICommRead(uint8_t* readBuffer, uint32_t readSize);

//void SPIFlashTest(void);

#endif
