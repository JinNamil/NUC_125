#include "NUC121.h"
#include "SPIFlash.h"
#include "USPIComm.h"
#include "HID_FIDO_and_MassStorage.h"

#if 1
#include <stdio.h>
//#include <stdint.h>
#include <string.h>
    #define DBG_PRINTF      printf
#else
    #define DBG_PRINTF(...)
#endif

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal HIRC 48 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(SPI0_MODULE);
    CLK_EnableModuleClock(PDMA_MODULE);
    CLK_EnableModuleClock(USCI0_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK0, MODULE_NoMsk);
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_HIRC, CLK_CLKDIV0_USB(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PAH multi-function pins for USCI_SPI0 SPI_MISO and SPI_MOSI */
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA10MFP_Msk | SYS_GPA_MFPH_PA11MFP_Msk);
    SYS->GPA_MFPH |= SYS_GPA_MFPH_PA10MFP_USCI0_DAT1 | SYS_GPA_MFPH_PA11MFP_USCI0_DAT0;

    /* Set PBL multi-function pins for USCI_SPI0 SPI_SS and SPI_CLK */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk);
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB4MFP_USCI0_CTL0 | SYS_GPB_MFPL_PB5MFP_USCI0_CLK;

    /* Set PCH pins for SPI0 (PC.8 is SPI_SS, PC.9 is SPI_CLK, PC.10 is SPI_MISO, PC.11 is SPI_MOSI) */
    SYS->GPC_MFPH &= ~(SYS_GPC_MFPH_PC8MFP_Msk | SYS_GPC_MFPH_PC9MFP_Msk | SYS_GPC_MFPH_PC10MFP_Msk | SYS_GPC_MFPH_PC11MFP_Msk);
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC8MFP_SPI0_SS | SYS_GPC_MFPH_PC9MFP_SPI0_CLK | SYS_GPC_MFPH_PC10MFP_SPI0_MISO | SYS_GPC_MFPH_PC11MFP_SPI0_MOSI;

    /* Set PDL multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD1MFP_Msk | SYS_GPD_MFPL_PD2MFP_Msk);
    SYS->GPD_MFPL |= SYS_GPD_MFPL_PD1MFP_UART0_RXD | SYS_GPD_MFPL_PD2MFP_UART0_TXD;

    /* Init GPIO for MS500 Reset and Set High */
    GPIO_SetMode(PC, BIT13, GPIO_MODE_OUTPUT);
    PC13 = 1;

#ifndef NO_MASS_STORAGE
    /* Init GPIO for Flash Memory WP and Set High */
    GPIO_SetMode(PC, BIT12, GPIO_MODE_OUTPUT);
    PC12 = 1;
#endif

    GPIO_SetMode(PC, BIT2, GPIO_MODE_INPUT);
}

/* UART0 for Terminal */
void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC_DIV2, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*
 * Init SPI0 for Flash Memory
 *     Configure as a master, SPI clock rate = 24 MHz
 *     clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge.
 */
void SPI0_Init(void)
{
#ifndef NO_MASS_STORAGE
    /* Set IP clock divider. */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 32, 24000000UL);

    /* Disable the automatic hardware slave select function. Set SPI0 SS as High */
    SPI_DisableAutoSS(SPI0);
    SPI_SET_SS_HIGH(SPI0);
#endif
}

/*
 * Init USCI SPI0 for Communicate to MS500
 *     Configure USCI_SPI0 as a master, USCI_SPI0 clock rate 5 MHz,
 *     clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge.
 */
void USCI_SPI0_Init(void)
{
    /* Set IP clock divider. */
#ifdef NO_SPI_16_BIT
    USPI_Open(USPI0, USPI_MASTER, USPI_MODE_0, 8, 4000000UL);
#else
	  USPI_Open(USPI0, USPI_MASTER, USPI_MODE_0, 16, 5000000UL);
#endif

    /* Enable the automatic hardware slave select function. */
    USPI_EnableAutoSS(USPI0, 0, USPI_SS_ACTIVE_LOW);
}

void GPIO_INT_Init(void)
{
    /* Init GPIO for USPI Interrupt */
    GPIO_SetMode(PB, BIT14, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 14, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPAB_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_HCLK, GPIO_DBCTL_DBCLKSEL_16);
    GPIO_ENABLE_DEBOUNCE(PB, BIT14);
}
#if 1
static void __io_delay(uint32_t delay)
{
    volatile uint32_t i;
    for ( i=0; i<=delay; i++ )
    {
        __NOP();
    }
}
#endif

void BLE_TEST(void)
{
	uint8_t testBuf_t[64] = {0,};
	volatile uint32_t i;

	//if( PB14 == 1 )
	{
		//memset(testBuf_t, 0, 64);
		USPICommRead(testBuf_t, 64);			
		
		for(i =0 ; i < 64 ; i++)
			DBG_PRINTF("%x ", testBuf_t[i]);
			
		DBG_PRINTF("\n");
		
		DBG_PRINTF("READ TEST COMPLETE\n\n");
	}
	__io_delay(10000000);
}

/* Main Function */
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init system and multi-function I/O */
    SYS_Init();

    SYS_LockReg();

    /* Init GPIO for SPI Int */
    GPIO_INT_Init();

    /* Init UART for debug message */
    UART0_Init();
    DBG_PRINTF("\n");
    DBG_PRINTF("NUC121 Start %s %s\n", __DATE__, __TIME__);

#ifndef NO_MASS_STORAGE
    /* Init SPI0 for Flash Memory */
    SPI0_Init();
#endif

    /* Init USCI SPI0 for Communicate to MS500 */
    USCI_SPI0_Init();

    /* MS500 Reset( 1ms ) */
    PC13 = 0;
    CLK_SysTickDelay(1000);
    PC13 = 1;

    /* Wait for the MS500 initialize */
    while ( PC2 != 1 )
    DBG_PRINTF("MS500 Initialize done. USB Start\n");
		
    /* Open USB controller */
    USBD_Open(&gsInfo, HID_MSC_ClassRequest, NULL);

#ifndef NO_MASS_STORAGE
    /* Set SET CONFIGURATION request callback function */
    USBD_SetConfigCallback(MSC_SetConfig);
#endif

    /* Endpoint configuration for FIDO */
    HID_MSC_Init();

    /* Start USB device */
    USBD_Start();
    NVIC_EnableIRQ(USBD_IRQn);

    while (1)
    {
		BLE_TEST();
		
        HID_MSC_ProcessCmd();
    }
}
