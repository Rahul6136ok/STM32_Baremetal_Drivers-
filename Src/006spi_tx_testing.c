/*
 * 006spi_tx_testing.c
 *
 *  Created on: Jul 9, 2024
 *      Author: Rahul
 *  Exercise :
 *  1. Test the SPI_sendData API to send the string "Hello world" and use the below configurations
 *   1) SPI-2 Master mode
 *   2)SCLK =max possible
 *   3)DFF=0 and DFF=1;
 *   4)Find out the MCU pins over which SPI2 peripherals communicates to the External world
 *   5)For this application we don't want to use MISO and NSS because
 *     there is no slave so don't configure those pins
 *   6)Chances of getting MODF fault - Cleared by SSI
 */
#include<String.h>
#include "stm32f407xx.h"

//PB14 -- SPI2_MISO
//PB15 -- SPI2_MOSI
//PB13 -- SPI2_SCLK
//PB12 -- SPI2_NSS
// Alternate function mode: 5


void SPI2_inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx =SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2 ; //generates sclk of 8MH
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN ; // Software slave management enabled for NSS pins

	SPI_Init(&SPI2handle);
}

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins ;
	SPIPins.pGPIOx=GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5 ;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13 ;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15 ;
	GPIO_Init(&SPIPins);

	//MISO        For this application we don't want to use MISO and NSS because there is no slave so don't configure those pins
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14 ;
	//GPIO_Init(&SPIPins);
	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12 ;
	//GPIO_Init(&SPIPins);
}

int main(void)
{
	char user_data[]= "Hello world" ;

	//this function is used to initialize the GPIO pins to behave as SPI2 Pins
	SPI2_GPIOInits();

	// This function is used to initialize the SPI2 peripheral parameter
	SPI2_inits();

	// This makes NSS signal internally high and and avoids MODF error
	SPI_SSIConfig(SPI2,ENABLE);

	//Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,ENABLE);

	// TO send data
	SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

	//lets confirm SPI is not busy
    //while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	//Disable the peripheral
	SPI_PeripheralControl(SPI2,DISABLE);

	while(1);

	return 0;

}
