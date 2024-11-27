/*
 * 007spi_txonly_arduino.c
 *
 *  Created on: Jul 10, 2024
 *      Author: Rahul
 *  Test the SPI to send the string "Hello world"
 *  SPI-2 Master mode
 *  SCLK= Max possible
 *   do the transmission only if Master button is pressed
 */

#include<String.h>
#include "stm32f407xx.h"

//PB14 -- SPI2_MISO
//PB15 -- SPI2_MOSI
//PB13 -- SPI2_SCLK
//PB12 -- SPI2_NSS
// Alternate function mode: 5

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

void SPI2_inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx =SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2 ; //generates sclk of 2MH
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI ; // Hardware slave management enabled for NSS pins

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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12 ;
	GPIO_Init(&SPIPins);
}


void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;
	//This is Btn GPIO Configuration
	    GPIOBtn.pGPIOx = GPIOA;
	    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	    // GPIOBtn_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP; only applicable when mode is output
	    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	    GPIO_Init(&GPIOBtn);
}

int main(void)
{
	char user_data[]= "Hello world";

	GPIO_ButtonInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 Pins
	SPI2_GPIOInits();

	// This function is used to initialize the SPI2 peripheral parameter
	SPI2_inits();

	// This makes NSS signal internally high and and avoids MODF error only in Software slave management
	//SPI_SSIConfig(SPI2,ENABLE);
	/*
	 * make SSOE 1 does NSS output enabled .
	 * the NSS pins is automatically managed by the hardware .
	 * i.e. when SPE=1 , NSS will be pulled to low
	 * and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2,ENABLE);
	while(1)          // code loop
	{
	     // wait till Button is pressed
	     while (!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

	     // to avoid button de-bouncing related issues 200ms of delay
	     delay();

	     //Enable the SPI2 peripheral
	     SPI_PeripheralControl(SPI2,ENABLE);

	     //First send length information
	     uint8_t dataLen = strlen (user_data);
	     SPI_SendData(SPI2,&dataLen,1);

	     // TO send data
	     SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

	     //lets confirm SPI is not busy
	     while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

	     //Disable the peripheral
	     SPI_PeripheralControl(SPI2,DISABLE);
	}


	return 0;

}
