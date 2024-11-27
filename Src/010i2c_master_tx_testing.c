/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Jul 18, 2024
 *      Author: Rahul
 *    Exercise
 *    I2C Master (STM32Discovery ) and I2C Slave (Arduino Board) Communication
 *    when the button on stm32 board (master) is pressed , master Should send data to the arduino board (Slave).
 *    The data received by the Arduino board will be Displaced on the serial monitor terminol of the Arduino IDE
 *    1. Use I2C SCL= 100KHz (standard mode) .
 *    2. Use External pull up resistor (3.3K-ohm) for SDA and SCL line .
 *
 *    Note: If you don't have external pull up resistor , you can also try with activating the STM32 I2C pins internal pull up resistors.
 *
 *    Things you Need
 *    1. Arduino Board
 *    2. STM32 Board
 *    3. Logic Level Converter
 *    4. Breadboard and jumper wires
 *    5. 2 pull-up resistors of value 3.3Kohm or 4.7Kohm (you can also use internal pull up resistors of the pins for testing purpose)
 */


#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"

#define MY_ADDR     0x61

#define SLAVE_ADDR  0x68    //read form slave
void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

I2C_Handle_t I2C1Handle;

//some data
uint8_t some_data[] = "We are testing I2C master Tx\n";

/*
 * PB6 -> SCL
 * PB9 -> SDA
 */


void I2C_GPIOInits(void)
{
	GPIO_Handle_t I2CPins ;
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	/*
	 * Note : In the below line use GPIO_NO_PUPD option if you want to use external pullup resistors, then you have to use 3.3K pull up resistors
	 * for both SDA and SCL lines
	*/
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//SDA
	//Note : since we found a glitch on PB9 , you can also try with PB7
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;

	GPIO_Init(&I2CPins);




}

void I2C1_Init(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress =MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);

}
void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

}

int main(void)
{
	GPIO_ButtonInit();

	//i2c pin inits
	I2C_GPIOInits();

	//i2c peripheral configuration
	I2C1_Init();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	while(1)
		{
			//wait till button is pressed
			while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

			//to avoid button de-bouncing related issues 200ms of delay
			delay();

			//send some data to the slave
			I2C_MasterSendData(&I2C1Handle,some_data,strlen((char*)some_data),SLAVE_ADDR,0);

		}

}
