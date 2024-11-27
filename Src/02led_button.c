/*
 * 02led_button.c
 *
 *  Created on: Jun 24, 2024
 *      Author: Rahul
 */


#include "stm32f407xx.h"

#define HIGH 1
#define BTN_PRESSED HIGH

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}


int main(void)
{
    GPIO_Handle_t GpioLed, GPIOBtn;

    //This is led GPIO Configuration
    GpioLed.pGPIOx = GPIOD;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&GpioLed);



    //This is Btn GPIO Configuration
    GPIOBtn.pGPIOx = GPIOA;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;
    // GPIOBtn_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP; only applicable when mode is output
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&GPIOBtn);

    while(1)
    {
        if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED)
    	{
        	delay();
        	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
    	}

    }

    return 0;
}


//see diagram already have pull down resistor in Discovery Board
