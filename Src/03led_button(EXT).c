/*
 * 03led_button.c
 *
 *  Created on: Jun 24, 2024
 *      Author: Rahul
 */

#include "stm32f407xx.h"

#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}


int main(void)
{
    GPIO_Handle_t GpioLed, GPIOBtn;

    //This is led GPIO Configuration
    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOA, ENABLE);

    GPIO_Init(&GpioLed);



    //This is Btn GPIO Configuration
    GPIOBtn.pGPIOx = GPIOB;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;
    // GPIOBtn_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP; only applicable when mode is output
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOB, ENABLE);

    GPIO_Init(&GPIOBtn);

    while(1)
    {
        if (GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == BTN_PRESSED)
    	{
        	delay();
        	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
    	}

    }

    return 0;
}

