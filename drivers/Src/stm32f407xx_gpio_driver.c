/*
 * stm32f407xx_gpio.c
 *
 *  Created on: Jun 21, 2024
 *      Author: Rahul
 */


#include "stm32f407xx_gpio_driver.h"

/*
 *  Peripheral clock setup
 */
/********************************************************
 * @fn        - GPIO_PeriClockControl
 *
 * @brief     -
 *
 * @param[in] -Base address of the gpio peripherals
 * @param[in] -ENABLE or DISABLE Macros (1 to enable, 0 to disable).
 * @param[in] -Enables or disables peripheral clock for the given GPIO port.
 *
 *
 * @return    - none
 *
 *
 * @Note      - none
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx ,uint8_t EnorDi)
{
	if (EnorDi== ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN() ;
		}else if(pGPIOx ==GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx ==GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx ==GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx ==GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx ==GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx ==GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx ==GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if(pGPIOx ==GPIOI)
		{
			GPIOI_PCLK_EN();
		}

	}else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI() ;
		}else if(pGPIOx ==GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx ==GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if(pGPIOx ==GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if(pGPIOx ==GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if(pGPIOx ==GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if(pGPIOx ==GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if(pGPIOx ==GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if(pGPIOx ==GPIOI)
		{
			GPIOI_PCLK_DI();
		}

	}
}




/********************************************************
 * @fn        - GPIO_Init
 *
 * @brief     -
 *
 * @brief  Initializes the GPIO port.
 * @param  pGPIOHandle: Pointer to GPIO handle structure which contains the configuration information for the specified GPIO.
 *
 *
 * @return    - none
 *
 *
 * @Note      - none
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	 uint32_t temp=0; //temp. register

	 //Enable the peripheral Clock
	 GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);

	//1. Configure the mode of GPIO pin

	 if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
		{
			//the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
			pGPIOHandle->pGPIOx->MODER |= temp; //setting

		}else
	{

			// This part will code later .( interrupt mode)
			if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_FT )
			{
				//1. Configure the FTSR
				EXTI->FTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );//Set

				EXTI->RTSR &=~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );//Clear the corresponding RTSR bit

			}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_RT)
			{
				//1. Configure the RTSR
				EXTI->RTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );//Set

				EXTI->FTSR &=~ (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );//Clear the corresponding FTSR bit

			}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_RFT)
			{
				EXTI->FTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );//Set
				EXTI->RTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );//Set
			}

			//2. COnfigure the GPIO port selection in SYSCFG_EXTICR

			uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /4 ;
			uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4 ;
			uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[temp1]= portcode << (temp2 * 4);


			//3. COnfigure the EXTI interrupt delivery using IMR
			EXTI ->IMR |=1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ;








	}
	temp=0;

	//2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//3. Configure the pupd setting
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	//4. Configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5. Configure the alt functionality
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    	{
    		//configure the alt function registers.
    		uint8_t temp1, temp2;

    		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
    		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
    		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
    		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
    	}
}



/********************************************************
 * @fn        - void GPIO_DeInit
 *
 * @brief     -
 *
 * @brief  De-initializes the GPIO port.
 * @param  pGPIOx: Base address of the GPIO port.
 *
 *
 * @return    - none
 *
 *
 * @Note      - none
 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET() ;
	}else if(pGPIOx ==GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx ==GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx ==GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx ==GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx ==GPIOF)
	{
		GPIOF_REG_RESET();
	}else if(pGPIOx ==GPIOG)
	{
		GPIOG_REG_RESET();
	}else if(pGPIOx ==GPIOH)
	{
		GPIOH_REG_RESET();
	}else if(pGPIOx ==GPIOI)
	{
		GPIOI_REG_RESET();
	}



}

//Data read and write

/*
 *  GPIO_ReadFromInputPin
 */
/********************************************************
 * @fn        - GPIO_ReadFromInputPin
 *
 * @brief     -
 *
 * @brief  Reads the value from the specified GPIO pin.
 * @param  pGPIOx: Base address of the GPIO port.
 * @param  PinNumber: GPIO pin number to read from.
 *
 *
 * @return    - Pin state (0 or 1).
 *
 *
 * @Note      - none
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx ,uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber)& 0x00000001 );

	return value ;
}

/*
 *  GPIO_ReadFromInputPort
 */
/********************************************************
 * @fn        - GPIO_ReadFromInputPort
 *
 * @brief     -
 *
 * @brief  Reads the value from the entire GPIO port.
 * @param  pGPIOx: Base address of the GPIO port.
 *
 *
 * @return    - Port value (16-bit)
 *
 *
 * @Note      - none
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value ;
}

/*
 *  GPIO_WriteToOutputPin
 */
/********************************************************
 * @fn        - GPIO_WriteToOutputPin
 *
 * @brief     -
 *
 * @brief  Writes a value to the specified GPIO pin.
 * @param  pGPIOx: Base address of the GPIO port.
 * @param  PinNumber: GPIO pin number to write to.
 * @param  Value: Value to write (0 or 1).
 *
 *
 * @return    - none
 *
 *
 * @Note      - none
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value)
{
	if (Value== GPIO_PIN_SET)
	{
		// Write 1 to the output data register at the bit feild corresponding to pin number
		pGPIOx->ODR |= (1<< PinNumber);
	}else
	{
		// write 0
		pGPIOx->ODR &=~(1<< PinNumber);
	}
}

/*
 *  GPIO_WriteToOutputPort
 */
/********************************************************
 * @fn        - GPIO_WriteToOutputPort
 *
 * @brief  Writes a value to the entire GPIO port.
 * @param  pGPIOx: Base address of the GPIO port.
 * @param  Value: 16-bit value to write to the port
 *
 *
 * @return    - none
 *
 *
 * @Note      - none
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/*
 *  GPIO_ToggleOutputPin
 */
/********************************************************
 * @fn        - GPIO_ToggleOutputPin
 *
 * @brief  Toggles the value of the specified GPIO pin.
 * @param  pGPIOx: Base address of the GPIO port.
 * @param  PinNumber: GPIO pin number to toggle.
 *
 *
 * @return    - none
 *
 *
 * @Note      - none
 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

//IRQ Configuration and ISR handling

/*
 *  GPIO_IRQConfig
 */
/********************************************************
 * @fn        - GPIO_IRQConfig
 *
 * @brief  Configures the IRQ for the specified GPIO pin.
 * @param  IRQNumber: IRQ number to configure.
 * @param  IRQPriority: Priority of the IRQ.
 * @param  ENorDi: Enable or disable macro (1 to enable, 0 to disable).
 *
 *
 * @return    - none
 *
 *
 * @Note      - none
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber ,uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if(IRQNumber <=31 )
		{
			// Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if (IRQNumber > 31 && IRQNumber < 64)
		{
			// Program ISER1 register
			 *NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			// Program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			// Program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber <64 )
		{
			// Program ICER0 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32));
		}else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			// Program ICER0 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64)) ;
		}
	}
}

/********************************************************
 * @fn        - GPIO_IRQPriorityConfig
 *
 * @brief  Configures the IRQ for the specified GPIO pin.
 * @param  IRQNumber: IRQ number to configure.
 * @param  IRQPriority: Priority of the IRQ.
 * @param  ENorDi: Enable or disable macro (1 to enable, 0 to disable).
 *
 *
 * @return    - none
 *
 *
 * @Note      - none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber ,uint8_t IRQPriority)
{
	//1. First lets find out the IPR register
	uint8_t iprx = IRQNumber /4 ;
	uint8_t iprx_section = IRQNumber /4;

	uint8_t shift_amount = ( 8* iprx_section )+ (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}


/*
 *  GPIO_IRQHandling
 */
/********************************************************
 * @fn        - GPIO_IRQHandling
 *
 * @brief  Handles the IRQ for the specified GPIO pin.
 * @param  PinNumber: GPIO pin number which generated the IRQ.
 *
 *
 * @return    - none
 *
 *
 * @Note      - none
 */

void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear the EXTI PR register corresponding to the pin number
	if (EXTI -> PR & ( 1<< PinNumber))
	{
		//Clear
		EXTI ->PR |= ( 1 << PinNumber);

	}
}
