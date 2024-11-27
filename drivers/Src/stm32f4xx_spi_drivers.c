
#include "stm32f407xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);



/*
 *  Peripheral clock setup
 */
/********************************************************
 * @fn        - SPI_PeriClockControl
 *
 * @brief     -
 *
 * @param[in] -Base address of the SPI peripherals
 * @param[in] -ENABLE or DISABLE Macros (1 to enable, 0 to disable).
 * @param[in] -Enables or disables peripheral clock for the given SPI port.
 *
 *
 * @return    - none
 *
 *
 * @Note      - none
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx ,uint8_t EnorDi)
{
	if (EnorDi== ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN() ;
			}else if(pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}else if(pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}

		}else
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI() ;
			}else if(pSPIx == SPI1)
			{
				SPI2_PCLK_DI() ;
			}else if(pSPIx == SPI1)
			{
				SPI3_PCLK_DI() ;
			}


		}
}

/********************************************************
 * @fn        - SPI_Init
 *
 * @brief     -
 *
 * @brief  Initializes the SPI port.
 * @param  pSPIHandle: Pointer to SPI handle structure which contains the configuration information for the specified SPI.
 *
 *
 * @return    - none
 *
 *
 * @Note      - none
 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	// Peripheral clock Enable
	SPI_PeriClockControl(pSPIHandle->pSPIx , ENABLE);

	//First lets configure the SPI_CR1 registers

	uint32_t tempreg = 0;
	// Configure the device mode
	tempreg = 0;
	//1. Configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the BusConfig
	if (tempreg |= pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD )
	{
		// bidi mode should be cleared
		tempreg &=~( 1 << SPI_CR1_BIDIMODE);
	}else if (tempreg |= pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD )
	{
		// bidi mode should be set
		tempreg |=( 1<< SPI_CR1_BIDIMODE);
	}else if (tempreg |= pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY )
	{
		// bidi mode should be cleared
		tempreg &=~( 1<< SPI_CR1_BIDIMODE);
		// RXONLY bit must be set
		tempreg |=( 1 << SPI_CR1_RXONLY);
	}

	//3. Configure the SPI serial Clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR ;

	//4. Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL ;

	//6. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx ->SPI_CR1 = tempreg;


}

/********************************************************
 * @fn        - void SPI_DeInit
 *
 * @brief     -
 *
 * @brief  De-initializes the SPI port.
 * @param  pSPIx: Base address of the SPI port.
 *
 *
 * @return    - none
 *
 *
 * @Note      - none
 */

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - This function de-initializes the SPI peripheral
 *
 * @param[in]         - Base address of the SPI peripheral
 *
 * @return            - None
 *
 * @Note              - None
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    if (pSPIx == SPI1)
    {
        SPI1_REG_RESET();
    }
    else if (pSPIx == SPI2)
    {
        SPI2_REG_RESET();
    }
    else if (pSPIx == SPI3)
    {
        SPI3_REG_RESET();
    }
}
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if (pSPIx->SPI_SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/********************************************************
 * @fn        - SPI_SendData
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
 * @Note      - This is a Blocking Call
 */
void SPI_SendData(SPI_RegDef_t *pSPIx ,uint8_t *pTxBuffer , uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set ** Here we are polling for the TXE flag to set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );

		//2. check the DFF bit in CR1
		if ((pSPIx->SPI_CR1 & (1<< SPI_CR1_DFF) ))
		{
			// 16 bit DFF
			//1. Load the data into the DR(Data Register)
			pSPIx->SPI_DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			// 8 bit DFF
			//1. Load the data into the DR(Data Register)
			pSPIx->SPI_DR = *pTxBuffer ;
			Len--;
			pTxBuffer++;
		}
	}
}


/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx ,uint8_t *pRxBuffer , uint32_t Len)
{
	while(Len > 0)
		{
			//1. wait until RXNE is set
			while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  == (uint8_t)FLAG_RESET );

			//2. check the DFF bit in CR1
			if( (pSPIx->SPI_CR1 & ( 1 << SPI_CR1_DFF) ) )
			{
				//16 bit DFF
				//1. load the data from DR to Rxbuffer address
				 *((uint16_t*)pRxBuffer) = pSPIx->SPI_DR ;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}else
			{
				//8 bit DFF
				*(pRxBuffer) = pSPIx->SPI_DR ;
				Len--;
				pRxBuffer++;
			}
		}

}

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 <<  SPI_CR1_SPE);
	}else
	{
		pSPIx->SPI_CR1 &= ~(1<< SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 <<  SPI_CR1_SSI);
	}else
	{
		pSPIx->SPI_CR1 &= ~(1<< SPI_CR1_SSI);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->SPI_CR2 |= (1 <<  SPI_CR2_SSOE);
	}else
	{
		pSPIx->SPI_CR2 &= ~(1<< SPI_CR2_SSOE);
	}
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

void SPI_IRQInterruptConfig(uint8_t IRQNumber ,uint8_t EnorDi)
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber ,uint8_t IRQPriority)
{
	//1. First lets find out the IPR register
	uint8_t iprx = IRQNumber /4 ;
	uint8_t iprx_section = IRQNumber /4;

	uint8_t shift_amount = ( 8* iprx_section )+ (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer ,uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1 . Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2.  Mark the SPI state as busy in transmission so that
		//    no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= ( 1 << SPI_CR2_TXEIE );

	}

	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer ,uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1 . Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2.  Mark the SPI state as busy in reception so that
		//    no other code can take over same SPI peripheral until reception is over
		   pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= ( 1 << SPI_CR2_RXNEIE );

	}


	return state;

}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1 ,temp2 ;
	//first lets check for TXE
	temp1=pHandle->pSPIx->SPI_SR & (1 << SPI_SR_TXE);
	temp2=pHandle->pSPIx->SPI_CR2 & (1<<SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//check for TXE
	temp1=pHandle->pSPIx->SPI_SR & (1 << SPI_SR_RXNE);
	temp2=pHandle->pSPIx->SPI_CR2 & (1<<SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// check for ovr flag
	temp1=pHandle->pSPIx->SPI_SR & (1 << SPI_SR_OVR);
	temp2=pHandle->pSPIx->SPI_CR2 & (1<<SPI_CR2_ERRIE);
	if (temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}


}


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//2. check the DFF bit in CR1
			if ((pSPIHandle->pSPIx->SPI_CR1 & (1<< SPI_CR1_DFF) ))
			{
				// 16 bit DFF
				//1. Load the data into the DR(Data Register)
				pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pRxBuffer);
				pSPIHandle->TxLen--;
				pSPIHandle->TxLen--;
				pSPIHandle->pTxBuffer++;
			}else
			{
				// 8 bit DFF
				//1. Load the data into the DR(Data Register)
				pSPIHandle->pSPIx->SPI_DR = *pSPIHandle->pTxBuffer ;
				pSPIHandle->TxLen--;
				pSPIHandle->pTxBuffer++;
			}
			if(! pSPIHandle->TxLen)
			{
				//TxLen is zero , so close the spi transmission and inform the application that
				//TX is over.

				//this prevents interrupts from setting up of TXE flag
				pSPIHandle->pSPIx->SPI_CR2 &=~(1<< SPI_CR2_TXEIE) ;
				pSPIHandle->pTxBuffer = NULL;
				pSPIHandle->TxLen = 0;
				pSPIHandle->TxState = SPI_READY;

				SPI_CloseTransmisson(pSPIHandle);
				SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
			}


}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//2. check the DFF bit in CR1
				if (pSPIHandle->pSPIx->SPI_CR1 & (1<< 11) )
				{
					// 16 bit DFF
					//1. Load the data into the DR(Data Register)
					*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->SPI_DR;
					pSPIHandle->RxLen-=2;
					pSPIHandle->pRxBuffer++;
					pSPIHandle->pRxBuffer++;
				}else
				{
					// 8 bit DFF
					//1. Load the data into the DR(Data Register)
					*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->SPI_DR;
					pSPIHandle->RxLen--;
					pSPIHandle->pTxBuffer++;
				}
				if(! pSPIHandle->RxLen)
				{
					//Reception is complete
					//Lets turn off the Rxne interrupt

					//this prevents interrupts from setting up of TXE flag
					pSPIHandle->pSPIx->SPI_CR2 &=~(1<< SPI_CR2_RXNEIE) ;
					pSPIHandle->pRxBuffer = NULL;
					pSPIHandle->RxLen = 0;
					pSPIHandle->RxState = SPI_READY;

					//SPI_CloseTransmisson(pSPIHandle);
					SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
				}

}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->SPI_DR;
		temp = pSPIHandle->pSPIx->SPI_SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}


void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &=~(1<< SPI_CR2_TXEIE) ;
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &=~(1<< SPI_CR2_RXNEIE) ;
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp ;
	temp = pSPIx->SPI_DR;
	temp =pSPIx->SPI_SR;
	(void)temp;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	//This is a weak implementation . the user application may override this function.
}


