/*
 * stm32f4xx_rcc_driver.c
 *
 *  Created on: Jul 23, 2024
 *      Author: Rahul
 */

#include "stm32f4xx_rcc_driver.h"
uint16_t AHB_PreScaler[8] = {2,4,8,16,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};


uint32_t RCC_GetPCLK1Value(void)
{
    uint32_t pclk1, systemClk;
    uint8_t clksrc, temp, ahbp, apbp1;

    // Get clock source
    clksrc = ((RCC->CFGR >> 2) & 0x3);

    // Determine system clock
    if (clksrc == 0)
    {
        systemClk = 16000000; // HSI oscillator clock
    }
    else if (clksrc == 1)
    {
        systemClk = 8000000; // HSE oscillator clock
    }
    else if (clksrc == 2)
    {
        systemClk = RCC_GetPLLOutputClock(); // PLL clock
    }

    // Get AHB prescaler
    temp = ((RCC->CFGR >> 4) & 0xF);
    if (temp < 8)
    {
        ahbp = 1;
    }
    else
    {
        ahbp = AHB_PreScaler[temp - 8];
    }

    // Get APB1 prescaler
    temp = ((RCC->CFGR >> 10) & 0x7);
    if (temp < 4)
    {
        apbp1 = 1;
    }
    else
    {
        apbp1 = APB1_PreScaler[temp - 4];
    }

    // Calculate PCLK1
    pclk1 = (systemClk / ahbp) / apbp1;

    return pclk1;
}



uint32_t RCC_GetPCLK2Value(void)
    {
	uint32_t SystemClock=0,tmp,pclk2;
		uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;

		uint8_t ahbp,apb2p;

		if(clk_src == 0)
		{
			SystemClock = 16000000;
		}else
		{
			SystemClock = 8000000;
		}
		tmp = (RCC->CFGR >> 4 ) & 0xF;

		if(tmp < 0x08)
		{
			ahbp = 1;
		}else
		{
	       ahbp = AHB_PreScaler[tmp-8];
		}

		tmp = (RCC->CFGR >> 13 ) & 0x7;
		if(tmp < 0x04)
		{
			apb2p = 1;
		}else
		{
			apb2p = APB1_PreScaler[tmp-4];
		}

		pclk2 = (SystemClock / ahbp )/ apb2p;

		return pclk2;
    }

uint32_t RCC_GetPLLOutputClock(void)
        {
            uint32_t pllInputClock = 16000000; // Assuming HSI is used as PLL source
            uint32_t pllOutputClock;

            // Check if HSE is used as PLL source
            if ((RCC->PLLCFGR & (1 << 22)) != 0)
            {
                pllInputClock = 8000000; // HSE value, modify if different
            }

            // Calculate PLL output clock
            pllOutputClock = (pllInputClock / PLL_M) * PLL_N / PLL_P;

            return pllOutputClock;
        }
