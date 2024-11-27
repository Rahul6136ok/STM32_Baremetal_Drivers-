/*
 * stm32f407xx.h
 *
 *  Created on: Jun 19, 2024
 *      Author: Rahul
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>
#include<stddef.h>



/***********************START :Processor specific Details********************************/
/*
 * ARM Cortex MX Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0  ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1  ((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2  ((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3  ((volatile uint32_t*)0xE000E10C)
#define NVIC_ISER4  ((volatile uint32_t*)0xE000E110)
#define NVIC_ISER5  ((volatile uint32_t*)0xE000E114)
#define NVIC_ISER6  ((volatile uint32_t*)0xE000E118)
#define NVIC_ISER7  ((volatile uint32_t*)0xE000E11C)

/*
 * ARM Cortex MX Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0  ((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1  ((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2  ((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3  ((volatile uint32_t*)0xE000E18C)
#define NVIC_ICER4  ((volatile uint32_t*)0xE000E190)
#define NVIC_ICER5  ((volatile uint32_t*)0xE000E194)
#define NVIC_ICER6  ((volatile uint32_t*)0xE000E198)
#define NVIC_ICER7  ((volatile uint32_t*)0xE000E19C)


/*
 * ARM Cortex MX Processor Priority Register Addresses calculation
 */
#define NVIC_PR_BASE_ADDR   ((volatile  uint32_t*) 0xE000E400)






#define NO_PR_BITS_IMPLEMENTED   4


// Base address of Flash and SRAM memories
#define FLASH_BASEADDR     0x08000000U
#define SRAM1_BASEADDR     0x20000000U
#define SRAM2_BASEADDR     0x2001C000U
#define ROM_BASEADDR       0x1FFF0000U
#define SRAM               SRAM1_BASEADDR

// AHBx and APBx Peripherals bus address
#define PERIPH_BASE        0x40000000U
#define APB1PERIPH_BASE    PERIPH_BASE
#define APB2PERIPH_BASE    0x40010000U
#define AHB1PERIPH_BASE    0x40020000U
#define AHB2PERIPH_BASE    0x50000000U

// Base address of peripherals which are hanging on AHB1 bus
#define GPIOA_BASEADDR     (AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASEADDR     (AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASEADDR     (AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASEADDR     (AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASEADDR     (AHB1PERIPH_BASE + 0x1000U)
#define GPIOF_BASEADDR     (AHB1PERIPH_BASE + 0x1400U)
#define GPIOG_BASEADDR     (AHB1PERIPH_BASE + 0x1800U)
#define GPIOH_BASEADDR     (AHB1PERIPH_BASE + 0x1C00U)
#define GPIOI_BASEADDR     (AHB1PERIPH_BASE + 0x2000U)
#define RCC_BASEADDR       (AHB1PERIPH_BASE + 0x3800U)


// Base address of peripherals which are hanging on APB1 Bus
#define I2C1_BASEADDR      (APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASEADDR      (APB1PERIPH_BASE + 0x5800U)
#define I2C3_BASEADDR      (APB1PERIPH_BASE + 0x5C00U)

#define SPI2_BASEADDR      (APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASEADDR      (APB1PERIPH_BASE + 0x3C00U)

#define USART2_BASEADDR    (APB1PERIPH_BASE + 0x4400U)
#define USART3_BASEADDR    (APB1PERIPH_BASE + 0x4800U)

#define UART4_BASEADDR     (APB1PERIPH_BASE + 0x4C00U)
#define UART5_BASEADDR     (APB1PERIPH_BASE + 0x5000U)

// Base address of peripherals which are hanging on APB2 Bus
#define EXTI_BASEADDR      (APB2PERIPH_BASE + 0x3C00U)
#define SPI1_BASEADDR      (APB2PERIPH_BASE + 0x3000U)
#define SYSCFG_BASEADDR    (APB2PERIPH_BASE + 0x3800U)
#define USART1_BASEADDR    (APB2PERIPH_BASE + 0x1000U)
#define USART6_BASEADDR    (APB2PERIPH_BASE + 0x1400U)

/********************peripherals register definition structures******************************/
/*
 * Note: Registers of a peripheral are specific to MCU
 * e.g: Number of Registers of SPI peripherals of STM32f4x family of MCU may be different(more or less)
 * Compared to numbers of registers of SPI peripherals of STM32Lx and STM32F0x family of MCUs
 * please check your Device RM
 */

typedef struct
{
	volatile uint32_t MODER;         //GPIO port mode register
	volatile uint32_t OTYPER;         //GPIO port output type register
	volatile uint32_t OSPEEDR;       //GPIO port output speed register
	volatile uint32_t PUPDR;          ////GPIO port pull-up/pull-down register
	volatile uint32_t IDR;            //GPIO port input data register
	volatile uint32_t ODR;            //GPIO port output speed register
	volatile uint32_t BSRR;            //GPIO port bit set/reset register
	volatile uint32_t LCKR;           //GPIO port configuration lock register
	volatile uint32_t AFR[2];         //GPIO alternate function high register
}GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t Reserved0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t Reserved1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t Reserved2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t Reserved3[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t Reserved;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t Reserved5[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t Reserved6[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
	volatile uint32_t CKGATENR;
	volatile uint32_t DCKCFGR2;
}RCC_RegDef_t;

/*
 * Peripheral definition structures for EXTI
 */

typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;


/*
 * Peripheral definition structures for SPI
 */
typedef struct
{
	volatile uint32_t SPI_CR1;
	volatile uint32_t SPI_CR2;
	volatile uint32_t SPI_SR;
	volatile uint32_t SPI_DR;
	volatile uint32_t SPI_CRCPR;
	volatile uint32_t SPI_RXCRCR;
	volatile uint32_t SPI_TXCRCR;
	volatile uint32_t SPI_I2SCFGR;
	volatile uint32_t SPI_I2SPR;
}SPI_RegDef_t;

/*
 * Peripheral definition structures for SYSCFG
 */

typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	volatile uint32_t RESERVED1[2];
	volatile uint32_t CMPCR;
	volatile uint32_t RESERVED2[2];
	volatile uint32_t CFGR;
}SYSCFG_RegDef_t;

/*
 * perioherals register definition structure of I2C
 */

typedef struct
{
	volatile uint32_t I2C_CR1;
	volatile uint32_t I2C_CR2;
	volatile uint32_t I2C_OAR1;
	volatile uint32_t I2C_OAR2;
	volatile uint32_t I2C_DR;
	volatile uint32_t I2C_SR1;
	volatile uint32_t I2C_SR2;
	volatile uint32_t I2C_CCR;
	volatile uint32_t I2C_TRISE;
	volatile uint32_t I2C_FLTR;
}I2C_RegDef_t;

/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	volatile uint32_t USART_SR;
	volatile uint32_t USART_DR;
	volatile uint32_t USART_BRR;
	volatile uint32_t USART_CR1;
	volatile uint32_t USART_CR2;
	volatile uint32_t USART_CR3;
	volatile uint32_t USART_GTPR;
}USART_RegDef_t;




// peripherals definition (peripheral base address typecasted to xxx_RegDef_t

#define GPIOA        ((GPIO_RegDef_t*) GPIOA_BASEADDR )
#define GPIOB        ((GPIO_RegDef_t*) GPIOB_BASEADDR )
#define GPIOC        ((GPIO_RegDef_t*) GPIOC_BASEADDR )
#define GPIOD        ((GPIO_RegDef_t*) GPIOD_BASEADDR )
#define GPIOE        ((GPIO_RegDef_t*) GPIOE_BASEADDR )
#define GPIOF        ((GPIO_RegDef_t*) GPIOF_BASEADDR )
#define GPIOG        ((GPIO_RegDef_t*) GPIOG_BASEADDR )
#define GPIOH        ((GPIO_RegDef_t*) GPIOH_BASEADDR )
#define GPIOI        ((GPIO_RegDef_t*) GPIOI_BASEADDR )

#define RCC ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1       ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2       ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3       ((SPI_RegDef_t*)SPI2_BASEADDR)

#define I2C1       ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2       ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3       ((I2C_RegDef_t*)I2C3_BASEADDR)


#define USART1  	((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  	((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  	((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  		((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  		((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  	((USART_RegDef_t*)USART6_BASEADDR)

//Clock Enable Macros for GPIOx peripherals

#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0)) //GPIOA_PERI_CLOCK_ENABLE
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() (RCC->AHB1ENR |= (1 << 8))


//Clock Enable Macros for I2Cx peripherals

#define I2C1_PCLK_EN() (RCC->APB1ENR |= ( 1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= ( 1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= ( 1 << 23))

//Clock Enable Macros for SPIx peripherals

#define SPI1_PCLK_EN() (RCC->APB1ENR |= ( 1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= ( 1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= ( 1 << 15))
#define SPI4_PCLK_EN() (RCC->APB1ENR |= ( 1 << 13))

//Clock Enable Macros for USARTx peripherals

#define USART1_PCLK_EN() (RCC->APB2ENR |= ( 1 << 14))
#define USART2_PCLK_EN() (RCC->APB1ENR |= ( 1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= ( 1 << 18))
//Clock Enable Macros for UARTx peripherals

#define UART4_PCLK_EN()  (RCC->APB1ENR |= ( 1 << 19))
#define UART5_PCLK_EN()  (RCC->APB1ENR |= ( 1 << 20))
//Clock Enable Macros for SYSCFG peripherals

// Clock Enable Macros for SYSCFG peripherals
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

// Clock Disable Macros for SYSCFG peripherals
#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))





//Clock Disable Macros for GPIOx peripherals
#define GPIOA_PCLK_DI() (RCC->AHB1ENR &=~( 1 << 0)) //GPIOA_PERI_CLOCK_DISABLE
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &=~( 1 << 1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &=~( 1 << 2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &=~( 1 << 3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &=~( 1 << 4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &=~( 1 << 5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &=~( 1 << 6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &=~( 1 << 7))
#define GPIOI_PCLK_DI() (RCC->AHB1ENR &=~( 1 << 8))

//Clock Disable Macros for I2Cx peripherals
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~( 1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~( 1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~( 1 << 23))

//Clock Disable Macros for SPIx peripherals

#define SPI1_PCLK_DI() (RCC->APB1ENR |= ( 1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR |= ( 1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR |= ( 1 << 15))
#define SPI4_PCLK_DI() (RCC->APB1ENR |= ( 1 << 13))

//Clock Disable Macros for USARTx peripherals
#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI() (RCC->APB1ENR &=~( 1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR &=~( 1 << 18))

//Clock Disable Macros for UARTx peripherals
#define UART4_PCLK_DI()  (RCC->APB1ENR &=~( 1 << 19))
#define UART5_PCLK_DI()  (RCC->APB1ENR &=~( 1 << 20))

/*
 * Macros to reset GPIOx peripherals
 * do...while... condition zero loop
 * This is a Technique in 'C' programming to execute multiple
 * 'c' statement using single 'C' macro
 */
#define GPIOA_REG_RESET() do{ ((RCC->AHB1ENR |= (1 << 0))); (RCC->AHB1ENR &=~( 1 << 0));}while(0)
#define GPIOB_REG_RESET() do{ ((RCC->AHB1ENR |= (1 << 1))); (RCC->AHB1ENR &=~( 1 << 1));}while(0)
#define GPIOC_REG_RESET() do{ ((RCC->AHB1ENR |= (1 << 2))); (RCC->AHB1ENR &=~( 1 << 2));}while(0)
#define GPIOD_REG_RESET() do{ ((RCC->AHB1ENR |= (1 << 3))); (RCC->AHB1ENR &=~( 1 << 3));}while(0)
#define GPIOE_REG_RESET() do{ ((RCC->AHB1ENR |= (1 << 4))); (RCC->AHB1ENR &=~( 1 << 4));}while(0)
#define GPIOF_REG_RESET() do{ ((RCC->AHB1ENR |= (1 << 5))); (RCC->AHB1ENR &=~( 1 << 5));}while(0)
#define GPIOG_REG_RESET() do{ ((RCC->AHB1ENR |= (1 << 6))); (RCC->AHB1ENR &=~( 1 << 6));}while(0)
#define GPIOH_REG_RESET() do{ ((RCC->AHB1ENR |= (1 << 7))); (RCC->AHB1ENR &=~( 1 << 7));}while(0)
#define GPIOI_REG_RESET() do{ ((RCC->AHB1ENR |= (1 << 8))); (RCC->AHB1ENR &=~( 1 << 8));}while(0)


/*
 * Macros to reset SPIx peripherals
 * do...while... condition zero loop
 * This is a Technique in 'C' programming to execute multiple
 * 'c' statement using single 'C' macro
 */
#define SPI1_REG_RESET()            do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()            do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()            do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)

/*
 * Macros to reset I2Cx peripherals
 * do...while... condition zero loop
 * This is a Technique in 'C' programming to execute multiple
 * 'c' statement using single 'C' macro
 */
#define I2C1_REG_RESET()            do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()            do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()            do{ (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); }while(0)


#define GPIO_BASEADDR_TO_CODE(x)           ((x == GPIOA)?0:\
                                            (x == GPIOB)?1:\
                                            (x == GPIOC)?2:\
                                            (x == GPIOD)?3:\
                                            (x == GPIOE)?4:\
                                            (x == GPIOF)?5:\
                                            (x == GPIOG)?6:\
                                            (x == GPIOG)?7:0 ) //C conditional operator

/*
 * IRQ (Interrupt Request) Number of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * Todo : you may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0     6
#define IRQ_NO_EXTI1     7
#define IRQ_NO_EXTI2     8
#define IRQ_NO_EXTI3     9
#define IRQ_NO_EXTI4     10
#define IRQ_NO_EXTI9_5   23
#define IRQ_NO_EXTI15_10 40
#define IRQ_NO_SPI1      35
#define IRQ_NO_SPI2      36
#define IRQ_NO_SPI3      51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV   31
#define IRQ_NO_I2C1_ER   32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


/*
 * macros for all the possible priority levels
 */

#define NVIC_IRQ_PRI0  0
#define NVIC_IRQ_PRI15 15




// some generic macros
#define ENABLE           1
#define DISABLE          0
#define SET            ENABLE
#define RESET          DISABLE
#define GPIO_PIN_SET     SET
#define GPIO_PIN_RESET  RESET
#define FLAG_RESET      RESET
#define FLAG_SET         SET

/*******************************************************************************************************************
 ******Bit position definition for SPI peripherals   ***************************************************************
 ******************************************************************************************************************/
/*
 * Bit Position definitions SPI_CR1
 */
#define SPI_CR1_CPHA             0
#define SPI_CR1_CPOL             1
#define SPI_CR1_MSTR             2
#define SPI_CR1_BR               3
#define SPI_CR1_SPE              6
#define SPI_CR1_LSBFIRST         7
#define SPI_CR1_SSI              8
#define SPI_CR1_SSM              9
#define SPI_CR1_RXONLY          10
#define SPI_CR1_DFF             11
#define SPI_CR1_CRCNEXT         12
#define SPI_CR1_CRCEN           13
#define SPI_CR1_BIDIOE          14
#define SPI_CR1_BIDIMODE        15



/*
 * Bit Position definitions SPI_CPR2
 */
#define SPI_CR2_RXDMAEN  0
#define SPI_CR2_TXDMAEN  1
#define SPI_CR2_SSOE     2
#define SPI_CR2_FRF      4
#define SPI_CR2_ERRIE    5
#define SPI_CR2_RXNEIE   6
#define SPI_CR2_TXEIE    7

/*
 * Bit Position definitions SPI_SR
 */
#define SPI_SR_RXNE      0
#define SPI_SR_TXE       1
#define SPI_SR_CHSIDE    2
#define SPI_SR_UDR       3
#define SPI_SR_CRCERR    4
#define SPI_SR_MODF      5
#define SPI_SR_OVR       6
#define SPI_SR_BSY       7
#define SPI_SR_FRE       8


/*******************************************************************************************************************
 ******Bit position definition for I2C peripherals   ***************************************************************
 ******************************************************************************************************************/

/*
 * Bit Position definitions I2C_CR1
 */

#define I2C_CR1_PE         0
#define I2C_CR1_SMBUS      1
#define I2C_CR1_SMBTYPE    3
#define I2C_CR1_ENARPSMB   4
#define I2C_CR1_ENPEC      5
#define I2C_CR1_ENGC       6
#define I2C_CR1_NOSTRETCH  7
#define I2C_CR1_START      8
#define I2C_CR1_STOP       9
#define I2C_CR1_ACK        10
#define I2C_CR1_POS        11
#define I2C_CR1_PEC        12
#define I2C_CR1_ALERT      13
#define I2C_CR1_SWRST      15

/*
 * Bit Position definitions I2C_CR2
 */

#define I2C_CR2_FREQ       0
#define I2C_CR2_ITERREN    8
#define I2C_CR2_ITEVTEN    9
#define I2C_CR2_ITBUFEN    10
#define I2C_CR2_DMAEN       11
#define I2C_CR2_LAST       12

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD7_1 				 	 1
#define I2C_OAR1_ADD9_8  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit Position definitions I2C_SR1
 */
#define I2C_SR1_SB      0
#define I2C_SR1_ADDR    1
#define I2C_SR1_BTF     2
#define I2C_SR1_ADD10   3
#define I2C_SR1_STOPF   4
#define I2C_SR1_RxNE    6
#define I2C_SR1_TxE     7
#define I2C_SR1_BERR    8
#define I2C_SR1_ARLO    9
#define I2C_SR1_AF      10
#define I2C_SR1_OVR     11
#define I2C_SR1_PECERR  12
#define I2C_SR1_TIMEOUT  14
#define I2C_SR1_SMBALERT 15


/*
 * Bit Position definitions I2C_SR2
 */
#define I2C_SR2_MSL         0
#define I2C_SR2_BUSY        1
#define I2C_SR2_TRA         2
#define I2C_SR2_GENCALL     4
#define I2C_SR2_SMBDEFAULT  5
#define I2C_SR2_SMBHOST     6
#define I2C_SR2_DUALF       7
#define I2C_SR2_PEC         8


/*
 * Bit Position definitions I2C_CCR
 */
#define I2C_CCR_CCR    0
#define I2C_CCR_DUTY   14
#define I2C_CCR_F_S    15

/*
 * Bit Position definitions I2C_TRISE
 */

#define I2C_TRISE_TRISE 0



/*******************************************************************************************************************
 ******Bit position definition for USART peripherals   ***************************************************************
 ******************************************************************************************************************/
/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9




#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f4xx_rcc_driver.h"

















#endif /* INC_STM32F407XX_H_ */
