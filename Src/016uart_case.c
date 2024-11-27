/*
 * 016uart_case.c
 *
 *  Created on: Jul 23, 2024
 *      Author: Rahul
 *      Semi Hosting - Add Linker flag
 *  Write a Program for stm32 board which transmits different messages to the arduino board
 *  over UART Communication.
 *  for Every message STM32 board sends , arduino code wil change the case of alphabets
 *  (Lower case to Upper case and Vice versa ) and sends message back to the STM32 board
 *  The stm32 board should capture the reply from the arduino
 *  board display using Semi Hosting
 */
#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"

// We have 3 different messages that we transmit to Arduino.
// You can by all means add more messages.
char *msg[3]= {"hihihihihihihihihi123","Hello How are you?","Today is Monday !"};

// Reply from Arduino will be stored here.
char rx_buf[1024];

USART_Handle_t usart2_handle;

// This flag indicates reception completion.
uint8_t rxCmplt = RESET;

extern void initialise_monitor_handles();

void USART2_Init(void) {
    usart2_handle.pUSARTx = USART2;
    usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
    usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;
    usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
    usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
    USART_Init(&usart2_handle);
}

void USART2_GPIOInit(void) {
    GPIO_Handle_t usart_gpios;
    usart_gpios.pGPIOx = GPIOA;
    usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

    // Initialize PA2 as USART2 TX
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIO_Init(&usart_gpios);

    // Initialize PA3 as USART2 RX
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    GPIO_Init(&usart_gpios);
}

void GPIO_ButtonInit(void) {
    GPIO_Handle_t GPIOBtn, GpioLed;

    // This is button GPIO configuration
    GPIOBtn.pGPIOx = GPIOA;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIOBtn);

    // This is LED GPIO configuration
    GpioLed.pGPIOx = GPIOD;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOD, ENABLE);

    GPIO_Init(&GpioLed);
}

void delay(void) {
    for(uint32_t i = 0; i < 500000 / 2; i++);
}

int main(void) {
    uint32_t cnt = 0;

    initialise_monitor_handles();

    GPIO_ButtonInit();
    USART2_GPIOInit();
    USART2_Init();

    USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE);

    USART_PeripheralControl(USART2, ENABLE);

    printf("Application is running\n");

    // Do forever
    while(1) {
        // Wait till button is pressed
        while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

        // To avoid button de-bouncing related issues, add a 200ms delay
        delay();

        // Next message index; make sure that cnt value doesn't exceed 2
        cnt = cnt % 3;

        // First let's enable the reception in interrupt mode
        // This code enables the receive interrupt
        while (USART_ReceiveDataIT(&usart2_handle, (uint8_t*)rx_buf, strlen(msg[cnt])) != USART_READY);

        // Send the message indexed by cnt in blocking mode
        USART_SendData(&usart2_handle, (uint8_t*)msg[cnt], strlen(msg[cnt]));

        printf("Transmitted: %s\n", msg[cnt]);

        // Now let's wait until all the bytes are received from the Arduino.
        // When all the bytes are received, rxCmplt will be SET in application callback
        while(rxCmplt != SET);

        // Just make sure that the last byte should be null otherwise %s fails while printing
        rx_buf[strlen(msg[cnt])] = '\0';

        // Print what we received from the Arduino
        printf("Received: %s\n", rx_buf);

        // Invalidate the flag
        rxCmplt = RESET;

        // Move on to the next message indexed in msg[]
        cnt++;
    }

    return 0;
}

void USART2_IRQHandler(void)
{
    USART_IRQHandling(&usart2_handle);
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t ApEv) {
    if(ApEv == USART_EVENT_RX_CMPLT)
    {
        rxCmplt = SET;
    } else if (ApEv == USART_EVENT_TX_CMPLT)
    {
        // Transmission complete event
    }
}




