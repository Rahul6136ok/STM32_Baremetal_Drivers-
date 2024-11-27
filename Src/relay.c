/*
 * relay.c
 *
 *  Created on: Jul 24, 2024
 *      Author: Rahul
 */


#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

void delay(void)
{
	for(uint32_t i = 0 ; i < 50000/2 ; i ++);
}

// Relay control commands
#define RELAY1_ON "RELAY1_ON"
#define RELAY1_OFF "RELAY1_OFF"
#define RELAY2_ON "RELAY2_ON"
#define RELAY2_OFF "RELAY2_OFF"
#define RELAY3_ON "RELAY3_ON"
#define RELAY3_OFF "RELAY3_OFF"
#define RELAY4_ON "RELAY4_ON"
#define RELAY4_OFF "RELAY4_OFF"

// Reply from ESP32 will be stored here.
char rx_buf[1024];

USART_Handle_t usart2_handle;
uint8_t rxCmplt = RESET;

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

void ESP32_SendCommand(char *command)
{
    USART_SendData(&usart2_handle, (uint8_t *)command, strlen(command));
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    USART_SendData(&usart2_handle, (uint8_t *)"\r\n", 2);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

void ESP32_Init(void)
{
    // Reset ESP32
    ESP32_SendCommand("AT+RST");
    delay();

    // Set Wi-Fi mode to Station
    ESP32_SendCommand("AT+CWMODE=1");
    delay();

    // Connect to Wi-Fi network
    ESP32_SendCommand("AT+CWJAP=\"SSID\",\"PASSWORD\"");
    delay();

    // Check connection status
    ESP32_SendCommand("AT+CIPSTATUS");
    delay();

    // Open TCP connection to server
    ESP32_SendCommand("AT+CIPSTART=\"TCP\",\"SERVER_IP\",PORT");
    delay();
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

void Relay_GPIOInit(void) {
    GPIO_Handle_t relay_gpios;

    relay_gpios.pGPIOx = GPIOB; // Assuming GPIOB for relays
    relay_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    relay_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    relay_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    relay_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    // Initialize relay pins
    relay_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIO_Init(&relay_gpios);

    relay_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
    GPIO_Init(&relay_gpios);

    relay_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIO_Init(&relay_gpios);

    relay_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    GPIO_Init(&relay_gpios);
}

void Relay_On(uint8_t relay_number) {
    switch(relay_number) {
        case 1: GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_0, GPIO_PIN_SET); break;
        case 2: GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_1, GPIO_PIN_SET); break;
        case 3: GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_2, GPIO_PIN_SET); break;
        case 4: GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_3, GPIO_PIN_SET); break;
    }
}

void Relay_Off(uint8_t relay_number) {
    switch(relay_number) {
        case 1: GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_0, GPIO_PIN_RESET); break;
        case 2: GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_1, GPIO_PIN_RESET); break;
        case 3: GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_2, GPIO_PIN_RESET); break;
        case 4: GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_3, GPIO_PIN_RESET); break;
    }
}

int main(void)
{
    USART2_GPIOInit();
    Relay_GPIOInit();
    USART2_Init();

    USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE);

    USART_PeripheralControl(USART2, ENABLE);

    printf("STM32 Application is running\n");

    // Initialize ESP32 and connect to Wi-Fi
    ESP32_Init();

    while(1)
    {
        // Wait until a message is received from ESP32
        while (USART_ReceiveDataIT(&usart2_handle, (uint8_t*)rx_buf, sizeof(rx_buf)) != USART_READY);

        // Control relays based on received message
        if (strcmp(rx_buf, RELAY1_ON) == 0) {
            Relay_On(1);
        } else if (strcmp(rx_buf, RELAY1_OFF) == 0) {
            Relay_Off(1);
        } else if (strcmp(rx_buf, RELAY2_ON) == 0) {
            Relay_On(2);
        } else if (strcmp(rx_buf, RELAY2_OFF) == 0) {
            Relay_Off(2);
        } else if (strcmp(rx_buf, RELAY3_ON) == 0) {
            Relay_On(3);
        } else if (strcmp(rx_buf, RELAY3_OFF) == 0) {
            Relay_Off(3);
        } else if (strcmp(rx_buf, RELAY4_ON) == 0) {
            Relay_On(4);
        } else if (strcmp(rx_buf, RELAY4_OFF) == 0) {
            Relay_Off(4);
        }

        // Clear the receive buffer for next message
        memset(rx_buf, 0, sizeof(rx_buf));
    }

    return 0;
}

void USART2_IRQHandler(void)
{
    USART_IRQHandling(&usart2_handle);
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t ApEv) {
    if(ApEv == USART_EVENT_RX_CMPLT) {
        rxCmplt = SET; // Set the completion flag
    }
}
