#ifndef INC_STM32L4XX_USART_DRIVER_H_
#define INC_STM32L4XX_USART_DRIVER_H_

#include <stdint.h>
#include <stddef.h>
#include "stm32l476xx.h"

#define USART_MODE_TX   0
#define USART_MODE_RX   1
#define USART_MODE_TXRX 2

// TODO recheck these for 4 Mhz or set clock to 16 on init of UART
#define USART_STD_BAUD_1200                 1200
#define USART_STD_BAUD_2400                 400
#define USART_STD_BAUD_9600                 9600
#define USART_STD_BAUD_19200                19200
#define USART_STD_BAUD_38400                38400
#define USART_STD_BAUD_57600                57600
#define USART_STD_BAUD_115200               115200
#define USART_STD_BAUD_230400               230400
#define USART_STD_BAUD_460800               460800
#define USART_STD_BAUD_921600               921600
#define USART_STD_BAUD_2M                   2000000
#define SUART_STD_BAUD_3M                   3000000

#define USART_PARITY_EN_ODD     2
#define USART_PARITY_EN_EVEN    1
#define USART_PARITY_DISABLE    0

#define USART_WORDLEN_7BITS  2
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

#define USART_HW_FLOW_CTRL_NONE     0
#define USART_HW_FLOW_CTRL_CTS      1
#define USART_HW_FLOW_CTRL_RTS      2
#define USART_HW_FLOW_CTRL_CTS_RTS  3

typedef struct {
    uint8_t USART_mode; // transmit/receive or both
    uint32_t UASRT_Baud;
    uint8_t UASRT_NoOfStopBits;
    uint8_t UASRT_WordLength;
    uint8_t UASRT_ParityControl; // no, even or odd
    uint8_t UASRT_HwFlowControl;
} USART_Config_t;

typedef struct {
    USART_RegDef_t *pUSARTx;
    USART_Config_t USART_Config;
} USART_Handle_t;

void USART_PCLK(USART_RegDef_t *pUSARTx, uint8_t isEnable);

void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

void USART_SetBaudRate(USART_Handle_t *pUSARTHandle, uint32_t baud_rate);

void USART_Write(USART_Handle_t *pUSARTHandle, uint8_t *source, uint8_t size);
void USART_Read(USART_Handle_t *pUSARTHandle, uint8_t *destination, uint8_t size);

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t isEnable);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void USART_Enable(USART_RegDef_t *pUSARTx, uint8_t isEnable);

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t flagMask);

void USART_AppEventCallback(USART_Handle_t *pHandle, uint8_t event);

#endif /* INC_STM32L4XX_USART_DRIVER_H_ */
