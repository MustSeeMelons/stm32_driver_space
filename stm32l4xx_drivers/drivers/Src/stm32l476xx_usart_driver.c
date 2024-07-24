#include "stm32l4xx_usart_driver.h"

void USART_PCLK(USART_RegDef_t *pUSARTx, uint8_t isEnable) {
    if(pUSARTx == USART1) {
        USART1_PCLK_EN();
    } else if (pUSARTx == USART2) {
        USART2_PCLK_EN();
    } else if (pUSARTx == USART3) {
        USART3_PCLK_EN();
    } else if (pUSARTx == UART4) {
        UART4_PCLK_EN();
    } else if (pUSARTx == UART5) {
        UART5_PCLK_EN();
    }
}

void USART_Init(USART_Handle_t *pUSARTHandle) {
    USART_Config_t config = pUSARTHandle->USART_Config;

    // Confifgure mode
    switch (config.USART_mode) {
        case USART_MODE_TX:
            pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TE);
            break;
        case USART_MODE_RX:
            pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RE);
            break;
        case USART_MODE_TXRX:
            pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TE);
            pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RE);
            break;
    }

    // Configure stop bits
    pUSARTHandle->pUSARTx->CR1 &= ~(0x3 << USART_CR2_STOP);
    pUSARTHandle->pUSARTx->CR1 |= (config.UASRT_NoOfStopBits << USART_CR2_STOP);

    // Configure word length
    switch (config.UASRT_WordLength) {
        case USART_WORDLEN_7BITS:
            pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_M0);
            pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_M1);
            break;
        case USART_WORDLEN_8BITS:
            pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_M0);
            pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_M1);
            break;
        case USART_WORDLEN_9BITS:
            pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_M0);
            pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_M1);
            break;
    }

    // Configure parity control
    switch (config.UASRT_ParityControl) {
        case USART_PARITY_DISABLE:
            pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_PCE);
            break;
        case USART_PARITY_EN_EVEN:

            pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_PCE);
            pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_PS);
            break;
        case USART_PARITY_EN_ODD:
            pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_PCE);
            pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_PS);
            break;
    }

    // configure flow control
    switch (config.UASRT_HwFlowControl) {
        case USART_HW_FLOW_CTRL_NONE:
            pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR3_RTSE);
            pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR3_CTSE);
            break;
        case USART_HW_FLOW_CTRL_CTS:
            pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR3_RTSE);
            pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR3_CTSE);
            break;
        case USART_HW_FLOW_CTRL_RTS:
            pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR3_CTSE);
            pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR3_RTSE);
            break;
        case USART_HW_FLOW_CTRL_CTS_RTS:
            pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR3_CTSE);
            pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR3_RTSE);
            break;
    }

    // Configure baud
    USART_SetBaudRate(pUSARTHandle, config.UASRT_Baud);
}

void USART_DeInit(USART_RegDef_t *pUSARTx) {
    if (pUSARTx == USART1) {
        USART1_REG_RESET();
    } else if (pUSARTx == USART2) {
        USART2_REG_RESET();
    } else if (pUSARTx == USART3) {
        USART3_REG_RESET();
    } else if (pUSARTx == UART4) {
        UART4_REG_RESET();
    } else if (pUSARTx == UART5) {
        UART5_REG_RESET();
    }
}

// TODO this function assumes way too much
void USART_SetBaudRate(USART_Handle_t *pUSARTHandle, uint32_t baud_rate) {
    // XXX debug shows mantisa/fraction part as the BRR but the datasheet does not, following the sheet

    // XXX assume 4 Mhz MSI (default), should make a function for this
    // XXX assume OVER8 = 0

    uint32_t usart_div = 4000000 / baud_rate;
    pUSARTHandle->pUSARTx->BRR = usart_div;

    // XXX If OVER8 = 1
    // BRR[3:0] has to equal USARTDIV[3:0] right shifted by 1
    // BRR[X:4] are USARTDIV[X:4]
    // BRR[3:0] must be unset!
}

void USART_Write(USART_Handle_t *pUSARTHandle, uint8_t *source, uint8_t size) {
    uint8_t m0 = (pUSARTHandle->pUSARTx->CR1 >> USART_CR1_M0);
    uint8_t m1 = (pUSARTHandle->pUSARTx->CR1 >> USART_CR1_M1);

    uint8_t word_len = m0 | (m1 << 1);


    for (size_t i = 0; i < size; i++) {
        // Wait when we can write new data
        while ((pUSARTHandle->pUSARTx->ISR & (1 << USART_ISR_TXE)) == 0)
            ;

        // Depending on word length, write the data
        switch (word_len) {
            case USART_WORDLEN_8BITS: {
                *((uint8_t*) &pUSARTHandle->pUSARTx->TDR) = *source;
                source++;
                break;
            }
            // XXX sending 8 bits seems very funky
            case USART_WORDLEN_9BITS: {
                uint16_t *pData = (uint16_t*) source;
                // Mask all other bits except for the first 9
                uint16_t masked = *pData & (uint16_t) 0x01FF;

                *((uint16_t*) &pUSARTHandle->pUSARTx->TDR) = masked;

                switch (pUSARTHandle->USART_Config.UASRT_ParityControl) {
                    case USART_PARITY_DISABLE:
                        source += 2;
                        break;
                    case USART_PARITY_EN_EVEN:
                    case USART_PARITY_EN_ODD:
                        source++;
                        break;
                }

                break;
            }
        }

    }

    // Wait for transaction to complete
    while ((pUSARTHandle->pUSARTx->ISR & (1 << USART_ISR_TC)) == 0)
        ;
}

void USART_Read(USART_Handle_t *pUSARTHandle, uint8_t *destination, uint8_t size) {
    uint8_t word_len = (pUSARTHandle->pUSARTx->CR1 >> USART_CR1_M0) & 0x3;

    for (size_t i = 0; i < size; i++) {
        // Wait till we have something to read
        while ((pUSARTHandle->pUSARTx->ISR & (1 << USART_ISR_RXNE)) == 0)
            ;

        // Depending on word length, write the data
        switch (word_len) {
            case USART_WORDLEN_8BITS: {
                switch (pUSARTHandle->USART_Config.UASRT_ParityControl) {
                    case USART_PARITY_DISABLE:
                        *destination = (uint8_t) pUSARTHandle->pUSARTx->RDR;
                        break;
                    case USART_PARITY_EN_EVEN:
                    case USART_PARITY_EN_ODD:
                        *destination = (uint8_t) (pUSARTHandle->pUSARTx->RDR & 0x7F);
                        break;
                }

                destination++;
                break;
            }
            case USART_WORDLEN_9BITS: {
                switch (pUSARTHandle->USART_Config.UASRT_ParityControl) {
                    case USART_PARITY_DISABLE:
                        uint16_t *pDest = (uint16_t*) destination;
                        *pDest =
                                (uint16_t) (pUSARTHandle->pUSARTx->RDR & 0x01FF);

                        break;
                    case USART_PARITY_EN_EVEN:
                    case USART_PARITY_EN_ODD:
                        *destination = (pUSARTHandle->pUSARTx->RDR & (uint8_t) 0x7F);
                        destination++;
                        break;
                }
                break;
            }
        }
    }
}

// XXX IRQ functions should be made global, they are the same for all drivers
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t isEnable) {
    if (isEnable == ENABLE) {
        if (IRQNumber <= 31) {
            // ISER0
            *NVIC_ISER0 |= (1 << IRQNumber);
        } else if (IRQNumber > 32 && IRQNumber < 64) {
            // ISER1
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        } else if (IRQNumber >= 64 && IRQNumber < 96) {
            // ISER2
            *NVIC_ISER2 |= (1 << IRQNumber % 32);
        }
    } else {
        if (IRQNumber <= 31) {
            // ICER0
            *NVIC_ICER0 |= (1 << IRQNumber);
        } else if (IRQNumber > 32 && IRQNumber < 64) {
            // ICER1
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        } else if (IRQNumber >= 64 && IRQNumber < 96) {
            // ICER2
            *NVIC_ICER2 |= (1 << (IRQNumber % 32));
        }
    }
}

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
    uint8_t index = IRQNumber / 4;
    uint8_t offset = IRQNumber % 4;

    // PR lower 4 bits are ignored, thus have to shift prio by 4
    IRQPriority = IRQPriority << NO_PR_BITS;

    *(NVIC_PR_BASE_ADDR + index) |= IRQPriority << (offset * 8);
}

void USART_Enable(USART_RegDef_t *pUSARTx, uint8_t isEnable) {
    pUSARTx->CR1 |= (1 << USART_CR1_UE);
}

__attribute__((weak))  void USART_AppEventCallback(USART_Handle_t *pHandle, uint8_t event) {
    // implement me
}
