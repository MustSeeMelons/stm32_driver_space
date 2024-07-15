#include "stm32l476xx_spi_driver.h"

void SPI_PCLK(SPI_RegDef_t *pSPIx, uint8_t isEnable) {
    if (isEnable == ENABLE) {
        if (pSPIx == SPI1) {
            SPI1_PCLK_EN();
        } else if (pSPIx == SPI2) {
            SPI2_PCLK_EN();
        } else if (pSPIx == SPI3) {
            SPI3_PCLK_EN();
        }
    } else {
        if (pSPIx == SPI1) {
            SPI1_PCLK_DI();
        } else if (pSPIx == SPI2) {
            SPI2_PCLK_DI();
        } else if (pSPIx == SPI3) {
            SPI3_PCLK_DI();
        }
    }
}

void SPI_Init(SPI_Handle_t *pSPIHandle) {
    SPI_PCLK(pSPIHandle->pSPIx, ENABLE);

    SPI_Config_t *config = &pSPIHandle->SPIConfig;

    // Device mode
    if (config->SPI_DeviceMode == SPI_DEVICE_MODE_MASTER) {
        pSPIHandle->pSPIx->CR1 |= 0x1 << SPI_CR1_MSTR;
    } else {
        pSPIHandle->pSPIx->CR1 &= ~(0x1 << SPI_CR1_MSTR);
    }

    // Bus config
    if (config->SPI_BusConfig == SPI_BUS_CONFIG_FD) {
        // 2-line unidirectional
        pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);

        // Must be cleared in uni 2 line
        pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_RXONLY);

    } else if (config->SPI_BusConfig == SPI_BUS_CONFIG_HD) {
        pSPIHandle->pSPIx->CR1 |= 1 << SPI_CR1_BIDIMODE;
    } else if (config->SPI_BusConfig == SPI_BUS_CONFIG_S_RXONLY) {
        // 2-line unidirectional, tho we use but one line
        pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);

        // Must be cleared in uni 2 line
        pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_RXONLY);
    }

    // Clock speed
    pSPIHandle->pSPIx->CR1 &= ~(0x7 << SPI_CR1_BR);
    pSPIHandle->pSPIx->CR1 |= (config->SPI_SclkSpeed << SPI_CR1_BR);

    // DFF
    pSPIHandle->pSPIx->CR2 &= ~(0xF << SPI_CR2_DS);

    if(config->SPI_DFF == SPI_DFF_8BITS) {
        pSPIHandle->pSPIx->CR2 |= 0x7 << SPI_CR2_DS;

        // FRXTH, RXNE event on 8 >= bits
        pSPIHandle->pSPIx->CR2 |= 1 << SPI_CR2_FRXTH;
    } else {
        pSPIHandle->pSPIx->CR2 |= 0xF << SPI_CR2_DS;

        // FRXTH, RXNE event on 16 >= bits
        pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_FRXTH);
    }

    // CPOL
    if (config->SPI_CPOL == SPI_CPOL_HIGH) {
        pSPIHandle->pSPIx->CR1 |= 1 << SPI_CR1_CPOL;
    } else {
        pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_CPOL);
    }

    // CPHA
    if (config->SPI_CPHA == SPI_CPAH_HIGH) {
        pSPIHandle->pSPIx->CR1 |= 1 << SPI_CR1_CPHA;
    } else {
        pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_CPHA);
    }

    // SSM
    if (config->SPI_SSM == SPI_SSM_EN) {
        pSPIHandle->pSPIx->CR1 |= 1 << SPI_CR1_SSM;

        SPI_SSIConfig(pSPIHandle->pSPIx, ENABLE);
    } else {
        pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_SSM);

        SPI_SSIConfig(pSPIHandle->pSPIx, DISABLE);
    }


}

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
    if (pSPIx == SPI1) {
        SPI1_REG_RESET();
    } else if (pSPIx == SPI2) {
        SPI2_REG_RESET();
    } else if (pSPIx == SPI3) {
        SPI3_REG_RESET();
    }
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagMask) {
    if (pSPIx->SR & flagMask) {
        return FLAG_SET;
    }

    return FLAG_RESET;
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *source, uint32_t size) {
    uint16_t is_16 = (pSPIx->CR2 >> SPI_CR2_DS & 0xF) == 0xF;

    // Alternative: source++
    uint8_t index = 0;

    while (size > 0) {
        // Whait while TX buffer is not empty

        // Alternative: (pSPIx->SR >> SPI_SR_TXE & 0x1) == 0
        while ((pSPIx->SR & (0x1 << SPI_SR_TXE)) == 0)
            ;

        // Place a single or two bytes in the data register
        if (is_16) {
            uint16_t low = *(source + index++);
            uint16_t high = *(source + index++);

            // Alternative *((uint16_t*)source + index++)
            pSPIx->DR = high << 8 | low;
            size -= 2;
        } else {
            *((volatile uint8_t*) &pSPIx->DR) = *(source + index++);
            size--;
        }
    }
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *destination, uint32_t size) {
    uint16_t is_16 = (pSPIx->CR2 >> SPI_CR2_DS & 0xF) == 0xF;

    while (size > 0) {
        // Wait for buffer to fill
        while((pSPIx->SR & (0x01 << SPI_SR_RXNE)) == 0);

        if (is_16) {
            *((uint16_t*) destination) = (uint16_t) pSPIx->DR;
            (uint16_t*) destination++;
            size -= 2;
        } else {
            *destination = (uint8_t) pSPIx->DR;
            destination++;
            size--;
        }
    }
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t isEnable) {
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
    uint8_t index = IRQNumber / 4;
    uint8_t offset = IRQNumber % 4;

    // PR lower 4 bits are ignored, thus have to shift prio by 4
    IRQPriority = IRQPriority << NO_PR_BITS;

    *(NVIC_PR_BASE_ADDR + index) |= IRQPriority << (offset * 8);
}

void SPI_Enable(SPI_RegDef_t *pSPIx, uint8_t isEnable) {
    // SSM == 0, SPE == 1 => NSS LOW when SPE == 1
    // SSM == 0, SPE == 0 => NSS high when SPE 0
    // If SSOE is set to 1

    if (isEnable == ENABLE) {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    } else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t isEnable) {
    if (isEnable == ENABLE) {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    } else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}

// SS output enable
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t isEnable) {
    if (isEnable == ENABLE) {
        // SS output enabled, cannot work in multi master mode
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    } else {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pHandle, uint8_t *source, uint32_t size) {
    if (pHandle->txState != SPI_BUSY_TX) {
        // Saver buffer
        pHandle->pTxBuffer = source;
        // Save length
        pHandle->txLen = size;
        // Mark SPI as busy
        pHandle->txState = SPI_BUSY_TX;
        // Enable TX IT
        pHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
        // Handle transmission ir ISR
    }

    return pHandle->txState;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pHandle, uint8_t *destination, uint32_t size) {
    if (pHandle->rxState != SPI_BUSY_RX) {
        // Saver buffer
        pHandle->pRxBuffer = destination;
        // Save length
        pHandle->rxLen = size;
        // Mark SPI as busy
        pHandle->rxState = SPI_BUSY_RX;
        // Enable TX IT
        pHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
        // Handle transmission ir ISR
    }

    return pHandle->rxState;
}

static void spi_txe_it_handler(SPI_Handle_t *pHandle) {
    uint16_t is_16 = (pHandle->pSPIx->CR2 >> SPI_CR2_DS & 0xF) == 0xF;

    uint8_t *source = pHandle->pTxBuffer;

    // Place a single or two bytes in the data register
    if (is_16) {
        pHandle->pSPIx->DR = *((uint16_t*) source);
        (uint16_t*) source++;
        pHandle->txLen -= 2;
    } else {
        *((volatile uint8_t*) &pHandle->pSPIx->DR) = *source;
        source++;
        pHandle->txLen--;
    }

    pHandle->pTxBuffer = source;

    if (!pHandle->rxLen) {
        SPI_CloseTransmission(pHandle);

        SPI_AppEventCallback(pHandle, SPI_EVENT_TX_COMPLETE);
    }
}

static void spi_rxe_it_handler(SPI_Handle_t *pHandle) {
    uint16_t is_16 = (pHandle->pSPIx->CR2 >> SPI_CR2_DS & 0xF) == 0xF;

    if (is_16) {
        *((uint16_t*) pHandle->pRxBuffer) = (uint16_t) pHandle->pSPIx->DR;
        (uint16_t*) pHandle->pRxBuffer++;
        pHandle->rxState -= 2;
    } else {
        *pHandle->pRxBuffer = (uint8_t) pHandle->pSPIx->DR;
        pHandle->pRxBuffer++;
        pHandle->rxState--;
    }

    if (!pHandle->rxLen) {
        SPI_CloseReception(pHandle);

        SPI_AppEventCallback(pHandle, SPI_EVENT_RX_COMPLETE);
    }
}

static void spi_err_it_handler(SPI_Handle_t *pHandle) {
    if (pHandle->txState != SPI_BUSY_TX) {
        SPI_ClearOVR(pHandle);
    }

    SPI_AppEventCallback(pHandle, SPI_EVENT_OVR_ERR);
}

void SPI_IRQHandle(SPI_Handle_t *pHandle) {
    uint8_t tx_flag = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
    uint8_t tx_it_flag = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

    if (tx_flag && tx_it_flag) {
        spi_txe_it_handler(pHandle);
    }

    uint8_t rx_flag = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
    uint8_t rx_it_flag = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

    if (rx_flag && rx_it_flag) {
        spi_rxe_it_handler(pHandle);
    }

    uint8_t ovr_flag = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
    uint8_t err_it_flag = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

    if (ovr_flag && err_it_flag) {
        spi_err_it_handler(pHandle);
    }
}

void SPI_ClearOVR(SPI_Handle_t *pHandle) {
    uint8_t d;
    d = pHandle->pSPIx->DR;
    d = pHandle->pSPIx->SR;
    (void) d; // it is "used" now
}

void SPI_CloseTransmission(SPI_Handle_t *pHandle) {
    pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    pHandle->pTxBuffer = NULL;
    pHandle->txLen = 0;
    pHandle->txState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pHandle) {
    pHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pHandle->pRxBuffer = NULL;
    pHandle->rxLen = 0;
    pHandle->rxState = SPI_READY;
}

__attribute__((weak)) void SPI_AppEventCallback(SPI_Handle_t *pHandle, uint8_t event) {
    // Override me
}
