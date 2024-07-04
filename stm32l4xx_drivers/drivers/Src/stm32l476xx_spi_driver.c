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
    pSPIHandle->pSPIx->CR1 |= config->SPI_SclkSpeed << SPI_CR1_BR;

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

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t isEnable) {}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {}
void SPI_IRQHandle(SPI_Handle_t *pHandle) {}

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
