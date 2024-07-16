#include "stm32l4xx_i2c_driver.h"

void I2C_PCLK(I2C_RegDef_t *pI2Cx, uint8_t isEnable) {
    if (pI2Cx == I2C1) {
        I2C1_PCLK_EN();
    } else if (pI2Cx == I2C2) {
        I2C2_PCLK_EN();
    } else if (pI2Cx == I2C3) {
        I2C3_PCLK_EN();
    }
}

// XXX Note: MSI 4 Mhz as SYSCLK is the reset value
void I2C_Init(I2C_Handle_t *pI2CHandle) {
    // XXX We need 16 Mhz for this to all to work for now
    // XXX It is a side effect, should be commented as such and seperate function created
    RCC->CR |= (1 << RCC_CR_MSIRGSEL);

    // Set 16 Mhz
    RCC->CR &= ~(0xF << RCC_CR_MSIRANGE);
    RCC->CR |= (RCC_CR_MSIRANGE_16M << RCC_CR_MSIRANGE);

    I2C_Config_t config = pI2CHandle->I2C_Config;

    // These values are from DS timing examples
    uint8_t sda_del = 0;
    uint8_t scl_del = 0;

    // XXX We could check RCC->SW for clock source.
    // XXX Afterward process MSI possibilities ans HSI16. And propably PLL as well.
    float tick_period = (float)1 / 16000000UL;
    float i2c_period = 0;

    // Set prescaler to DIV2
    pI2CHandle->pI2Cx->TIMINGR &= ~(0x7 << I2C_TIMINGR_PRESC);
    pI2CHandle->pI2Cx->TIMINGR |= (1 << I2C_TIMINGR_PRESC);

    // Must set SCLH and SCLL bits in TIMINGR for master clock config
    if (config.I2C_SCLSpeed == I2C_SCL_SPEED_SM) {
        sda_del = 0x2;
        scl_del = 0x4;
        i2c_period = (float) 1 / 100000UL;
    } else if (config.I2C_SCLSpeed == I2C_SCL_SPEED_FM2K) {
        sda_del = 0x2;
        scl_del = 0x3;
        i2c_period = (float) 1 / 200000UL;
    } else if (config.I2C_SCLSpeed == I2C_SCL_SPEED_FM4K) {
        sda_del = 0x1;
        scl_del = 0x3;
        i2c_period = (float) 1 / 400000UL;
    } else {
        // TODO do a crash
    }

    // Calculate clock high/low time
    float half_period = i2c_period / 2;
    uint8_t duty_cycle = (uint8_t) half_period / tick_period;

    pI2CHandle->pI2Cx->TIMINGR &= ~(0xFF << I2C_TIMINGR_SCLL);
    pI2CHandle->pI2Cx->TIMINGR &= ~(0xFF << I2C_TIMINGR_SCLH);

    pI2CHandle->pI2Cx->TIMINGR |= (duty_cycle << I2C_TIMINGR_SCLL);
    pI2CHandle->pI2Cx->TIMINGR |= (duty_cycle << I2C_TIMINGR_SCLH);

    // Clear delays
    pI2CHandle->pI2Cx->TIMINGR &= ~(0x7 << I2C_TIMINGR_SDADEL);
    pI2CHandle->pI2Cx->TIMINGR &= ~(0x7 << I2C_TIMINGR_SCLDEL);

    // Setdelays
    pI2CHandle->pI2Cx->TIMINGR |= sda_del << I2C_TIMINGR_SDADEL;
    pI2CHandle->pI2Cx->TIMINGR |= scl_del << I2C_TIMINGR_SCLDEL;

    // Assuming non-zero is a proper address, doing the slave way
    if (config.I2C_DeviceAddress != 0x0) {
        pI2CHandle->pI2Cx->OAR1 &= ~0x3FF;

        pI2CHandle->pI2Cx->OAR1 |= (config.I2C_DeviceAddress << 1);
        pI2CHandle->pI2Cx->OAR1 |= (1 << I2C_OA1_EN);
    } else {
        pI2CHandle->pI2Cx->OAR1 &= !(1 << I2C_OA1_EN);
    }
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx) {
    if (pI2Cx == I2C1) {
        I2C1_REG_RESET();
    } else if (pI2Cx == I2C2) {
        I2C2_REG_RESET();
    } else if (pI2Cx == I2C3) {
        I2C3_REG_RESET();
    }
}

void I2C_MasterWrite(I2C_Handle_t *pI2CHandle, uint8_t *source, uint8_t size, uint8_t slave_addr, I2C_Options options) {
    // Set to 7 bit addressing
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ADD10);

    // Set byte count to send
    pI2CHandle->pI2Cx->CR2 &= ~(0x7F << I2C_CR2_NBYTES);
    pI2CHandle->pI2Cx->CR2 |= (size << I2C_CR2_NBYTES);

    if (options.repeated_start != 1) {
        // Stop condition when X bytes are sent
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_AUTOEND);
    } else {
        pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_AUTOEND);
    }

    // Master requests a write
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_RD_WRN);

    // Configure slave address, assume 7 bit
    pI2CHandle->pI2Cx->CR2 &= ~(0x3FF << I2C_CR2_SADD);
    pI2CHandle->pI2Cx->CR2 |= ((slave_addr << 1) << I2C_CR2_SADD);

    // Generate start condition
    pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_START);

    // Wait for address to be sent
    while (((pI2CHandle->pI2Cx->CR2 >> I2C_CR2_START) & 0x1) == 1)
        ;

    while (size > 0) {
        // We will get here either from address or data fail
        if (((pI2CHandle->pI2Cx->ISR >> I2C_ISR_NACKF) & 0x1) == 1) {
            while (1) {
                // Just hang out here
                // XXX Do housekeeping and return false prolly
            }
        }

        // Wait for buffer to be empty
        while (((pI2CHandle->pI2Cx->ISR >> I2C_ISR_TXIS) & 0x1) == 0)
            ;

        *((volatile uint8_t*) &pI2CHandle->pI2Cx->TXDR) = *source;
        source++;
        size--;
    }

    if (options.repeated_start != 1) {
        // Wait for stop to occur
        while (((pI2CHandle->pI2Cx->ISR >> I2C_ISR_STOPF) & 0x1) == 0)
            ;
    } else {
        // Wait for N bytes to be transferred
        while (((pI2CHandle->pI2Cx->ISR >> I2C_ISR_TC) & 0x1) == 0)
            ;
    }
}

void I2C_MasterRead(I2C_Handle_t *pI2CHandle, uint8_t *destination, uint8_t size, uint8_t slave_addr, I2C_Options options) {
    // Must have cleared before address phase
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_AUTOEND);

    // Set to 7 bit addressing
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ADD10);

    // Set byte count to read
    pI2CHandle->pI2Cx->CR2 &= ~(0x7F << I2C_CR2_NBYTES);
    pI2CHandle->pI2Cx->CR2 |= (size << I2C_CR2_NBYTES);

    // Configure slave address, assume 7 bit
    pI2CHandle->pI2Cx->CR2 &= ~(0x3FF << I2C_CR2_SADD);
    pI2CHandle->pI2Cx->CR2 |= ((slave_addr << 1) << I2C_CR2_SADD);

    // Master requests a read
    pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_RD_WRN);

    // Generate start condition
    pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_START);

    // Wait for address to be sent
    while (((pI2CHandle->pI2Cx->CR2 >> I2C_CR2_START) & 0x1) == 1)
        ;

    if (options.repeated_start != 1) {
        // Stop condition when X bytes are read
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_AUTOEND);
    }

    while (size > 0) {
        // Wait for received data to be copied into the readable register
        while (((pI2CHandle->pI2Cx->ISR >> I2C_ISR_RXNE) & 0x1) == 0)
            ;

        *destination = (uint8_t) pI2CHandle->pI2Cx->RXDR;

        size--;
        destination++;
    }

    if (options.repeated_start != 1) {
        // Wait for stop to occur
        while (((pI2CHandle->pI2Cx->ISR >> I2C_ISR_STOPF) & 0x1) == 0)
            ;
    } else {
        // Wait for N bytes to be transferred
        while (((pI2CHandle->pI2Cx->ISR >> I2C_ISR_TC) & 0x1) == 0)
            ;
    }
}

void I2C_Enable(I2C_RegDef_t *pI2Cx, uint8_t isEnable) {
    if (isEnable) {
        pI2Cx->CR1 |= 1 << I2C_CR1_PE;
    } else {
        pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
    }
}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t isEnable) {
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
    uint8_t index = IRQNumber / 4;
    uint8_t offset = IRQNumber % 4;

    // PR lower 4 bits are ignored, thus have to shift prio by 4
    IRQPriority = IRQPriority << NO_PR_BITS;

    *(NVIC_PR_BASE_ADDR + index) |= IRQPriority << (offset * 8);
}