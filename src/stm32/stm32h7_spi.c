// SPI functions on STM32H7
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/io.h" // readb, writeb
#include "command.h" // shutdown
#include "gpio.h" // spi_setup
#include "internal.h" // gpio_peripheral
#include "sched.h" // sched_shutdown

struct spi_info {
    SPI_TypeDef *spi;
    uint8_t miso_pin, mosi_pin, sck_pin;
    uint8_t miso_function, mosi_function, sck_function;
};

// Define the SPI buses with the requested pins
DECL_ENUMERATION("spi_bus", "spi1", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi1", "PG9,PD7,PG11");

DECL_ENUMERATION("spi_bus", "spi2", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi2", "PB14,PB15,PD3");

//TODO: remove, just for debug on toolhead mcu (miso, mosi, sclk)
DECL_ENUMERATION("spi_bus", "spi3", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi3", "PC11,PB2,PC10");

DECL_ENUMERATION("spi_bus", "spi4", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi4", "PE13,PE14,PE12");

DECL_ENUMERATION("spi_bus", "spi5", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi5", "PF8,PF9,PF7");

DECL_ENUMERATION("spi_bus", "spi6", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi6", "PG12,PB5,PC12");

static const struct spi_info spi_bus[] = {
    { SPI1, GPIO('G', 9), GPIO('D', 7), GPIO('G', 11), GPIO_FUNCTION(5), GPIO_FUNCTION(5), GPIO_FUNCTION(5) }, // spi1
    { SPI2, GPIO('B', 14), GPIO('B', 15), GPIO('D', 3), GPIO_FUNCTION(5), GPIO_FUNCTION(5), GPIO_FUNCTION(5) }, // spi2
    { SPI3, GPIO('C', 11), GPIO('B', 2), GPIO('C', 10), GPIO_FUNCTION(6), GPIO_FUNCTION(7), GPIO_FUNCTION(6) }, // spi3
    { SPI4, GPIO('E', 13), GPIO('E', 14), GPIO('E', 12), GPIO_FUNCTION(5), GPIO_FUNCTION(5), GPIO_FUNCTION(5) }, // spi4
    { SPI5, GPIO('F', 8), GPIO('F', 9), GPIO('F', 7), GPIO_FUNCTION(5), GPIO_FUNCTION(5), GPIO_FUNCTION(5) }, // spi5
    { SPI6, GPIO('G', 12), GPIO('B', 5), GPIO('C', 12), GPIO_FUNCTION(5), GPIO_FUNCTION(8), GPIO_FUNCTION(5) }, // spi6
};

struct spi_config
spi_setup(uint32_t bus, uint8_t mode, uint32_t rate)
{
    if (bus >= ARRAY_SIZE(spi_bus))
        shutdown("Invalid spi bus");

    // Enable SPI
    SPI_TypeDef *spi = spi_bus[bus].spi;
    if (!is_enabled_pclock((uint32_t)spi)) {
        enable_pclock((uint32_t)spi);
        gpio_peripheral(spi_bus[bus].miso_pin, spi_bus[bus].miso_function, 1);
        gpio_peripheral(spi_bus[bus].mosi_pin, spi_bus[bus].mosi_function, 0);
        gpio_peripheral(spi_bus[bus].sck_pin, spi_bus[bus].sck_function, 0);
    }

    // Calculate CR1 register
    uint32_t pclk = get_pclock_frequency((uint32_t)spi);
    uint32_t div = 0;
    while ((pclk >> (div + 1)) > rate && div < 7)
        div++;

    spi->CFG1 |= (div << SPI_CFG1_MBR_Pos) | (7 << SPI_CFG1_DSIZE_Pos);
    CLEAR_BIT(spi->CFG1, SPI_CFG1_CRCSIZE);
    spi->CFG2 |= ((mode << SPI_CFG2_CPHA_Pos) | SPI_CFG2_MASTER | SPI_CFG2_SSM
                   | SPI_CFG2_AFCNTR | SPI_CFG2_SSOE);
    spi->CR1 |= SPI_CR1_SSI;

    return (struct spi_config){ .spi = spi, .div = div, .mode = mode };
}

void
spi_prepare(struct spi_config config)
{
    uint32_t div = config.div;
    uint32_t mode = config.mode;
    SPI_TypeDef *spi = config.spi;
    // Reload frequency
    spi->CFG1 = (spi->CFG1 & ~SPI_CFG1_MBR_Msk);
    spi->CFG1 |= (div << SPI_CFG1_MBR_Pos);
    // Reload mode
    spi->CFG2 = (spi->CFG2 & ~SPI_CFG2_CPHA_Msk);
    spi->CFG2 |= (mode << SPI_CFG2_CPHA_Pos);
}

// A stalled SPI peripheral must shut down with a report rather than spin
// with interrupts disabled until the independent watchdog hard-resets the
// chip (a mid-session MCU reset desynchronizes the host serial link).
#define SPI_WAIT_LOOPS 1000000

static void
spi_wait(SPI_TypeDef *spi, uint32_t mask)
{
    uint32_t loops = SPI_WAIT_LOOPS;
    while ((spi->SR & mask) == 0) {
        if (!--loops)
            shutdown("Timeout waiting on stm32h7 spi");
    }
}

void
spi_transfer(struct spi_config config, uint8_t receive_data,
             uint8_t len, uint8_t *data)
{
    uint8_t rdata = 0;
    SPI_TypeDef *spi = config.spi;

    MODIFY_REG(spi->CR2, SPI_CR2_TSIZE, len);
    // Enable SPI and start transfer, these MUST be set in this sequence
    SET_BIT(spi->CR1, SPI_CR1_SPE);
    SET_BIT(spi->CR1, SPI_CR1_CSTART);

    while (len--) {
        writeb((void *)&spi->TXDR, *data);
        spi_wait(spi, SPI_SR_RXWNE | SPI_SR_RXPLVL);
        rdata = readb((void *)&spi->RXDR);

        if (receive_data) {
            *data = rdata;
        }
        data++;
    }

    spi_wait(spi, SPI_SR_EOT);

    // Clear flags and disable SPI
    SET_BIT(spi->IFCR, 0xFFFFFFFF);
    CLEAR_BIT(spi->CR1, SPI_CR1_SPE);
}
