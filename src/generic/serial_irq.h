#ifndef __GENERIC_SERIAL_IRQ_H
#define __GENERIC_SERIAL_IRQ_H

#include <stdint.h> // uint32_t

// callback provided by board specific code
void serial_enable_tx_irq(void);
#if CONFIG_STM32_SERIAL_RS485
uint_fast8_t serial_tx_is_active(void);
#endif

// serial_irq.c
void serial_rx_byte(uint_fast8_t data);
int serial_get_tx_byte(uint8_t *pdata);

#endif // serial_irq.h
