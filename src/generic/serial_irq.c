// Generic interrupt based serial uart helper code
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memmove
#include "autoconf.h" // CONFIG_SERIAL_BAUD
#include "board/io.h" // readb
#include "board/irq.h" // irq_save
#include "board/misc.h" // console_sendf
#include "board/pgm.h" // READP
#include "command.h" // DECL_CONSTANT
#include "sched.h" // sched_wake_tasks
#include "serial_irq.h" // serial_enable_tx_irq

#define RX_BUFFER_SIZE 192

static uint8_t receive_buf[RX_BUFFER_SIZE], receive_pos;
#if CONFIG_STM32_SERIAL_RS485
// Hold asynchronous responses until the host grants a half-duplex bus turn.
#define TX_BUFFER_SIZE 255
#define TX_COMMAND_RESERVE 96
#define TX_ASYNC_LIMIT (TX_BUFFER_SIZE - TX_COMMAND_RESERVE)
#else
#define TX_BUFFER_SIZE 96
#endif
static uint8_t transmit_buf[TX_BUFFER_SIZE], transmit_pos, transmit_max;
#if CONFIG_STM32_SERIAL_RS485
// Reports generated while the final ACK is already on the wire belong to the
// next bus turn. Keeping them separate avoids both a collision and lost
// one-shot events (for example, an endstop or trsync state transition).
static uint8_t deferred_buf[TX_BUFFER_SIZE], deferred_max;
#endif

DECL_CONSTANT("SERIAL_BAUD", CONFIG_SERIAL_BAUD);
#if CONFIG_STM32_SERIAL_RS485
// Limit the host to one outstanding packet on a half-duplex link.
DECL_CONSTANT("RECEIVE_WINDOW", MESSAGE_MAX);
DECL_CONSTANT("SERIAL_HALF_DUPLEX", 1);
#else
DECL_CONSTANT("RECEIVE_WINDOW", RX_BUFFER_SIZE);
#endif

// Rx interrupt - store read data
void
serial_rx_byte(uint_fast8_t data)
{
    if (data == MESSAGE_SYNC)
        sched_wake_tasks();
    if (receive_pos >= sizeof(receive_buf))
        // Serial overflow - ignore it as crc error will force retransmit
        return;
    receive_buf[receive_pos++] = data;
}

// Tx interrupt - get next byte to transmit
int
serial_get_tx_byte(uint8_t *pdata)
{
    if (transmit_pos >= transmit_max)
        return -1;
    *pdata = transmit_buf[transmit_pos++];
    return 0;
}

// Remove from the receive buffer the given number of bytes
static void
console_pop_input(uint_fast8_t len)
{
    uint_fast8_t copied = 0;
    for (;;) {
        uint_fast8_t rpos = readb(&receive_pos);
        uint_fast8_t needcopy = rpos - len;
        if (needcopy) {
            memmove(&receive_buf[copied], &receive_buf[copied + len]
                    , needcopy - copied);
            copied = needcopy;
            sched_wake_tasks();
        }
        irqstatus_t flag = irq_save();
        if (rpos != readb(&receive_pos)) {
            // Raced with irq handler - retry
            irq_restore(flag);
            continue;
        }
        receive_pos = needcopy;
        irq_restore(flag);
        break;
    }
}

#if CONFIG_STM32_SERIAL_RS485
static uint_fast8_t
console_has_output(void)
{
    return readb(&transmit_pos) < readb(&transmit_max);
}

// Remove bytes already sent in the preceding turn. Call with IRQs disabled.
static void
console_compact_output(void)
{
    uint_fast8_t tpos = readb(&transmit_pos);
    uint_fast8_t tmax = readb(&transmit_max);
    if (tpos >= tmax) {
        writeb(&transmit_max, 0);
        writeb(&transmit_pos, 0);
    } else if (tpos) {
        tmax -= tpos;
        memmove(transmit_buf, &transmit_buf[tpos], tmax);
        writeb(&transmit_pos, 0);
        writeb(&transmit_max, tmax);
    }
}

// Leave room for the response to the next host command and its final ACK.
// If the host has been disconnected for a long time, queued telemetry is
// stale and may otherwise fill the bounded transmit buffer.
static void
console_prepare_output(void)
{
    irqstatus_t flag = irq_save();
    console_compact_output();
    uint_fast8_t tmax = readb(&transmit_max);
    if (tmax > TX_ASYNC_LIMIT) {
        writeb(&transmit_max, 0);
        writeb(&transmit_pos, 0);
        tmax = 0;
    }
    uint_fast8_t dmax = readb(&deferred_max);
    uint_fast8_t dpos = 0;
    while (dpos < dmax) {
        uint_fast8_t msglen = deferred_buf[dpos];
        if (msglen < MESSAGE_MIN || dpos + msglen > dmax) {
            // A partial frame can only result from a prior buffer overflow.
            dpos = dmax;
            break;
        }
        if (tmax + msglen > TX_ASYNC_LIMIT)
            break;
        memcpy(&transmit_buf[tmax], &deferred_buf[dpos], msglen);
        tmax += msglen;
        dpos += msglen;
    }
    writeb(&transmit_max, tmax);
    dmax -= dpos;
    if (dmax)
        memmove(deferred_buf, &deferred_buf[dpos], dmax);
    writeb(&deferred_max, dmax);
    irq_restore(flag);
}
#endif

// Process any incoming commands
void
console_task(void)
{
    uint_fast8_t rpos = readb(&receive_pos), pop_count;
#if CONFIG_STM32_SERIAL_RS485
    // command_find_block() may queue a NAK. Keep interrupts disabled until a
    // negative result has started TX so no asynchronous report can be placed
    // after that final NAK.
    irqstatus_t find_flag = irq_save();
    console_compact_output();
    uint_fast8_t pre_find_max = readb(&transmit_max);
#endif
    int_fast8_t ret = command_find_block(receive_buf, rpos, &pop_count);
#if CONFIG_STM32_SERIAL_RS485
    if (ret <= 0) {
        if (ret < 0) {
            if (CONFIG_HAVE_BOOTLOADER_REQUEST && pop_count == 32
                && !memcmp(receive_buf,
                           " \x1c Request Serial Bootloader!! ~", 32))
                bootloader_request();
            console_pop_input(pop_count);
            // Transmit only if command_find_block() actually appended a NAK.
            // A leading retransmit sync byte and later invalid fragments must
            // not grant queued asynchronous data a response turn by accident.
            if (readb(&transmit_max) > pre_find_max)
                serial_enable_tx_irq();
        }
        irq_restore(find_flag);
        return;
    }
    irq_restore(find_flag);
    console_prepare_output();
#endif
    if (ret > 0)
        command_dispatch(receive_buf, pop_count);
    if (ret) {
        if (CONFIG_HAVE_BOOTLOADER_REQUEST && ret < 0 && pop_count == 32
            && !memcmp(receive_buf, " \x1c Request Serial Bootloader!! ~", 32))
            bootloader_request();
        console_pop_input(pop_count);
#if CONFIG_STM32_SERIAL_RS485
        // A shutdown report is normally asynchronous and could have occurred
        // during the preceding transmit turn. Repeat it on the next host turn
        // so a dropped best-effort response can never hide an MCU shutdown.
        if (ret > 0 && sched_is_shutdown())
            sched_report_shutdown();
#endif
#if CONFIG_STM32_SERIAL_RS485
        // The host is the RS-485 bus master. Send queued asynchronous
        // responses, command responses, and the final empty ACK as one turn.
        // Queue the ACK and start hardware DE atomically so it is guaranteed
        // to remain the final frame of this turn.
        irqstatus_t ack_flag = irq_save();
        command_send_ack();
        if (console_has_output())
            serial_enable_tx_irq();
        irq_restore(ack_flag);
#else
        if (ret > 0)
            command_send_ack();
#endif
    }
}
DECL_TASK(console_task);

// Encode and transmit a "response" message
void
console_sendf(const struct command_encoder *ce, va_list args)
{
#if CONFIG_STM32_SERIAL_RS485
    if (serial_tx_is_active()) {
        // The current turn's final ACK has already been queued. Defer this
        // response instead of placing data after the ACK or dropping a
        // potentially one-shot event.
        uint_fast8_t dmax = readb(&deferred_max);
        uint_fast8_t max_size = READP(ce->max_size);
        if (dmax + max_size > sizeof(deferred_buf))
            return;
        uint8_t *dbuf = &deferred_buf[dmax];
        uint_fast8_t msglen = command_encode_and_frame(dbuf, ce, args);
        writeb(&deferred_max, dmax + msglen);
        return;
    }
#endif

    // Verify space for message
    uint_fast8_t tpos = readb(&transmit_pos), tmax = readb(&transmit_max);
    if (tpos >= tmax) {
        tpos = tmax = 0;
        writeb(&transmit_max, 0);
        writeb(&transmit_pos, 0);
    }
    uint_fast8_t max_size = READP(ce->max_size);
#if CONFIG_STM32_SERIAL_RS485
    if (max_size > MESSAGE_MIN
        && tmax + max_size > sizeof(transmit_buf) - MESSAGE_MIN)
        // Always preserve enough room for the final ACK. A missing best-effort
        // response can be queried again; a missing ACK stalls the whole bus.
        return;
#endif
    if (tmax + max_size > sizeof(transmit_buf)) {
        if (tmax + max_size - tpos > sizeof(transmit_buf))
            // Not enough space for message
            return;
        // Disable TX irq and move buffer
        writeb(&transmit_max, 0);
        tpos = readb(&transmit_pos);
        tmax -= tpos;
        memmove(&transmit_buf[0], &transmit_buf[tpos], tmax);
        writeb(&transmit_pos, 0);
        writeb(&transmit_max, tmax);
#if !CONFIG_STM32_SERIAL_RS485
        serial_enable_tx_irq();
#endif
    }

    // Generate message
    uint8_t *buf = &transmit_buf[tmax];
    uint_fast8_t msglen = command_encode_and_frame(buf, ce, args);

    // Start message transmit
    writeb(&transmit_max, tmax + msglen);
#if !CONFIG_STM32_SERIAL_RS485
    serial_enable_tx_irq();
#endif
}
