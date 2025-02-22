/**
 * Part of a library to run I2C master from a PIO state machine.
 * (use with pio_i2c.h and pio_i2c.pio)
 *
 * Originally from (then heavily commented and slightly tweaked)
 * https://github.com/raspberrypi/pico-examples/tree/master/pio/i2c
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pio_i2c.h"

// bit indices of assorted parts of instruction
// (see PIO program)
const int PIO_I2C_ICOUNT_LSB = 10;
const int PIO_I2C_FINAL_LSB  = 9;
const int PIO_I2C_DATA_LSB   = 1;
const int PIO_I2C_NAK_LSB    = 0;

// determine whether state machine interrupt was set
// (will happen if unexpected NAK was observed)
bool pio_i2c_check_error(PIO pio, uint sm) {
    return pio_interrupt_get(pio, sm);
}

// manually restart state machine after NAK-triggered interrupt
void pio_i2c_resume_after_error(PIO pio, uint sm) {
    // empty FIFO buffer
    pio_sm_drain_tx_fifo(pio, sm);
    // restart by manually making state machine execute an unconditional jump
    // to the desired entry point (beginning of wrap loop)
    pio_sm_exec(pio, sm, (pio->sm[sm].execctrl & PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS) >> PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    // clear interrupt
    pio_interrupt_clear(pio, sm);
}

// Put a 16-bin instruction into the PIO state machine FIFO
// (which holds 4 instructions). Nonblocking. Don't check if FIFO is full
// or whether state machine is currently stalled due to I2C ACK error. If
// either is the case, might overwrite data in TX FIFO.
// (basically identical to pio_sm_put() but 16 bits)
static inline void pio_i2c_put16_nocheck(PIO pio, uint sm, uint16_t data) {
    // some versions of GCC dislike this
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif
    *(io_rw_16 *)&pio->txf[sm] = data;
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
}

// Same as pio_i2c_put16_nocheck(), but blocks until TX FIFO has room,
// and also errors out on I2C NAK issues.
void pio_i2c_put16_or_err(PIO pio, uint sm, uint16_t data) {
    // block while state machine FIFO is full
    while (pio_sm_is_tx_fifo_full(pio, sm))
        if (pio_i2c_check_error(pio, sm))
            return;
    if (pio_i2c_check_error(pio, sm))
        return;
    // some versions of GCC dislike this
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif
    *(io_rw_16 *)&pio->txf[sm] = data;
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
}

// get data from FIFO
uint8_t pio_i2c_get(PIO pio, uint sm) {
    return (uint8_t)pio_sm_get(pio, sm);
}

// make state machine emit I2C start signal
void pio_i2c_start(PIO pio, uint sm) {
    pio_i2c_put16_or_err(pio, sm, 1u << PIO_I2C_ICOUNT_LSB);                      // tell state machine to execute the next 1+1=2 commands
    pio_i2c_put16_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD0]); // while clock is high (idling), pull SDA low to signal start
    pio_i2c_put16_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD0]); // also pull clock low to be ready to send data
}

// make state machine emit I2C stop signal
void pio_i2c_stop(PIO pio, uint sm) {
    pio_i2c_put16_or_err(pio, sm, 2u << PIO_I2C_ICOUNT_LSB);                      // tell state machine to execute the next 1+2=3 commands
    pio_i2c_put16_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD0]); // SDA is unknown; pull it low while clock is down (which it should already be)
    pio_i2c_put16_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD0]); // clock high while SDA low
    pio_i2c_put16_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD1]); // SDA high while clock is low to return to idle
};

// replace a stop with this in order to later do a restart
void pio_i2c_restart_stop(PIO pio, uint sm) {
    pio_i2c_put16_or_err(pio, sm, 1u << PIO_I2C_ICOUNT_LSB);
    pio_i2c_put16_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD1]);
    pio_i2c_put16_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD1]);
};

// wait until state machine has nothing left in FIFO or emits IRQ error
static void pio_i2c_wait_idle(PIO pio, uint sm) {
    // clear the TXSTALL bit (which is set when state machine stalls on empty TX FIFO)
    pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm);
    // wait until TXSTALL bit is set again by PIO state machine, or IRQ error
    while (!(pio->fdebug & 1u << (PIO_FDEBUG_TXSTALL_LSB + sm) || pio_i2c_check_error(pio, sm)))
        tight_loop_contents(); // SDK no-op function
}

int pio_i2c_write_blocking(PIO pio, uint sm, uint8_t addr, uint8_t *txbuf, uint len, bool nostop) {
    int err = 0;
    // emit start signal
    pio_i2c_start(pio, sm);
    // write data, expect ack, 8 bits (address + 0 for writing), don't send master ack
    pio_i2c_put16_or_err(pio, sm, (addr << 2) | 1u);
    while (len && !pio_i2c_check_error(pio, sm)) {
        // wait until there's room in the FIFO buffer
        if (!pio_sm_is_tx_fifo_full(pio, sm)) {
            --len;
            // then send data
            // expect ack (except on last byte?), 8 bits, don't send master ack
            pio_i2c_put16_nocheck(pio, sm, (*txbuf++ << PIO_I2C_DATA_LSB) | ((len == 0) << PIO_I2C_FINAL_LSB) | 1u);
        }
        if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            (void)pio_i2c_get(pio, sm);
        }
    }
    if(nostop){
        pio_i2c_restart_stop(pio, sm);
    }else{
        // emit stop signal
        pio_i2c_stop(pio, sm);
    }
    // wait until state machine is done sending everything
    pio_i2c_wait_idle(pio, sm);
    while (!pio_sm_is_rx_fifo_empty(pio, sm))
        (void)pio_i2c_get(pio, sm);
    // if hit an error, restart
    if (pio_i2c_check_error(pio, sm)) {
        err = -1;
        pio_i2c_resume_after_error(pio, sm);
        pio_i2c_stop(pio, sm);
    }
    return err;
}

int pio_i2c_read_blocking(PIO pio, uint sm, uint8_t addr, uint8_t *rxbuf, uint len) {
    int err = 0;
    // emit start signal
    pio_i2c_start(pio, sm);
    // empty data input FIFO
    while (!pio_sm_is_rx_fifo_empty(pio, sm))
        (void)pio_i2c_get(pio, sm);
    // write data, expect ack, 8 bits (address + 1 for reading), don't send master ack
    pio_i2c_put16_or_err(pio, sm, (addr << 2) | 3u);
    // send data
    // use tx_remain variable to track how many bytes to tell state machine to expect
    // use len to track how many bytes we've read out
    uint32_t tx_remain = len;
    bool first = true;
    while ((tx_remain || len) && !pio_i2c_check_error(pio, sm)) { // in very fast loop...
        // when the FIFO buffer has room, send command to tell state machine to read another byte
        if (tx_remain && !pio_sm_is_tx_fifo_full(pio, sm)) {
            --tx_remain;
            // send 0xff (doesn't write to bus b/c pullup), expect NAK on final byte, otherwise have master send ack
            pio_i2c_put16_nocheck(pio, sm, (0xffu << 1) | (tx_remain ? 0 : (1u << PIO_I2C_FINAL_LSB) | (1u << PIO_I2C_NAK_LSB)));
        }
        // when there's stuff available, read it
        if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            if (first) {
                // Ignore returned address byte
                (void)pio_i2c_get(pio, sm);
                first = false;
            }else {
                --len;
                *rxbuf++ = pio_i2c_get(pio, sm);
            }
        }
    }
    // emit stop signal
    pio_i2c_stop(pio, sm);
    // wait until state machine is done sending everything
    pio_i2c_wait_idle(pio, sm);
    // if hit an error, restart
    if (pio_i2c_check_error(pio, sm)) {
        err = -1;
        pio_i2c_resume_after_error(pio, sm);
        pio_i2c_stop(pio, sm);
    }
    return err;
}

