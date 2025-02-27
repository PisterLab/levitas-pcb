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

// Get data from FIFO.
// Although the state machine hardware returns 16 bits,
// only 8 of them are meaningful: these are the bits
// from the I2C bus. The others should be zero.
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


// Send an I2C write command to the given PIO state machine
// (according to protocol defined by its pio_i2c.pio program),
// using the I2C address `addr`.
//
// Specifically, send `len` bytes contained in the array `txbuf`.
// If nostop == false, and the command with a stop signal as normal.
// If nostop == true, don't send a stop signal and instead prepare
// for another I2C command to be immediately sent (by a subsequent
// function call) thus becoming an I2C restart.
//
// Note this function exits about 1.5ish I2C clock periods before the
// state machine finishes sending the end of the I2C stop signal.
int pio_i2c_write_blocking(PIO pio, uint sm, uint8_t addr, uint8_t *txbuf, uint len, bool nostop) {

    // First, recall the PIO state machine program works as follows.
    // Send 16-bit commands to the state machine, which either contain
    // information on an 8-bit byte to read or write from the bus, or
    // a more instantaneous command (e.g., bring SDA high). Communication
    // to the state machine occurs via TX and RX FIFO buffers, each of which
    // contains four 16-bit commands. New commands are put into the TX
    // buffer, and the state machine outputs data read from the bus to the RX
    // buffer. If the RX FIFO buffer is full, the state machine stalls (!).
    //
    // Also note that, at any point, the state machine can throw an error by
    // raising its interrupt signal. This is done when an I2C message is
    // improperly acknowledged. To deal with this, any of the below calls
    // will simply exit early if an error condition exists, then we reset
    // the state machine before this function exits.

    // When this function begins, we expect both TX and RX FIFO buffers to
    // be empty, but in case a previous function messed up we'll fix that.
    // (i.e., this is a bit of minor error correction)
    if(!pio_sm_is_rx_fifo_empty(pio, sm)){ pio_i2c_get(pio, sm); }

    // Next, send commands to execute an I2C start signal. These
    // should not result in any returned bytes in the RX FIFO buffer.
    pio_i2c_start(pio, sm);

    // Track the number of things added and removed from PIO state machine
    // FIFO buffers via the subset of 16-bit commands that read or write to
    // the I2C bus. Note reads and writes both require sending a byte and
    // receiving a byte (with the sent and received bytes being mere
    // placeholders, respectively). We only want to write for now.
    uint bytes_sent = 0;
    uint bytes_received = 0;

    // Begin the I2C write by sending the device address.
    // We format the 16-bit command for the PIO state machine as follows:
    // (expect ack, 8 bits (address + 0 for write), don't send master ack)
    pio_i2c_put16_or_err(pio, sm, (addr << 2) | 1u);
    // This should have triggered the PIO state machine byte write
    // function. As a result, we should expect to receive a corresponding
    // byte later (though its contents are useless).
    bytes_sent += 1;

    // We can now send every other byte of data we're interested in.
    // The PIO state machine runs much slower than the main microcontroller,
    // so do this in a fast loop. Remember to take bytes out of the RX FIFO
    // when they are ready so it doesn't become full and stall the state
    // machine. We're done when we've received the last expected byte.
    // (expected number of bytes = `len` plus one for the address byte)
    while (bytes_sent < len+1 || bytes_received < len+1) {

        // don't get stuck in infinite loop if state machine raises
        // error if an I2C ack fails; we'll reset the sensor, exit this
        // function, then try again later
        if(pio_i2c_check_error(pio, sm)){ break; }

        // write data byte
        if (!pio_sm_is_tx_fifo_full(pio, sm) && bytes_sent < len+1) {
            // (expect ack (except on last byte), 8 bits, don't send master ack)
            pio_i2c_put16_nocheck(pio, sm, (txbuf[bytes_sent-1] << PIO_I2C_DATA_LSB) | ((bytes_sent == len) << PIO_I2C_FINAL_LSB) | 1u);
            bytes_sent += 1;
        }

        // read dummy byte
        if (!pio_sm_is_rx_fifo_empty(pio, sm) && bytes_received < len+1) {
            pio_i2c_get(pio, sm);
            bytes_received += 1;
        }
    }

    // Note that although we've already read from the bus, the state machine
    // will be waiting about 42 of its cycles (each 1/32 of the I2C period,
    // i.e., 13us for a 100kHz I2C clock or 3us for a 400kHz clock) to finish
    // the I2C data including the nak/ack signal. This function can finish
    // in the meantime.

    // Finally, send an I2C stop condition in the same way we sent the
    // start condition (unless a restart is requested, in which case we
    // don't send a stop but toggle the bus so a subsequent start works
    // a little better.
    if(nostop){
        pio_i2c_restart_stop(pio, sm);
    }else{
        pio_i2c_stop(pio, sm);
    }

    // If state machine hit an error, restart it.
    int err = 0;
    if (pio_i2c_check_error(pio, sm)) {
        err = -1;
        pio_i2c_resume_after_error(pio, sm);
        pio_i2c_stop(pio, sm); // send a stop signal no matter `nostop`
    }
    return err;
}

int pio_i2c_write_blocking_4(PIO pio, uint sm0, uint sm1, uint sm2, uint sm3, uint8_t addr, uint8_t *txbuf, uint len, bool nostop) {
    int err = 0;
    // emit start signal
    pio_i2c_start(pio, sm0);
    pio_i2c_start(pio, sm1);
    pio_i2c_start(pio, sm2);
    pio_i2c_start(pio, sm3);
    // write data, expect ack, 8 bits (address + 0 for writing), don't send master ack
    pio_i2c_put16_or_err(pio, sm0, (addr << 2) | 1u);
    pio_i2c_put16_or_err(pio, sm1, (addr << 2) | 1u);
    pio_i2c_put16_or_err(pio, sm2, (addr << 2) | 1u);
    pio_i2c_put16_or_err(pio, sm3, (addr << 2) | 1u);

    uint tx0 = 0;
    uint rx0 = len + 1; // expect len bytes plus the address byte
    uint tx1 = 0;
    uint rx1 = len + 1; // expect len bytes plus the address byte
    uint tx2 = 0;
    uint rx2 = len + 1; // expect len bytes plus the address byte
    uint tx3 = 0;
    uint rx3 = len + 1; // expect len bytes plus the address byte
    while (tx0 < len && rx0 > 0 && tx1 < len && rx1 > 0 && tx2 < len && rx2 > 0 && tx3 < len && rx3 > 0) {
        if(pio_i2c_check_error(pio, sm0) || pio_i2c_check_error(pio, sm1) || pio_i2c_check_error(pio, sm2) || pio_i2c_check_error(pio, sm3)){
            break; // error!
        }
        // write data bytes
        if (!pio_sm_is_tx_fifo_full(pio, sm0) && tx0 < len) {
            // expect ack (except on last byte?), 8 bits, don't send master ack
            pio_i2c_put16_nocheck(pio, sm0, (txbuf[tx0] << PIO_I2C_DATA_LSB) | ((tx0 == len-1) << PIO_I2C_FINAL_LSB) | 1u);
            tx0 += 1;
        }
        // write data bytes
        if (!pio_sm_is_tx_fifo_full(pio, sm1) && tx0 < len) {
            // expect ack (except on last byte?), 8 bits, don't send master ack
            pio_i2c_put16_nocheck(pio, sm1, (txbuf[tx1] << PIO_I2C_DATA_LSB) | ((tx1 == len-1) << PIO_I2C_FINAL_LSB) | 1u);
            tx1 += 1;
        }
        // write data bytes
        if (!pio_sm_is_tx_fifo_full(pio, sm2) && tx2 < len) {
            // expect ack (except on last byte?), 8 bits, don't send master ack
            pio_i2c_put16_nocheck(pio, sm2, (txbuf[tx2] << PIO_I2C_DATA_LSB) | ((tx2 == len-1) << PIO_I2C_FINAL_LSB) | 1u);
            tx2 += 1;
        }
        // write data bytes
        if (!pio_sm_is_tx_fifo_full(pio, sm3) && tx3 < len) {
            // expect ack (except on last byte?), 8 bits, don't send master ack
            pio_i2c_put16_nocheck(pio, sm3, (txbuf[tx3] << PIO_I2C_DATA_LSB) | ((tx3 == len-1) << PIO_I2C_FINAL_LSB) | 1u);
            tx3 += 1;
        }

        // read dummy bytes
        if (!pio_sm_is_rx_fifo_empty(pio, sm0)) { (void)pio_i2c_get(pio, sm0); rx0 -= 1; }
        if (!pio_sm_is_rx_fifo_empty(pio, sm1)) { (void)pio_i2c_get(pio, sm1); rx1 -= 1; }
        if (!pio_sm_is_rx_fifo_empty(pio, sm2)) { (void)pio_i2c_get(pio, sm2); rx2 -= 1; }
        if (!pio_sm_is_rx_fifo_empty(pio, sm3)) { (void)pio_i2c_get(pio, sm3); rx3 -= 1; }
    }
    if(nostop){
        pio_i2c_restart_stop(pio, sm0);
        pio_i2c_restart_stop(pio, sm1);
        pio_i2c_restart_stop(pio, sm2);
        pio_i2c_restart_stop(pio, sm3);
    }else{
        // emit stop signal
        pio_i2c_stop(pio, sm0);
        pio_i2c_stop(pio, sm1);
        pio_i2c_stop(pio, sm2);
        pio_i2c_stop(pio, sm3);
    }
    // wait until state machine is done sending everything
    //pio_i2c_wait_idle(pio, sm);
    while (!pio_sm_is_rx_fifo_empty(pio, sm0)) // still useful for some reason? why?
        (void)pio_i2c_get(pio, sm0);
    while (!pio_sm_is_rx_fifo_empty(pio, sm1)) // still useful for some reason? why?
        (void)pio_i2c_get(pio, sm1);
    while (!pio_sm_is_rx_fifo_empty(pio, sm2)) // still useful for some reason? why?
        (void)pio_i2c_get(pio, sm2);
    while (!pio_sm_is_rx_fifo_empty(pio, sm3)) // still useful for some reason? why?
        (void)pio_i2c_get(pio, sm3);
    // if hit an error, restart
    if (pio_i2c_check_error(pio, sm0)) {
        err = -1;
        pio_i2c_resume_after_error(pio, sm0);
        pio_i2c_stop(pio, sm0);
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

    //uint tx = 0;
    //uint rx = len + 1; // expect len bytes plus the address byte
    while (tx_remain || len) { // in very fast loop...
        if(pio_i2c_check_error(pio, sm)){ break; } // error!
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

