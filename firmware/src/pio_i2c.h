/**
 * Part of a library to run I2C master from a PIO state machine.
 * (use with pio_i2c.c and pio_i2c.pio)
 *
 * Originally from
 * https://github.com/raspberrypi/pico-examples/tree/master/pio/i2c
 */

/**
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _PIO_I2C_H
#define _PIO_I2C_H

#include "pio_i2c.pio.h"

// ----------------------------------------------------------------------------
// Low-level functions

void pio_i2c_start(PIO pio, uint sm);
void pio_i2c_stop(PIO pio, uint sm);
void pio_i2c_restart_stop(PIO pio, uint sm);

bool pio_i2c_check_error(PIO pio, uint sm);
void pio_i2c_resume_after_error(PIO pio, uint sm);

static void pio_i2c_put16_nocheck(PIO pio, uint sm, uint16_t data);
static void pio_i2c_put16_or_err(PIO pio, uint sm, uint16_t data);
uint8_t pio_i2c_get(PIO pio, uint sm);

// ----------------------------------------------------------------------------
// Transaction-level functions

uint pio_i2c_write_blocking(PIO pio, uint sm, uint8_t addr, uint8_t *txbuf, uint len, bool nostop);
uint pio_i2c_write_blocking_4(PIO pio, uint sm0, uint sm1, uint sm2, uint sm3, uint8_t addr, uint8_t *txbuf, uint len, bool nostop);
uint pio_i2c_read_blocking(PIO pio, uint sm, uint8_t addr, uint8_t *rxbuf, uint len);
uint pio_i2c_read_blocking_4(PIO pio, uint sm0, uint sm1, uint sm2, uint sm3, uint8_t addr, uint8_t *rxbuf0,  uint8_t *rxbuf1, uint8_t *rxbuf2, uint8_t *rxbuf3, uint len);

#endif
