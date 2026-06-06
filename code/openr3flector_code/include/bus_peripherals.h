/*
 * This file is part of OpenR3flector
 *
 * Copyright (C) 2026 vantalane <mete@kestech.net>
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef _BUS_PERIPHS_H
#define _BUS_PERIPHS_H

/* (I2C) REGISTER MAP OF IS602B I2C to SPI bridge*/

enum is602b_func_id
{
	SPI_CONF = 0xF0,
	CLEAR_INT = 0xF1,
	IDLE_MODE = 0xF2,
	GPIO_WRITE = 0xF4,
	GPIO_READ = 0xF5,
	GPIO_ENABLE = 0xF6,
	GPIO_CONF = 0xF7,

};

/*For slave select defining*/
enum is602b_selects
{
	SS0 = (1 << 0),
	SS1 = (1 << 1),
	SS2 = (1 << 2),
	SS3 = (1 << 3)

};

/* (I2C) REGISTER MAP OF LNBH29 supply IC.*/
/*TODO*/

/*(I2C) REGISTER MAP OF STV0903 demodulator.*/
/*TODO*/

/* (SPI) REGISTER MAP OF MXIC flash */
/*TODO*/

#endif