/*
 * OmniVision ov490-ov10640 sensor camera wizard 1280x1080@30/UYVY/BT601/8bit
 *
 * Copyright (C) 2016-2017 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

struct ov490_reg {
	u16	reg;
	u8	val;
};

static const struct ov490_reg ov490_regs_wizard[] = {
/* The following registers should match firmware */
{0xfffd, 0x80},
{0xfffe, 0x82},
{0x0071, 0x11},
{0x0075, 0x11},
{0xfffe, 0x29},
{0x6010, 0x01},
{0x4017, 0x00},
{0xfffe, 0x28},
{0x6000, 0x04},
{0x6004, 0x00},
{0x6008, 0x00}, // PCLK polarity - useless due to silicon bug -> use 0x808000bb register
{0xfffe, 0x80},
{0x0091, 0x00},
{0x00bb, 0x1d}, // bit[3]=0 - PCLK polarity workaround
};
