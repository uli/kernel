/*
 * MAXIM R-Car H2 Demo board setup driver
 *
 * Copyright (C) 2016 Renesas Electronics Corporation
 * Copyright (C) 2015 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_gpio.h>

#include "max9286_ov10635.h"

static int maxim_probe(struct i2c_client *client,
		       const struct i2c_device_id *did)
{
	int err;
	int pwen;
	int cam_idx, des_addr, cam_offset;

	if (client->dev.of_node) {
		pwen = of_get_gpio(client->dev.of_node, 0);
		if (pwen > 0) {
			err = gpio_request(pwen, dev_name(&client->dev));
			if (err) {
				dev_err(&client->dev,
				"cannot request PWEN gpio %d: %d\n",
				pwen, err);
			} else {
				gpio_direction_output(pwen, 1);
				mdelay(250);
			}
			/*
			 * Powered MCU IMI cameras need delay between power-on
			 * and R-Car access to avoidi2c bus conflicts since
			 * linux kernel does not support i2c multi-mastering,
			 * IMI MCU is master and R-Car is also master.
			 * The i2c bus conflict results in R-Car i2c IP stall.
			 */
			mdelay(MAXIM_IMI_MCU_DELAY);
		}
	}

	des_addr = client->addr;

	switch (des_addr) {
	case DES0:
		cam_offset = 0;
		client->addr = DES1;			/* MAX9286-CAMx I2C */
		maxim_reg8_write(client, 0x0a, 0x00);
				/* disable reverse control for all cams */
		break;
	case DES1:
		cam_offset = 4;
		break;
	default:
		break;
	}

	for (cam_idx = cam_offset; cam_idx < MAXIM_NUM + cam_offset; cam_idx++) {
		/*
		 * SETUP CAMx (MAX9286/MAX9271/OV10635) I2C
		 */
		dev_info(&client->dev,
		"SETUP CAM%d(MAX9286/MAX9271/OV10635)I2C: 0x%x<->0x%x<->0x%x\n",
		cam_idx, maxim_map[0][cam_idx], maxim_map[1][cam_idx],
		maxim_map[2][cam_idx]);

		/* Reverse channel setup */
		client->addr = des_addr;		/* MAX9286-CAMx I2C */
		maxim_reg8_write(client, 0x0a,
			0xf0 | (1 << (cam_idx - cam_offset)));
				/* enable reverse control only for cam_idx */
		mdelay(2);
			/* wait 2ms after any change of reverse
			*  channel settings
			*/

		client->addr = des_addr;		/* MAX9286-CAMx I2C */
		maxim_reg8_write(client, 0x3f, 0x4f);
			/* enable custom reverse channel & first pulse length */
		maxim_reg8_write(client, 0x34, 0xa2 | MAXIM_I2C_SPEED);
			/* enable artificial ACKs, I2C speed set */
		mdelay(2);
			/* wait 2ms after any change of reverse
			*  channel settings
			*/
		maxim_reg8_write(client, 0x3b, 0x1e);
			/* first pulse length rise time changed
			*  from 300ns to 200ns
			*/
		mdelay(2);
			/* wait 2ms after any change of
			*  reverse channel settings
			*/

		client->addr = 0x40;		/* MAX9271-CAMx I2C */
		i2c_smbus_read_byte(client);	/* ping to wake-up */
		maxim_reg8_write(client, 0x04, 0x43);
				/* wake-up, enable reverse_control/conf_link */
		mdelay(5);	/* wait 5ms for conf_link to establish */
		maxim_reg8_write(client, 0x08, 0x1);
			/* reverse channel receiver high threshold enable */
		mdelay(2);
			/* wait 2ms after any change of reverse
			*  channel settings
			*/

		client->addr = des_addr;	/* MAX9286-CAMx I2C */
		maxim_reg8_write(client, 0x3b, 0x19);
			/* reverse channel increase amplitude 170mV
			*  to compensate high threshold enabled
			*/
		mdelay(2);
			/* wait 2ms after any change of reverse
			*  channel settings
			*/

		/* re-setup for the case of s/w reboot */
		client->addr = 0x40;			/* MAX9271-CAMx I2C */
		maxim_reg8_write(client, 0x04, 0x43);	
			/* wake-up, enable reverse_control/conf_link */
		mdelay(5);	/* wait 5ms for conf_link to establish */
		maxim_reg8_write(client, 0x08, 0x1);
			/* reverse channel receiver high threshold enable */
		mdelay(2);	/* wait 2ms after any change of reverse
				*  channel settings
				*/

		/* Initial setup */
		client->addr = des_addr;	/* MAX9286-CAMx I2C */
		maxim_reg8_write(client, 0x15, 0x13);
			/* disable CSI output, VC is set accordingly
			*  to Link number
			*/
#ifdef YUV_10BIT
		maxim_reg8_write(client, 0x12, 0xf4);
			/* enable CSI-2 Lanes D[0:3], DBL mode, YUV422 10-bit*/
#else
		if (MAXIM_NUM == 1)
			maxim_reg8_write(client, 0x12, 0x33);
			/* enable CSI-2 Lane D0 only, DBL mode, YUV422 8-bit*/
		else
			maxim_reg8_write(client, 0x12, 0xf3);
			/* enable CSI-2 Lanes D[0:3], DBL mode, YUV422  8-bit*/
#endif
#define FSYNC_PERIOD	(1280*800*2)
#if 0
		maxim_reg8_write(client, 0x01, 0x00);
			/* manual: FRAMESYNC set manually
			*  via [0x06:0x08] regs
			*/
#endif
		maxim_reg8_write(client, 0x06, FSYNC_PERIOD & 0xff);
		maxim_reg8_write(client, 0x07, (FSYNC_PERIOD >> 8) & 0xff);
		maxim_reg8_write(client, 0x08, FSYNC_PERIOD >> 16);
		if (MAXIM_NUM == 1) {
			maxim_reg8_write(client, 0x01, 0xc0);
				/* ECU (aka MCU) based FrameSync using
				*  GPI-to-GPO
				*/
			maxim_reg8_write(client, 0x00, 0xe1);
				/* enable GMSL link 0, auto detect link
				*  used for CSI clock source
				*/
			maxim_reg8_write(client, 0x69, 0x0e);
				/* Mask Links 1 2 3, unmask link 0 */
		} else {
			maxim_reg8_write(client, 0x01, 0x02);
				/* automatic: FRAMESYNC taken
				*  from the slowest Link
				*/
			maxim_reg8_write(client, 0x00, 0xef);
				/* enable GMSL links [0:3], auto detect link
				* used for CSI clock source
				*/
		}
		maxim_reg8_write(client, 0x0c, 0x89);
			/* enable HS/VS encoding, use D14/15 for HS/VS,
			* invert VS
			*/

		/* GMSL setup */
		client->addr = 0x40;			/* MAX9271-CAMx I2C */
		maxim_reg8_write(client, 0x0d, 0x22 | MAXIM_I2C_SPEED);
				/* disable artificial ACK, I2C speed set */
		maxim_reg8_write(client, 0x07, 0x94);
			/* RAW/YUV, PCLK rising edge, HS/VS encoding enabled */
#if 0
		maxim_reg8_write(client, 0x02, 0xff);
			/* spread spectrum +-4%, pclk range automatic,
				Gbps automatic  */
#endif
		client->addr = des_addr;	/* MAX9286-CAMx I2C */
		maxim_reg8_write(client, 0x34, 0x22 | MAXIM_I2C_SPEED);
				/* disable artificial ACK, I2C speed set */
		mdelay(2);			/* wait 2ms */

#if 1
		/* I2C translator setup */
		client->addr = 0x40;			/* MAX9271-CAMx I2C */
		maxim_reg8_write(client, 0x09, maxim_map[2][cam_idx] << 1);
							/* OV10635 I2C new */
		maxim_reg8_write(client, 0x0A, 0x30 << 1);
							/* OV10635 I2C */
		maxim_reg8_write(client, 0x0B, BROADCAST << 1);
							/* broadcast I2C */
		maxim_reg8_write(client, 0x0C, maxim_map[1][cam_idx] << 1);
						/* MAX9271-CAMx I2C new */
#else
		client->addr = 0x30;			/* OV10635-CAM0 I2C */
		maxim_reg16_write(client, 0x300C,
			(maxim_map[2][cam_idx] << 1) | 0x1); /* OV10635 new */
#endif

		/* I2C addresses change */
		client->addr = 0x40;			/* MAX9271-CAMx I2C */
		maxim_reg8_write(client, 0x01, maxim_map[0][cam_idx] << 1);
						/* MAX9286-CAM0 I2C new */
		maxim_reg8_write(client, 0x00, maxim_map[1][cam_idx] << 1);
						/* MAX9271-CAM0 I2C new */

#ifdef MAXIM_DUMP
		client->addr = maxim_map[0][cam_idx];
						/* MAX9286-CAMx I2C new */
		maxim_max927x_dump_regs(client);
		client->addr = maxim_map[1][cam_idx];
						/* MAX9271-CAMx I2C new */
		maxim_max927x_dump_regs(client);
		client->addr = maxim_map[2][cam_idx];
						/* OV10635-CAMx I2C */
		maxim_ov10635_dump_regs(client);
#endif

		/* make sure that the conf_link enabled -
		*  needed for reset/reboot, due to I2C runtime changeing
		*/
		client->addr = maxim_map[1][cam_idx];
						/* MAX9271-CAMx I2C new */
		maxim_reg8_write(client, 0x04, 0x43);
				/* wake-up, enable reverse_control/conf_link */
		mdelay(5);	/* wait 5ms for conf_link to establish */
	}

	/* Reverse channel setup */
	client->addr = des_addr;		/* MAX9286-CAMx I2C */
	maxim_reg8_write(client, 0x1b, 0x0f);
				/* enable equalizer for all links */
	if (MAXIM_NUM == 1)
		maxim_reg8_write(client, 0x0a, 0xf1);
				/* enable reverse control only for link0 */
	else
		maxim_reg8_write(client, 0x0a, 0xff);
				/* enable reverse control for all cams */
	mdelay(2);	/* wait 2ms after any change of reverse
			* channel settings
			*/

	/* NOTE: release des_addr to be used in sensor subdevice */
	client->addr = des_addr - 0x40;

	return 0;
}

static int maxim_remove(struct i2c_client *client)
{
	return 0;
}

static const struct of_device_id maxim_dt_ids[] = {
	{ .compatible = "maxim,max9286-ov10635-setup" },
	{},
};
MODULE_DEVICE_TABLE(of, maxim_dt_ids);

static const struct i2c_device_id maxim_id[] = {
	{ "maxim_setup", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, maxim_id);

static struct i2c_driver maxim_i2c_driver = {
	.driver	= {
		.name		= "maxim_setup",
		.of_match_table	= of_match_ptr(maxim_dt_ids),
	},
	.probe		= maxim_probe,
	.remove		= maxim_remove,
	.id_table	= maxim_id,
};

module_i2c_driver(maxim_i2c_driver);

MODULE_DESCRIPTION("Setup driver for 4 SoC Cameras MAX9286<->MAX9271<->OV10635");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
