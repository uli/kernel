/*
 * MAXIM max9286-max9271 GMSL driver
 *
 * Copyright (C) 2015-2017 Cogent Embedded, Inc.
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
#include <linux/videodev2.h>
#include <linux/notifier.h>

#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-of.h>
#include <media/v4l2-subdev.h>

#include "max9286_max9271.h"

#define MAXIM_I2C_I2C_SPEED_837KHZ	(0x7 << 2) /* 837kbps */
#define MAXIM_I2C_I2C_SPEED_533KHZ	(0x6 << 2) /* 533kbps */
#define MAXIM_I2C_I2C_SPEED_339KHZ	(0x5 << 2) /* 339 kbps */
#define MAXIM_I2C_I2C_SPEED_173KHZ	(0x4 << 2) /* 174kbps */
#define MAXIM_I2C_I2C_SPEED_105KHZ	(0x3 << 2) /* 105 kbps */
#define MAXIM_I2C_I2C_SPEED_085KHZ	(0x2 << 2) /* 84.7 kbps */
#define MAXIM_I2C_I2C_SPEED_028KHZ	(0x1 << 2) /* 28.3 kbps */
#define MAXIM_I2C_I2C_SPEED		MAXIM_I2C_I2C_SPEED_339KHZ

struct max9286_max9271_priv {
	struct v4l2_subdev	sd[4];
	struct device_node	*sd_of_node[4];
	int			des_addr;
	int			des_quirk_addr; /* second MAX9286 on the same I2C bus */
	int			links;
	int			links_mask;
	int			lanes;
	int			csi_rate;
	const char		*fsync_mode;
	int			fsync_period;
	char			pclk_rising_edge;
	int			gpio_resetb;
	int			active_low_resetb;
	int			timeout;
	atomic_t		use_count;
	struct i2c_client	*client;
	int			max9271_addr_map[4];
};

static void max9286_max9271_preinit(struct i2c_client *client, int addr)
{
	client->addr = addr;			/* MAX9286-CAMx I2C */
	reg8_write(client, 0x0a, 0x00);		/* disable reverse control for all cams */
	reg8_write(client, 0x00, 0x00);		/* disable all GMSL links [0:3] */
	usleep_range(2000, 2500);		/* wait 2ms after any change of reverse channel settings */
}

static int max9286_max9271_reverse_channel_setup(struct i2c_client *client, int idx)
{
	struct max9286_max9271_priv *priv = i2c_get_clientdata(client);
	u8 val = 0;
	int timeout = priv->timeout;
	int mask = priv->links_mask | BIT(idx);
	int ret = 0;

	/* Reverse channel enable */
	client->addr = priv->des_addr;				/* MAX9286-CAMx I2C */
	reg8_write(client, 0x3f, 0x4f);				/* enable custom reverse channel & first pulse length */
	reg8_write(client, 0x34, 0xa2 | MAXIM_I2C_I2C_SPEED);	/* enable artificial ACKs, I2C speed set */
	usleep_range(2000, 2500);				/* wait 2ms after any change of reverse channel settings */
	reg8_write(client, 0x00, 0xe0 | mask);			/* enable GMSL link for CAMx */
	reg8_write(client, 0x0a, (mask << 4) | mask);		/* enable reverse control for CAMx */
	reg8_write(client, 0x69, mask ^ 0x0f);			/* unmask link for CAMx */
	reg8_write(client, 0x1b, mask);				/* enable equalizer for CAMx */
	usleep_range(2000, 2500);				/* wait 2ms after any change of reverse channel settings */

	for (;;) {
		client->addr = priv->des_addr;			/* MAX9286-CAMx I2C */
		reg8_write(client, 0x3b, 0x1e);			/* first pulse length rise time changed from 300ns to 200ns, amplitude 100mV */
		usleep_range(2000, 2500);			/* wait 2ms after any change of reverse channel settings */

		client->addr = 0x40;				/* MAX9271-CAMx I2C */
		i2c_smbus_read_byte(client);			/* ping to wake-up */
		reg8_write(client, 0x04, 0x43);			/* wake-up, enable reverse_control/conf_link */
		reg8_write(client, 0x08, 0x1);			/* reverse channel receiver high threshold enable */
		usleep_range(2000, 2500);			/* wait 2ms after any change of reverse channel settings */

		client->addr = priv->des_addr;			/* MAX9286-CAMx I2C */
		reg8_write(client, 0x3b, 0x19);			/* reverse channel increase amplitude 170mV to compensate high threshold enabled */
		usleep_range(2000, 2500);			/* wait 2ms after any change of reverse channel settings */

		client->addr = 0x40;				/* MAX9271-CAMx I2C */
		i2c_smbus_read_byte(client);			/* ping to wake-up */
		reg8_write(client, 0x04, 0x43);			/* wake-up, enable reverse_control/conf_link */
		reg8_write(client, 0x08, 0x1);			/* reverse channel receiver high threshold enable */
		usleep_range(2000, 2500);			/* wait 2ms after any change of reverse channel settings */

		client->addr = 0x40;				/* MAX9271-CAMx I2C */
		reg8_read(client, 0x1e, &val);			/* read max9271 ID */
		if (val == MAX9271_ID || --timeout == 0)
			break;

		/* Check if already initialized (after reboot/reset ?) */
		client->addr = priv->max9271_addr_map[idx];	/* MAX9271-CAMx I2C */
		reg8_read(client, 0x1e, &val);			/* read max9271 ID */
		if (val == MAX9271_ID) {
			ret = -EADDRINUSE;
			break;
		}
	}

	if (!timeout) {
		ret = -ETIMEDOUT;
		mask &= ~BIT(idx);
		client->addr = priv->des_addr;			/* MAX9286-CAMx I2C */
		reg8_write(client, 0x00, 0xe0 | mask);		/* disable GMSL link for CAMx */
	}

	if ((mask & BIT(idx)) && (hweight8(mask) < (idx + 1)))
		dev_warn(&client->dev, "non-contiguous links are not allowed, "
				       "please fix link#%ld\n", ffz(mask));

	priv->links_mask = mask;

	dev_info(&client->dev, "link%d MAX9271 %sat 0x%x %s\n", idx,
			       ret == -EADDRINUSE ? "already " : "", priv->max9271_addr_map[idx],
			       ret == -ETIMEDOUT ? "not found: timeout GMSL link establish" : "");

	return ret;
}

static void max9286_max9271_initial_setup(struct i2c_client *client)
{
	struct max9286_max9271_priv *priv = i2c_get_clientdata(client);

	/* Initial setup */
	client->addr = priv->des_addr;				/* MAX9286-CAMx I2C */
	reg8_write(client, 0x15, 0x13);				/* disable CSI output, VC is set accordingly to Link number */
	switch (priv->lanes) {
	case 1:
		reg8_write(client, 0x12, 0x33);			/* enable CSI-2 Lane D0, DBL mode, YUV422 8-bit*/
		break;
	case 2:
		reg8_write(client, 0x12, 0x73);			/* enable CSI-2 Lanes D0,D1, DBL mode, YUV422 8-bit*/
		break;
	case 3:
		reg8_write(client, 0x12, 0xd3);			/* enable CSI-2 Lanes D0-D2, DBL mode, YUV422 8-bit*/
		break;
	case 4:
		reg8_write(client, 0x12, 0xf3);			/* enable CSI-2 Lanes D0-D3, DBL mode, YUV422 8-bit*/
		break;
	default:
		dev_err(&client->dev, "CSI2 lanes number is invalid (%d)\n", priv->lanes);
	}

	if (strcmp(priv->fsync_mode, "manual") == 0) {
		reg8_write(client, 0x06, priv->fsync_period & 0xff);
		reg8_write(client, 0x07, (priv->fsync_period >> 8) & 0xff);
		reg8_write(client, 0x08, priv->fsync_period >> 16);
		reg8_write(client, 0x01, 0x00);			/* manual: FRAMESYNC set manually via [0x06:0x08] regs */
	} else if (strcmp(priv->fsync_mode, "automatic") == 0) {
		reg8_write(client, 0x01, 0x02);			/* automatic: FRAMESYNC taken from the slowest Link */
	} else if (strcmp(priv->fsync_mode, "semi-automatic") == 0) {
		reg8_write(client, 0x01, 0x01);			/* semi-automatic: FRAMESYNC taken from the slowest Link */
	} else if (strcmp(priv->fsync_mode, "external") == 0) {
		reg8_write(client, 0x01, 0xc0);			/* ECU (aka MCU) based FrameSync using GPI-to-GPO */
	}

	reg8_write(client, 0x63, 0);				/* disable overlap window */
	reg8_write(client, 0x64, 0);
	reg8_write(client, 0x0c, 0x89);				/* enable HS/VS encoding, use D14/15 for HS/VS, invert VS */
}

static void max9286_max9271_gmsl_link_setup(struct i2c_client *client, int idx)
{
	struct max9286_max9271_priv *priv = i2c_get_clientdata(client);

	/* GMSL setup */
	client->addr = 0x40;					/* MAX9271-CAMx I2C */
	reg8_write(client, 0x0d, 0x22 | MAXIM_I2C_I2C_SPEED);	/* disable artificial ACK, I2C speed set */
	reg8_write(client, 0x07, 0x84 | (priv->pclk_rising_edge ? 0 : 0x10)); /* RAW/YUV, PCLK edge, HS/VS encoding enabled */
	usleep_range(2000, 2500);				/* wait 2ms */
	reg8_write(client, 0x02, 0xff);				/* spread spectrum +-4%, pclk range automatic, Gbps automatic  */
	usleep_range(2000, 2500);				/* wait 2ms */

	client->addr = priv->des_addr;				/* MAX9286-CAMx I2C */
	reg8_write(client, 0x34, 0x22 | MAXIM_I2C_I2C_SPEED);	/* disable artificial ACK, I2C speed set */
	usleep_range(2000, 2500);				/* wait 2ms */

	/* I2C translator setup */
	client->addr = 0x40;					/* MAX9271-CAMx I2C */
//	reg8_write(client, 0x09, maxim_map[2][idx] << 1);	/* SENSOR I2C translated - must be set by sensor driver */
//	reg8_write(client, 0x0A, 0x30 << 1);			/* SENSOR I2C native - must be set by sensor driver */
	reg8_write(client, 0x0B, BROADCAST << 1);		/* broadcast I2C */
	reg8_write(client, 0x0C, priv->max9271_addr_map[idx] << 1); /* MAX9271-CAMx I2C new */
	/* I2C addresse change */
	reg8_write(client, 0x01, priv->des_addr << 1);		/* MAX9286 I2C */
	reg8_write(client, 0x00, priv->max9271_addr_map[idx] << 1); /* MAX9271-CAM0 I2C new */
	usleep_range(2000, 2500);				/* wait 2ms */
	/* put MAX9271 in configuration link state  */
	client->addr = priv->max9271_addr_map[idx];		/* MAX9271-CAMx I2C new */
	reg8_write(client, 0x04, 0x43);				/* enable reverse_control/conf_link */
	usleep_range(2000, 2500);				/* wait 2ms */
#ifdef MAXIM_DUMP
	client->addr = priv->des_addr;				/* MAX9286-CAMx I2C */
	maxim_max927x_dump_regs(client);
	client->addr = priv->max9271_addr_map[idx];		/* MAX9271-CAMx I2C new */
	maxim_max927x_dump_regs(client);
#endif
	if (priv->gpio_resetb >= 1 && priv->gpio_resetb <= 5) {
		/* get out from sensor reset */
		client->addr = priv->max9271_addr_map[idx];	/* MAX9271-CAMx I2C new */
		reg8_write(client, 0x0f, (0xfe & ~BIT(priv->gpio_resetb)) |
				 (priv->active_low_resetb ? BIT(priv->gpio_resetb) : 0)); /* set GPIOn value to un-reset */
		reg8_write(client, 0x0e, 0x42 | BIT(priv->gpio_resetb)); /* set GPIOn direction output */
		usleep_range(2000, 2500);			/* wait 2ms */
	}
}

static int max9286_max9271_initialize(struct i2c_client *client)
{
	struct max9286_max9271_priv *priv = i2c_get_clientdata(client);
	int idx, ret;

	dev_info(&client->dev, "LINKs=%d, LANES=%d, FSYNC mode=%s, FSYNC period=%d, PCLK edge=%s\n",
			       priv->links, priv->lanes, priv->fsync_mode, priv->fsync_period,
			       priv->pclk_rising_edge ? "rising" : "falling");

	if (priv->des_quirk_addr)
		max9286_max9271_preinit(client, priv->des_quirk_addr);

	max9286_max9271_preinit(client, priv->des_addr);
	max9286_max9271_initial_setup(client);

	for (idx = 0; idx < priv->links; idx++) {
		ret = max9286_max9271_reverse_channel_setup(client, idx);
		if (ret)
			continue;
		max9286_max9271_gmsl_link_setup(client, idx);
	}

	client->addr = priv->des_addr;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int max9286_max9271_g_register(struct v4l2_subdev *sd,
				      struct v4l2_dbg_register *reg)
{
	struct max9286_max9271_priv *priv = v4l2_get_subdevdata(sd);
	struct i2c_client *client = priv->client;
	int ret;
	u8 val = 0;

	ret = reg8_read(client, (u8)reg->reg, &val);
	if (ret < 0)
		return ret;

	reg->val = val;
	reg->size = sizeof(u8);

	return 0;
}

static int max9286_max9271_s_register(struct v4l2_subdev *sd,
				      const struct v4l2_dbg_register *reg)
{
	struct max9286_max9271_priv *priv = v4l2_get_subdevdata(sd);
	struct i2c_client *client = priv->client;

	return reg8_write(client, (u8)reg->reg, (u8)reg->val);
}
#endif

static int max9286_max9271_s_power(struct v4l2_subdev *sd, int on)
{
	struct max9286_max9271_priv *priv = v4l2_get_subdevdata(sd);
	struct i2c_client *client = priv->client;

	if (on) {
		if (!atomic_read(&priv->use_count))
			reg8_write(client, 0x15, 0x9b);		/* enable CSI output, VC is set accordingly to Link number, BIT7 magic must be set */
		atomic_inc(&priv->use_count);
	} else {
		atomic_dec(&priv->use_count);
		if (!atomic_read(&priv->use_count))
			reg8_write(client, 0x15, 0x13);		/* disable CSI output, VC is set accordingly to Link number */
	}

	return 0;
}

static int max9286_max9271_registered_async(struct v4l2_subdev *sd)
{
	struct max9286_max9271_priv *priv = v4l2_get_subdevdata(sd);
	struct i2c_client *client = priv->client;
	int idx, tmp_addr;

	/* switch to GMSL serial_link for streaming video */
	tmp_addr = client->addr;
	idx = sd->grp_id;
	client->addr = priv->max9271_addr_map[idx];		/* MAX9271-CAMx */
	reg8_write(client, 0x04, 0x83);				/* enable reverse_control/serial_link */
	usleep_range(2000, 2500);				/* wait 2ms after changing reverse_control */
	client->addr = tmp_addr;

	return 0;
}

static struct v4l2_subdev_core_ops max9286_max9271_subdev_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register		= max9286_max9271_g_register,
	.s_register		= max9286_max9271_s_register,
#endif
	.s_power		= max9286_max9271_s_power,
};

static struct v4l2_subdev_ops max9286_max9271_subdev_ops = {
	.core	= &max9286_max9271_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops max9286_max9271_internal_ops = {
	.registered = max9286_max9271_registered_async,
};

static int max9286_max9271_parse_dt(struct i2c_client *client)
{
	struct max9286_max9271_priv *priv = i2c_get_clientdata(client);
	struct device_node *np = client->dev.of_node;
	struct device_node *endpoint = NULL;
	int err, pwen, i;
	int sensor_delay;
	char fsync_mode_default[20] = "manual"; /* manual, automatic, semi-automatic, external */
	u8 val = 0;

	if (of_property_read_u32(np, "maxim,links", &priv->links))
		priv->links = 4;

	if (of_property_read_u32(np, "maxim,lanes", &priv->lanes))
		priv->lanes = 4;

	pwen = of_get_gpio(np, 0);
	if (pwen > 0) {
		err = gpio_request_one(pwen, GPIOF_OUT_INIT_HIGH, dev_name(&client->dev));
		if (err)
			dev_err(&client->dev, "cannot request PWEN gpio %d: %d\n", pwen, err);
		else
			mdelay(250);
	}

	reg8_read(client, 0x1e, &val);				/* read max9286 ID */
	printk("==maxid 0x%02x\n", val);
	if (val != MAX9286_ID)
		return -ENODEV;

	if (of_property_read_u32(np, "maxim,resetb-gpio", &priv->gpio_resetb)) {
		priv->gpio_resetb = -1;
	} else {
		if (of_property_read_bool(np, "maxim,resetb-active-high"))
			priv->active_low_resetb = false;
		else
			priv->active_low_resetb = true;
	}

	if (!of_property_read_u32(np, "maxim,sensor_delay", &sensor_delay))
		mdelay(sensor_delay);

	if (of_property_read_string(np, "maxim,fsync-mode", &priv->fsync_mode))
		priv->fsync_mode = fsync_mode_default;

	if (of_property_read_u32(np, "maxim,fsync-period", &priv->fsync_period))
		priv->fsync_period = 3200000;			/* 96MHz/30fps */
	priv->pclk_rising_edge = true;
	if (of_property_read_bool(np, "maxim,pclk-falling-edge"))
		priv->pclk_rising_edge = false;
	if (of_property_read_u32(np, "maxim,timeout", &priv->timeout))
		priv->timeout = 100;
	if (of_property_read_u32(np, "maxim,i2c-quirk", &priv->des_quirk_addr))
		priv->des_quirk_addr = 0;

	for (i = 0; i < priv->links; i++) {
		endpoint = of_graph_get_next_endpoint(np, endpoint);
		if (!endpoint)
			break;

		of_node_put(endpoint);

		if (of_property_read_u32(endpoint, "max9271-addr", &priv->max9271_addr_map[i])) {
			dev_err(&client->dev, "max9271-addr not set\n");
			return -EINVAL;
		}

		priv->sd_of_node[i] = endpoint;
	}

	return 0;
}

static void max9286_max9271_setup_remote_endpoint(struct i2c_client *client)
{
	struct max9286_max9271_priv *priv = i2c_get_clientdata(client);
	struct device_node *np = client->dev.of_node;
	struct device_node *endpoint = NULL, *rendpoint = NULL;
	int i;
	struct property *csi_rate_prop, *dvp_order_prop;

	for (i = 0; ; i++) {
		endpoint = of_graph_get_next_endpoint(np, endpoint);
		if (!endpoint)
			break;

		of_node_put(endpoint);

		rendpoint = of_parse_phandle(endpoint, "remote-endpoint", 0);
		if (!rendpoint)
			continue;

		csi_rate_prop = of_find_property(endpoint, "csi-rate", NULL);
		if (csi_rate_prop) {
			/* CSI2_RATE = PCLK*sizeof(YUV8)*links/lanes */
			priv->csi_rate = cpu_to_be32(100 * 8 * hweight8(priv->links_mask) / priv->lanes);
			csi_rate_prop->value = &priv->csi_rate;
			of_update_property(rendpoint, csi_rate_prop);
		}

		dvp_order_prop = of_find_property(endpoint, "dvp-order", NULL);
		if (dvp_order_prop)
			of_update_property(rendpoint, dvp_order_prop);
	}
}

static int max9286_max9271_probe(struct i2c_client *client,
				 const struct i2c_device_id *did)
{
	struct max9286_max9271_priv *priv;
	int err, i;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);
	priv->des_addr = client->addr;
	priv->client = client;
	atomic_set(&priv->use_count, 0);

	printk("==maxparse\n");
	err = max9286_max9271_parse_dt(client);
	if (err)
		goto out;

	printk("==maxinit @ 0x%02x (%s)\n", client->addr, client->adapter->name);
	err = max9286_max9271_initialize(client);
	if (err < 0)
		goto out;

	max9286_max9271_setup_remote_endpoint(client);

	for (i = 0; i < 4; i++) {
		v4l2_subdev_init(&priv->sd[i], &max9286_max9271_subdev_ops);
		priv->sd[i].internal_ops = &max9286_max9271_internal_ops;
		priv->sd[i].owner = client->dev.driver->owner;
		priv->sd[i].dev = &client->dev;
		priv->sd[i].grp_id = i;
		v4l2_set_subdevdata(&priv->sd[i], priv);
		priv->sd[i].of_node = priv->sd_of_node[i];

		snprintf(priv->sd[i].name, V4L2_SUBDEV_NAME_SIZE, "%s.%d %d-%04x",
			 client->dev.driver->name, i, i2c_adapter_id(client->adapter),
			 client->addr);

		err = v4l2_async_register_subdev(&priv->sd[i]);
		if (err < 0)
			goto out;
	}

out:
	return err;
}

static int max9286_max9271_remove(struct i2c_client *client)
{
	struct max9286_max9271_priv *priv = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < 4; i++) {
		v4l2_async_unregister_subdev(&priv->sd[i]);
		v4l2_device_unregister_subdev(&priv->sd[i]);
	}

	return 0;
}

static const struct of_device_id max9286_max9271_dt_ids[] = {
	{ .compatible = "maxim,max9286-max9271" },
	{},
};
MODULE_DEVICE_TABLE(of, max9286_max9271_dt_ids);

static const struct i2c_device_id max9286_max9271_id[] = {
	{ "max9286_max9271", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max9286_max9271_id);

static struct i2c_driver max9286_max9271_i2c_driver = {
	.driver	= {
		.name		= "max9286_max9271",
		.of_match_table	= of_match_ptr(max9286_max9271_dt_ids),
	},
	.probe		= max9286_max9271_probe,
	.remove		= max9286_max9271_remove,
	.id_table	= max9286_max9271_id,
};

module_i2c_driver(max9286_max9271_i2c_driver);

MODULE_DESCRIPTION("GMSL driver for MAX9286-MAX9271");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
