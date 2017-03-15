/*
 * TI ti964-ti913 FPDLinkIII driver
 *
 * Copyright (C) 2017 Cogent Embedded, Inc.
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

#include "ti964_ti913.h"

struct ti964_ti913_priv {
	struct v4l2_subdev	sd[4];
	struct device_node	*sd_of_node[4];
	int			des_addr;
	int			links;
	int			lanes;
	const char		*forwarding_mode;
	atomic_t		use_count;
	struct i2c_client	*client;
	int			ti913_addr_map[4];
};

static void ti964_ti913_initial_setup(struct i2c_client *client)
{
	struct ti964_ti913_priv *priv = i2c_get_clientdata(client);

	/* Initial setup */
	client->addr = priv->des_addr;				/* TI964 I2C */
	reg8_write(client, 0x08, 0x1c);				/* I2C glitch filter depth */
	reg8_write(client, 0x0a, 0x79);				/* I2C high pulse width */
	reg8_write(client, 0x0b, 0x79);				/* I2C low pulse width */
	reg8_write(client, 0x0d, 0xb9);				/* VDDIO 3.3V */
	reg8_write(client, 0x1f, 0x02);				/* CSI rate 800mpbs */

	if (strcmp(priv->forwarding_mode, "round-robin") == 0) {
		reg8_write(client, 0x21, 0x01);			/* Round Robin forwarding enable */
	} else if (strcmp(priv->forwarding_mode, "synchronized") == 0) {
		reg8_write(client, 0x21, 0x44);			/* Basic Syncronized forwarding enable (FrameSync must be enabled!!) */
	}

	reg8_write(client, 0x32, 0x01);				/* Select TX (CSI) port 0 */
	reg8_write(client, 0x33, ((priv->lanes - 1) ^ 0x3) << 4); /* disable CSI output, set CSI lane count, non-continuous CSI mode */
	reg8_write(client, 0x20, 0xf0);				/* disable port forwarding */
#if 1
	/* FrameSync setup for 30FPS: period_counts=1/FPS/40ns=1/30/40e-9=833333 -> FS_TIME=833333 FPS=30.0 */
 #define FS_TIME 833333
	reg8_write(client, 0x1a, FS_TIME >> 16);		/* FrameSync time 24bit */
	reg8_write(client, 0x1b, (FS_TIME >> 8) & 0xff);
	reg8_write(client, 0x1c, FS_TIME & 0xff);
	reg8_write(client, 0x18, 0x43);				/* Enable FrameSync, 50/50 mode, Frame clock from 25MHz */
#else
	/* FrameSync setup for 30FPS: period_counts=1/FPS/12mks=1/30/12e-6=2777 -> HI=2, LO=2775 FPS=30.008 */
	reg8_write(client, 0x19, 2 >> 8);			/* FrameSync high time MSB */
	reg8_write(client, 0x1a, 2 >> 16);			/* FrameSync high time LSB */
	reg8_write(client, 0x1b, 2775 & 0xff);			/* FrameSync low time MSB */
	reg8_write(client, 0x1c, 2775 & 0xff);			/* FrameSync low time LSB */
	reg8_write(client, 0x18, 0x01);				/* Enable FrameSync, HI/LO mode, Frame clock from port0 */
#endif
}

//#define SENSOR_ID 0x30  // ov10635
//#define SENSOR_ID 0x24  // ov490

static void ti964_ti913_fpdlink3_setup(struct i2c_client *client, int idx)
{
	struct ti964_ti913_priv *priv = i2c_get_clientdata(client);

	/* FPDLinkIII setup */
	client->addr = priv->des_addr;				/* TI964 I2C */
	reg8_write(client, 0x4c, (idx << 4) | (1 << idx));	/* Select RX port number */
	usleep_range(2000, 2500);				/* wait 2ms */
	reg8_write(client, 0x58, 0x58);				/* Back channel: pass-through/backchannel/CRC enable, Freq=2.5Mbps */
	reg8_write(client, 0x5c, priv->ti913_addr_map[idx] << 1); /* TI913 I2C addr */
//	reg8_write(client, 0x5d, SENSOR_ID << 1);		/* SENSOR I2C native - must be set by sensor driver */
//	reg8_write(client, 0x65, (0x60 + idx) << 1);		/* SENSOR I2C translated - must be set by sensor driver */
	reg8_write(client, 0x6d, 0x7f);				/* Coax, RAW10 */
	reg8_write(client, 0x70, (idx << 6) | 0x1e);		/* CSI data type: yuv422 8-bit, assign VC */
	reg8_write(client, 0x7c, 0x81);				/* BIT(7) - magic to Use RAW10 as 8-bit mode */
	reg8_write(client, 0x6e, 0x99);				/* Backchannel GPIO0/GPIO1 set high */
}

static int ti964_ti913_initialize(struct i2c_client *client)
{
	struct ti964_ti913_priv *priv = i2c_get_clientdata(client);
	int idx;

	dev_info(&client->dev, "LINKs=%d, LANES=%d, FORWARDING=%s\n",
			       priv->links, priv->lanes, priv->forwarding_mode);

	ti964_ti913_initial_setup(client);

	for (idx = 0; idx < priv->links; idx++)
		ti964_ti913_fpdlink3_setup(client, idx);

	client->addr = priv->des_addr;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ti964_ti913_g_register(struct v4l2_subdev *sd,
				      struct v4l2_dbg_register *reg)
{
	struct ti964_ti913_priv *priv = v4l2_get_subdevdata(sd);
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

static int ti964_ti913_s_register(struct v4l2_subdev *sd,
				      const struct v4l2_dbg_register *reg)
{
	struct ti964_ti913_priv *priv = v4l2_get_subdevdata(sd);
	struct i2c_client *client = priv->client;

	return reg8_write(client, (u8)reg->reg, (u8)reg->val);
}
#endif

static int ti964_ti913_s_power(struct v4l2_subdev *sd, int on)
{
	struct ti964_ti913_priv *priv = v4l2_get_subdevdata(sd);
	struct i2c_client *client = priv->client;

	if (on) {
		if (!atomic_read(&priv->use_count)) {
			reg8_write(client, 0x20, 0x00);		/* enable port forwarding */
			reg8_write(client, 0x33, ((priv->lanes - 1) ^ 0x3) << 4 | 0x1); /* enable CSI output, set CSI lane count, non-continuous CSI mode */
		}
		atomic_inc(&priv->use_count);
	} else {
		atomic_dec(&priv->use_count);
		if (!atomic_read(&priv->use_count)) {
			reg8_write(client, 0x33, ((priv->lanes - 1) ^ 0x3) << 4); /* disable CSI output, set CSI lane count, non-continuous CSI mode */
			reg8_write(client, 0x20, 0xf0);		/* disable port forwarding */
		}
	}

	return 0;
}

static int ti964_ti913_registered_async(struct v4l2_subdev *sd)
{
	return 0;
}

static struct v4l2_subdev_core_ops ti964_ti913_subdev_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register		= ti964_ti913_g_register,
	.s_register		= ti964_ti913_s_register,
#endif
	.s_power		= ti964_ti913_s_power,
	.registered_async	= ti964_ti913_registered_async,
};

static struct v4l2_subdev_ops ti964_ti913_subdev_ops = {
	.core	= &ti964_ti913_subdev_core_ops,
};

static int ti964_ti913_parse_dt(struct i2c_client *client)
{
	struct ti964_ti913_priv *priv = i2c_get_clientdata(client);
	struct device_node *np = client->dev.of_node;
	struct device_node *endpoint = NULL, *rendpoint = NULL;
	int err, pwen, i;
	int sensor_delay;
	char forwarding_mode_default[20] = "round-robin"; /* round-robin, synchronized */
	struct property *csi_rate_prop, *dvp_order_prop;

	if (of_property_read_u32(np, "ti,links", &priv->links))
		priv->links = 4;

	if (of_property_read_u32(np, "ti,lanes", &priv->lanes))
		priv->lanes = 4;

	pwen = of_get_gpio(np, 0);
	if (pwen > 0) {
		err = gpio_request_one(pwen, GPIOF_OUT_INIT_HIGH, dev_name(&client->dev));
		if (err)
			dev_err(&client->dev, "cannot request PWEN gpio %d: %d\n", pwen, err);
		else
			mdelay(250);
	}

	if (!of_property_read_u32(np, "ti,sensor_delay", &sensor_delay))
		mdelay(sensor_delay);

	err = of_property_read_string(np, "ti,forwarding-mode", &priv->forwarding_mode);
	if (err)
		priv->forwarding_mode = forwarding_mode_default;

	for (i = 0; ; i++) {
		endpoint = of_graph_get_next_endpoint(np, endpoint);
		if (!endpoint)
			break;

		of_node_put(endpoint);

		if (i < priv->links) {
			if (of_property_read_u32(endpoint, "ti913-addr", &priv->ti913_addr_map[i])) {
				dev_err(&client->dev, "ti913-addr not set\n");
				return -EINVAL;
			}
			priv->sd_of_node[i] = endpoint;
		}

		rendpoint = of_parse_phandle(endpoint, "remote-endpoint", 0);
		if (!rendpoint)
			continue;

		csi_rate_prop = of_find_property(endpoint, "csi-rate", NULL);
		if (csi_rate_prop)
			of_update_property(rendpoint, csi_rate_prop);

		dvp_order_prop = of_find_property(endpoint, "dvp-order", NULL);
		if (dvp_order_prop)
			of_update_property(rendpoint, dvp_order_prop);
	}

	return 0;
}

static int ti964_ti913_probe(struct i2c_client *client,
			     const struct i2c_device_id *did)
{
	struct ti964_ti913_priv *priv;
	int err, i;
	u8 val = 0;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);
	priv->des_addr = client->addr;
	priv->client = client;
	atomic_set(&priv->use_count, 0);

	reg8_read(client, 0x00, &val);		/* read TI964 ID */
	if (val != (priv->des_addr << 1))
		return -ENODEV;

	err = ti964_ti913_parse_dt(client);
	if (err)
		goto out;

	err = ti964_ti913_initialize(client);
	if (err < 0)
		goto out;

	for (i = 0; i < 4; i++) {
		v4l2_subdev_init(&priv->sd[i], &ti964_ti913_subdev_ops);
		priv->sd[i].owner = client->dev.driver->owner;
		priv->sd[i].dev = &client->dev;
		priv->sd[i].grp_id = i;
		v4l2_set_subdevdata(&priv->sd[i], priv);
		priv->sd[i].of_node = priv->sd_of_node[i];

		snprintf(priv->sd[i].name, V4L2_SUBDEV_NAME_SIZE, "%s %d-%04x",
			 client->dev.driver->name, i2c_adapter_id(client->adapter),
			 client->addr);

		err = v4l2_async_register_subdev(&priv->sd[i]);
		if (err < 0)
			goto out;
	}

out:
	return err;
}

static int ti964_ti913_remove(struct i2c_client *client)
{
	struct ti964_ti913_priv *priv = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < 4; i++) {
		v4l2_async_unregister_subdev(&priv->sd[i]);
		v4l2_device_unregister_subdev(&priv->sd[i]);
	}

	return 0;
}

static const struct of_device_id ti964_ti913_dt_ids[] = {
	{ .compatible = "ti,ti964-ti913" },
	{},
};
MODULE_DEVICE_TABLE(of, ti964_ti913_dt_ids);

static const struct i2c_device_id ti964_ti913_id[] = {
	{ "ti964_ti913", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ti964_ti913_id);

static struct i2c_driver ti964_ti913_i2c_driver = {
	.driver	= {
		.name		= "ti964_ti913",
		.of_match_table	= of_match_ptr(ti964_ti913_dt_ids),
	},
	.probe		= ti964_ti913_probe,
	.remove		= ti964_ti913_remove,
	.id_table	= ti964_ti913_id,
};

module_i2c_driver(ti964_ti913_i2c_driver);

MODULE_DESCRIPTION("FPDLinkIII driver for TI964-TI913");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
