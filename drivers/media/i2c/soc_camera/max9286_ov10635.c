/*
 * MAXIM max9286-max9271 with OmniVision ov10635 sensor camera driver
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
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/videodev2.h>

#include <media/soc_camera.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>

#include "max9286_ov10635.h"
#include "max9286_ov10635_wizard_1280x800.h"

#define OV10635_PID			0x300a
#define OV10635_VER			0x300b
#define OV10635_VERSION_REG		0xa635
#define OV10635_VERSION(pid, ver)	(((pid) << 8) | ((ver) & 0xff))

#define OV10635_SENSOR_WIDTH		1312
#define OV10635_SENSOR_HEIGHT		814

#define OV10635_WIDTH			1280
#define OV10635_HEIGHT			800

struct ov10635_priv {
	struct v4l2_subdev		sd;
	struct v4l2_ctrl_handler	hdl;
	struct media_pad		pad;
	int				width;
	int				height;
};

static inline struct ov10635_priv *to_ov10635(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client),
		struct ov10635_priv, sd);
}

static int ov10635_set_regs(struct i2c_client *client,
			    const struct ov10635_reg *regs, int nr_regs)
{
	int i, ret;

	for (i = 0; i < nr_regs; i++) {
		ret = maxim_reg16_write(client, regs[i].reg, regs[i].val);
#if 0 /* Do not stop on write fail .... */
		if (ret)
			return ret;
#endif
	}

	return 0;
}

static int ov10635_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int cam_idx = client->addr - CAM;
	int tmp_addr;

if (!(client->addr == (CAM + 0) || client->addr == (CAM + 4)))
	return 0;

	/* switch to GMSL conf_link to access sensor registers */
	tmp_addr = client->addr;
	client->addr = maxim_map[0][cam_idx];	/* MAX9286-CAMx */
	maxim_reg8_write(client, 0x15, 0x93);
		/* disable CSI output,
		 *	VC is set accordingly to Link number,
		 *	BIT7 magic must be set
		 */
#if 0
	maxim_reg8_write(client, 0x0a, 0xff);
				/* enable reverse_control/conf_link */
#endif
	mdelay(5);
				/* wait 5ms for conf_link to establish */
	client->addr = maxim_map[1][cam_idx];	/* MAX9271-CAMx */
#if 0
	maxim_reg8_write(client, 0x04, 0x43);
				/* enable reverse_control/conf_link */
#endif
	mdelay(5);		/* wait 5ms for conf_link to establish */
	client->addr = tmp_addr;
#if 0
	maxim_reg16_write(client, 0x0100, enable); /* stream on/off */
#endif
	if (enable) {
		/* switch to GMSL serial_link for streaming video */
		tmp_addr = client->addr;
		client->addr = maxim_map[1][cam_idx];	/* MAX9271-CAMx */
		maxim_reg8_write(client, 0x04, 0x83);
				/* enable reverse_control/serial_link */
		mdelay(2);
				/* wait 2ms after changing reverse_control */
		client->addr = maxim_map[0][cam_idx];	/* MAX9286-CAMx */
#if 0
		maxim_reg8_write(client, 0x0a, 0xf0);
			/* disable reverse_control, enable serial_link */
#endif
		maxim_reg8_write(client, 0x15, 0x9b);
			/* enable CSI output,
			 *  VC is set accordingly to Link number,
			 *  BIT7 magic must be set
			 */
		mdelay(5);
			/* wait 2ms after changing reverse_control */
		client->addr = tmp_addr;
	}

	return 0;
}

static int ov10635_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;

	if (format->pad)
		return -EINVAL;

	mf->width	= OV10635_WIDTH;
	mf->height	= OV10635_HEIGHT;
#ifdef YUV_10BIT
	mf->code	= MEDIA_BUS_FMT_YUYV10_2X10;
#else
	mf->code	= MEDIA_BUS_FMT_YUYV8_2X8;
#endif
	mf->colorspace	= V4L2_COLORSPACE_SMPTE170M;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int ov10635_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;

#ifdef YUV_10BIT
	mf->code = MEDIA_BUS_FMT_YUYV10_2X10;
#else
	mf->code = MEDIA_BUS_FMT_YUYV8_2X8;
#endif
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		cfg->try_fmt = *mf;
		return 0;
	}

	return 0;
}

static int ov10635_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

#ifdef YUV_10BIT
	code->code = MEDIA_BUS_FMT_YUYV10_2X10;
#else
	code->code = MEDIA_BUS_FMT_YUYV8_2X8;
#endif

	return 0;
}

static int ov10635_g_mbus_config(struct v4l2_subdev *sd,
				 struct v4l2_mbus_config *cfg)
{
	cfg->flags = V4L2_MBUS_CSI2_1_LANE | V4L2_MBUS_CSI2_CHANNEL_0 |
		     V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	cfg->type = V4L2_MBUS_CSI2;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov10635_g_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 val;

	ret = maxim_reg16_read(client, (u16)reg->reg, &val);
	if (ret < 0)
		return ret;

	reg->val = val;
	reg->size = sizeof(u16);

	return 0;
}

static int ov10635_s_register(struct v4l2_subdev *sd,
			      const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return maxim_reg16_write(client, (u16)reg->reg, (u8)reg->val);
}
#endif

static struct v4l2_subdev_core_ops ov10635_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov10635_g_register,
	.s_register = ov10635_s_register,
#endif
};

static int ov10635_s_ctrl(struct v4l2_ctrl *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
	case V4L2_CID_CONTRAST:
	case V4L2_CID_SATURATION:
	case V4L2_CID_HUE:
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops ov10635_ctrl_ops = {
	.s_ctrl = ov10635_s_ctrl,
};

static struct v4l2_subdev_video_ops ov10635_video_ops = {
	.s_stream	= ov10635_s_stream,
	.g_mbus_config	= ov10635_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops ov10635_subdev_pad_ops = {
	.enum_mbus_code = ov10635_enum_mbus_code,
	.get_fmt	= ov10635_get_fmt,
	.set_fmt	= ov10635_set_fmt,
};

static struct v4l2_subdev_ops ov10635_subdev_ops = {
	.core	= &ov10635_core_ops,
	.video	= &ov10635_video_ops,
	.pad	= &ov10635_subdev_pad_ops,
};

static int ov10635_initialize(struct i2c_client *client)
{
	u8 pid, ver;
	int ret;
	int tmp_addr, cam_idx;

	/* check and show product ID and manufacturer ID */
	ret = maxim_reg16_read(client, OV10635_PID, &pid);
	if (ret)
		return ret;
	ret = maxim_reg16_read(client, OV10635_VER, &ver);
	if (ret)
		return ret;

	if (OV10635_VERSION(pid, ver) != OV10635_VERSION_REG) {
		dev_err(&client->dev, "Product ID error %x:%x\n", pid, ver);
		return -ENODEV;
	}

	dev_info(&client->dev, "ov10635 Product ID %x Manufacturer ID %x\n",
		 pid, ver);

#if !defined(MAXIM_IMI_MCU_POWERED)
#if 0
	/* IMI camera has GPIO1 routed to OV10635 reset pin */
	client->addr = maxim_map[1][cam_idx];	/* OV10635-CAMx I2C new */
	maxim_reg8_write(client, 0x0f, 0xfc);
					/* GPIO1 low, ov10635 in reset */
	mdelay(10);
	maxim_reg8_write(client, 0x0f, 0xfe);
				/* GPIO1 high, ov10635 out from reset */
#else
	/* s/w reset sensor */
	maxim_reg16_write(client, 0x103, 0x1);
	udelay(100);
#endif

	/* Program wizard registers */
	ret = ov10635_set_regs(client, ov10635_regs_wizard,
			       ARRAY_SIZE(ov10635_regs_wizard));
	if (ret)
		return ret;
#endif

	cam_idx = client->addr - CAM;
	/* switch to GMSL serial_link for streaming video */
	tmp_addr = client->addr;
	client->addr = maxim_map[1][cam_idx];	/* MAX9271-CAMx */
	maxim_reg8_write(client, 0x04, 0x83);
			/* enable reverse_control/serial_link */
	mdelay(2);	/* wait 2ms after changing reverse_control */
	client->addr = tmp_addr;

	return 0;
}

static int ov10635_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct ov10635_priv *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&priv->sd, client, &ov10635_subdev_ops);
	priv->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;

	v4l2_ctrl_handler_init(&priv->hdl, 4);
	v4l2_ctrl_new_std(&priv->hdl, &ov10635_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, 0, 255, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov10635_ctrl_ops,
			  V4L2_CID_CONTRAST, 0, 255, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov10635_ctrl_ops,
			  V4L2_CID_SATURATION, 0, 255, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov10635_ctrl_ops,
			  V4L2_CID_HUE, 0, 255, 1, 0);
	priv->sd.ctrl_handler = &priv->hdl;

	ret = priv->hdl.error;
	if (ret)
		goto cleanup;

	v4l2_ctrl_handler_setup(&priv->hdl);

	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
//	priv->sd.entity.flags |= MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_pads_init(&priv->sd.entity, 1, &priv->pad);
	if (ret < 0)
		goto cleanup;

	ret = ov10635_initialize(client);
	if (ret < 0)
		goto cleanup;

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret)
		goto cleanup;

	return 0;

cleanup:
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);
	v4l_err(client, "failed to probe @ 0x%02x (%s)\n",
		client->addr, client->adapter->name);
	return ret;
}

static int ov10635_remove(struct i2c_client *client)
{
	struct ov10635_priv *priv = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(&priv->sd);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);

	return 0;
}

static void ov10635_shutdown(struct i2c_client *client)
{
	struct ov10635_priv *priv = i2c_get_clientdata(client);

	/* make sure stream off during shutdown (reset/reboot) */
	ov10635_s_stream(&priv->sd, 0);
}

static const struct i2c_device_id ov10635_id[] = {
	{ "max9286-ov10635", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov10635_id);

static const struct of_device_id ov10635_of_ids[] = {
	{ .compatible = "maxim,max9286-ov10635", },
	{ }
};
MODULE_DEVICE_TABLE(of, ov10635_of_ids);

static struct i2c_driver ov10635_i2c_driver = {
	.driver	= {
		.name	= "max9286-ov10635",
		.of_match_table = ov10635_of_ids,
	},
	.probe		= ov10635_probe,
	.remove		= ov10635_remove,
	.shutdown	= ov10635_shutdown,
	.id_table	= ov10635_id,
};

module_i2c_driver(ov10635_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for MAX9286<->MAX9271<->OV10635");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
