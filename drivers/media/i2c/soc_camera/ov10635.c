/*
 * OmniVision ov10635 sensor camera driver
 *
 * Copyright (C) 2015-2017 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/videodev2.h>

#include <media/soc_camera.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-of.h>

#include "max9286_max9271.h"
#include "ov10635.h"

#define OV10635_I2C_ADDR		0x30

#define OV10635_PID			0x300a
#define OV10635_VER			0x300b
#define OV10635_VERSION_REG		0xa635
#define OV10635_VERSION(pid, ver)	(((pid) << 8) | ((ver) & 0xff))

struct ov10635_priv {
	struct v4l2_subdev		sd;
	struct v4l2_ctrl_handler	hdl;
	struct media_pad		pad;
	int				max9271_addr;
	int				ti964_addr;
	int				ti913_port;
	int				gpio_resetb;
	int				gpio_fsin;
	int				width;
	int				height;
	int				fps_denominator;
	int				init_complete;
	u8				id[6];
	int				dvp_order;
};

static inline struct ov10635_priv *to_ov10635(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov10635_priv, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ov10635_priv, hdl)->sd;
}

static int ov10635_set_regs(struct i2c_client *client,
			    const struct ov10635_reg *regs, int nr_regs)
{
	int i;

	for (i = 0; i < nr_regs; i++) {
		if (reg16_write(client, regs[i].reg, regs[i].val)) {
			usleep_range(100, 150); /* wait 100ns */
			reg16_write(client, regs[i].reg, regs[i].val);
		}
	}

	return 0;
}

static int ov10635_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int ov10635_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov10635_priv *priv = to_ov10635(client);

	if (format->pad)
		return -EINVAL;

	mf->width = priv->width;
	mf->height = priv->height;
	mf->code = MEDIA_BUS_FMT_YUYV8_2X8;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int ov10635_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov10635_priv *priv = to_ov10635(client);
	int ret = 0;

	mf->code = MEDIA_BUS_FMT_YUYV8_2X8;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		cfg->try_fmt = *mf;
		return 0;
	}

	if (!((mf->width == 1280 && mf->height == 800) ||
	      (mf->width == 640 && mf->height == 400))) {
		mf->width = 1280;
		mf->height = 800;
	}

	if (priv->width != mf->width) {
		priv->width = mf->width;
		priv->height = mf->height;

		switch(priv->width) {
		case 1280:
			ret = ov10635_set_regs(client, ov10635_regs_1280x800,
					       ARRAY_SIZE(ov10635_regs_1280x800));
			break;
		case 640:
			ret = ov10635_set_regs(client, ov10635_regs_640x400,
					       ARRAY_SIZE(ov10635_regs_640x400));
			break;
		}
	}

	return ret;
}

static int ov10635_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_YUYV8_2X8;

	return 0;
}

static int ov10635_get_edid(struct v4l2_subdev *sd, struct v4l2_edid *edid)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov10635_priv *priv = to_ov10635(client);

	memcpy(edid->edid, priv->id, 6);

	edid->edid[6] = 0xff;
	edid->edid[7] = client->addr;
	edid->edid[8] = OV10635_VERSION_REG >> 8;
	edid->edid[9] = OV10635_VERSION_REG & 0xff;

	return 0;
}

#if 0
static int ov10635_s_crop(struct v4l2_subdev *sd, const struct v4l2_crop *a)
{
	struct v4l2_crop a_writable = *a;
	struct v4l2_rect *rect = &a_writable.c;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov10635_priv *priv = to_ov10635(client);

	rect->left = 0;
	rect->top = 0;
	rect->width = priv->width;
	rect->height = priv->height;

	return 0;
}

static int ov10635_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov10635_priv *priv = to_ov10635(client);

	a->c.left = 0;
	a->c.top = 0;
	a->c.width = priv->width;
	a->c.height = priv->height;
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}
#endif

#if 0
static int ov10635_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left = 0;
	a->bounds.top = 0;
	a->bounds.width = OV10635_MAX_WIDTH;
	a->bounds.height = OV10635_MAX_HEIGHT;
	a->defrect = a->bounds;
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}
#endif

static int ov10635_g_mbus_config(struct v4l2_subdev *sd,
				 struct v4l2_mbus_config *cfg)
{
	cfg->flags = V4L2_MBUS_CSI2_1_LANE | V4L2_MBUS_CSI2_CHANNEL_0 |
		     V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	cfg->type = V4L2_MBUS_CSI2;

	return 0;
}

static int ov10635_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov10635_priv *priv = to_ov10635(client);
	struct v4l2_captureparm *cp = &parms->parm.capture;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;
	cp->timeperframe.denominator = priv->fps_denominator;

	return 0;
}

static int ov10635_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov10635_priv *priv = to_ov10635(client);
	struct v4l2_captureparm *cp = &parms->parm.capture;
	int ret = 0;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (cp->extendedmode != 0)
		return -EINVAL;

	if (priv->fps_denominator != cp->timeperframe.denominator) {
		switch (cp->timeperframe.denominator) {
		case 5:
			ret = ov10635_set_regs(client, ov10635_regs_5fps,
					       ARRAY_SIZE(ov10635_regs_5fps));
			break;
		case 10:
			ret = ov10635_set_regs(client, ov10635_regs_10fps,
					       ARRAY_SIZE(ov10635_regs_10fps));
			break;
		case 15:
			ret = ov10635_set_regs(client, ov10635_regs_15fps,
					       ARRAY_SIZE(ov10635_regs_15fps));
			break;
		case 30:
			ret = ov10635_set_regs(client, ov10635_regs_30fps,
					       ARRAY_SIZE(ov10635_regs_30fps));
			break;
		default:
			return -EINVAL;
		}

		priv->fps_denominator = cp->timeperframe.denominator;
	}

	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov10635_g_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 val = 0;

	ret = reg16_read(client, (u16)reg->reg, &val);
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

	return reg16_write(client, (u16)reg->reg, (u8)reg->val);
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
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov10635_priv *priv = to_ov10635(client);
	int ret = -EINVAL;
	u8 val = 0;

	if (!priv->init_complete)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		/* AEC/AGC target */
		ret = reg16_write(client, 0xc46a, ctrl->val);
		break;
	case V4L2_CID_CONTRAST:
		udelay(100);
		ret = ov10635_set_regs(client, &ov10635_regs_contrast[ctrl->val][0], 18);
		break;
	case V4L2_CID_SATURATION:
		ret = reg16_write(client, 0xc316, ctrl->val);
		break;
	case V4L2_CID_HUE:
		/* CMX ? */
		return 0;
	case V4L2_CID_GAMMA:
		ret = reg16_write(client, 0xc4be, ctrl->val >> 8);
		ret |= reg16_write(client, 0xc4bf, ctrl->val & 0xff);
		break;
	case V4L2_CID_AUTOGAIN:
		/* automatic gain/exposure */
		ret = reg16_write(client, 0x56d0, !ctrl->val);
		break;
	case V4L2_CID_GAIN:
		/* manual gain */
		ret = reg16_write(client, 0x3504, 0);
		ret |= reg16_write(client, 0x56d1, ctrl->val >> 8);
		ret |= reg16_write(client, 0x56d2, ctrl->val & 0xff);
		ret |= reg16_write(client, 0x3504, 1); /* validate gain */
		break;
	case V4L2_CID_EXPOSURE:
		/* maual exposure */
		ret = reg16_write(client, 0x3504, 0);
		ret |= reg16_write(client, 0x56d5, ctrl->val >> 8);
		ret |= reg16_write(client, 0x56d6, ctrl->val & 0xff);
		ret |= reg16_write(client, 0x3504, 1); /* validate exposure */
		break;
	case V4L2_CID_HFLIP:
		ret = reg16_read(client, 0x381d, &val);
		if (ret < 0)
			return ret;
		if (ctrl->val)
			val |= 0x3;
		else
			val &= ~0x3;
		ret = reg16_write(client, 0x381d, val);
		break;
	case V4L2_CID_VFLIP:
		ret = reg16_read(client, 0x381c, &val);
		if (ret < 0)
			return ret;
		if (ctrl->val)
			val |= 0xc0;
		else
			val &= ~0xc0;
		ret = reg16_write(client, 0x381c, val);
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops ov10635_ctrl_ops = {
	.s_ctrl = ov10635_s_ctrl,
};

static struct v4l2_subdev_video_ops ov10635_video_ops = {
	.s_stream	= ov10635_s_stream,
//	.cropcap	= ov10635_cropcap,
//	.g_crop		= ov10635_g_crop,
//	.s_crop		= ov10635_s_crop,
	.g_mbus_config	= ov10635_g_mbus_config,
	.g_parm		= ov10635_g_parm,
	.s_parm		= ov10635_s_parm,
};

static const struct v4l2_subdev_pad_ops ov10635_subdev_pad_ops = {
	.get_edid	= ov10635_get_edid,
	.enum_mbus_code	= ov10635_enum_mbus_code,
	.get_fmt	= ov10635_get_fmt,
	.set_fmt	= ov10635_set_fmt,
};

static struct v4l2_subdev_ops ov10635_subdev_ops = {
	.core	= &ov10635_core_ops,
	.video	= &ov10635_video_ops,
	.pad	= &ov10635_subdev_pad_ops,
};

static void ov10635_otp_id_read(struct i2c_client *client)
{
	struct ov10635_priv *priv = to_ov10635(client);
	int i;

	/* read camera id from OTP memory */
	reg16_write(client, 0x3d10, 1);

	usleep_range(15000, 16000); /* wait 15ms */

	for (i = 0; i < 6; i++)
		reg16_read(client, 0x3d00 + i, &priv->id[i]);
}

static ssize_t ov10635_otp_id_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(to_i2c_client(dev));
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov10635_priv *priv = to_ov10635(client);

	return snprintf(buf, 32, "%02x:%02x:%02x:%02x:%02x:%02x\n",
			priv->id[0], priv->id[1], priv->id[2], priv->id[3], priv->id[4], priv->id[5]);
}

static DEVICE_ATTR(otp_id_ov10635, S_IRUGO, ov10635_otp_id_show, NULL);

static int ov10635_initialize(struct i2c_client *client)
{
	struct ov10635_priv *priv = to_ov10635(client);
	u8 pid = 0, ver = 0;
	int ret;

	/* check and show product ID and manufacturer ID */
	ret = reg16_read(client, OV10635_PID, &pid);
	if (ret)
		return ret;
	ret = reg16_read(client, OV10635_VER, &ver);
	if (ret)
		return ret;

	if (OV10635_VERSION(pid, ver) != OV10635_VERSION_REG) {
		dev_err(&client->dev, "Product ID error %x:%x\n", pid, ver);
		return -ENODEV;
	}

	/* s/w reset sensor */
	reg16_write(client, 0x103, 0x1);
	udelay(100);
	/* Program wizard registers */
	ret = ov10635_set_regs(client, ov10635_regs_wizard,
			       ARRAY_SIZE(ov10635_regs_wizard));
	if (ret)
		return ret;

	/* Set DVP bit swap */
	reg16_write(client, 0x4709, priv->dvp_order << 4);

	ov10635_otp_id_read(client);

	dev_info(&client->dev, "ov10635 Product ID %x Manufacturer ID %x OTP_ID %02x:%02x:%02x:%02x:%02x:%02x\n",
		 pid, ver, priv->id[0], priv->id[1], priv->id[2], priv->id[3], priv->id[4], priv->id[5]);

	return 0;
}

static int ov10635_parse_dt(struct device_node *np, struct ov10635_priv *priv)
{
	struct i2c_client *client = v4l2_get_subdevdata(&priv->sd);
	int i;
	struct device_node *endpoint = NULL, *rendpoint = NULL;
	int tmp_addr;

	for (i = 0; ; i++) {
		endpoint = of_graph_get_next_endpoint(np, endpoint);
		if (!endpoint)
			break;

		of_node_put(endpoint);

		of_property_read_u32(endpoint, "dvp-order", &priv->dvp_order);

		rendpoint = of_parse_phandle(endpoint, "remote-endpoint", 0);
		if (!rendpoint)
			continue;

		of_property_read_u32(rendpoint, "max9271-addr", &priv->max9271_addr);
		of_property_read_u32(rendpoint, "ti964-addr", &priv->ti964_addr);
		of_property_read_u32(rendpoint, "ti913-port", &priv->ti913_port);
	}

	if (!priv->max9271_addr && !priv->ti964_addr) {
		dev_err(&client->dev, "max9271-addr or ti964-addr not set\n");
		return -EINVAL;
	}

	/* setup I2C translator address */
	tmp_addr = client->addr;
	if (priv->max9271_addr) {
		client->addr = priv->max9271_addr;			/* Serializer I2C address */

		reg8_write(client, 0x09, tmp_addr << 1);		/* Sensor translated I2C address */
		reg8_write(client, 0x0A, OV10635_I2C_ADDR << 1);	/* Sensor native I2C address */
		usleep_range(2000, 2500);				/* wait 2ms */
	};
	if (priv->ti964_addr) {
		client->addr = priv->ti964_addr;			/* Deserializer I2C address */

		reg8_write(client, 0x4c, (priv->ti913_port << 4) | (1 << priv->ti913_port)); /* Select RX port number */
		usleep_range(2000, 2500);				/* wait 2ms */
		reg8_write(client, 0x65, tmp_addr << 1);		/* Sensor translated I2C address */
		reg8_write(client, 0x5d, OV10635_I2C_ADDR << 1);	/* Sensor native I2C address */

		reg8_write(client, 0x6e, 0xa9);				/* GPIO0 - resetb, GPIO1 - fsin */
		udelay(100);
	}
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
	priv->width = OV10635_MAX_WIDTH;
	priv->height = OV10635_MAX_HEIGHT;
	priv->fps_denominator = 30;

	v4l2_ctrl_handler_init(&priv->hdl, 4);
	v4l2_ctrl_new_std(&priv->hdl, &ov10635_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, 0, 0xff, 1, 0x30);
	v4l2_ctrl_new_std(&priv->hdl, &ov10635_ctrl_ops,
			  V4L2_CID_CONTRAST, 0, 4, 1, 2);
	v4l2_ctrl_new_std(&priv->hdl, &ov10635_ctrl_ops,
			  V4L2_CID_SATURATION, 0, 0xff, 1, 0xff);
	v4l2_ctrl_new_std(&priv->hdl, &ov10635_ctrl_ops,
			  V4L2_CID_HUE, 0, 255, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov10635_ctrl_ops,
			  V4L2_CID_GAMMA, 0, 0xffff, 1, 0x233);
	v4l2_ctrl_new_std(&priv->hdl, &ov10635_ctrl_ops,
			  V4L2_CID_AUTOGAIN, 0, 1, 1, 1);
	v4l2_ctrl_new_std(&priv->hdl, &ov10635_ctrl_ops,
			  V4L2_CID_GAIN, 0, 0x3ff, 1, 0x10);
	v4l2_ctrl_new_std(&priv->hdl, &ov10635_ctrl_ops,
			  V4L2_CID_EXPOSURE, 0, 0xffff, 1, 0x80);
	v4l2_ctrl_new_std(&priv->hdl, &ov10635_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov10635_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	priv->sd.ctrl_handler = &priv->hdl;

	ret = priv->hdl.error;
	if (ret)
		goto cleanup;

	v4l2_ctrl_handler_setup(&priv->hdl);

	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->sd.entity.flags |= MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&priv->sd.entity, 1, &priv->pad);
	if (ret < 0)
		goto cleanup;

	ret = ov10635_parse_dt(client->dev.of_node, priv);
	if (ret)
		goto cleanup;

	ret = ov10635_initialize(client);
	if (ret < 0)
		goto cleanup;

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret)
		goto cleanup;

	if (device_create_file(&client->dev, &dev_attr_otp_id_ov10635) != 0) {
		dev_err(&client->dev, "sysfs otp_id entry creation failed\n");
		goto cleanup;
	}

	priv->init_complete = 1;

	return 0;

cleanup:
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);
#ifdef CONFIG_SOC_CAMERA_OV10635
	v4l_err(client, "failed to probe @ 0x%02x (%s)\n",
		client->addr, client->adapter->name);
#endif
	return ret;
}

static int ov10635_remove(struct i2c_client *client)
{
	struct ov10635_priv *priv = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_otp_id_ov10635);
	v4l2_async_unregister_subdev(&priv->sd);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);

	return 0;
}

#ifdef CONFIG_SOC_CAMERA_OV10635
static const struct i2c_device_id ov10635_id[] = {
	{ "ov10635", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov10635_id);

static const struct of_device_id ov10635_of_ids[] = {
	{ .compatible = "ovti,ov10635", },
	{ }
};
MODULE_DEVICE_TABLE(of, ov10635_of_ids);

static struct i2c_driver ov10635_i2c_driver = {
	.driver	= {
		.name		= "ov10635",
		.of_match_table	= ov10635_of_ids,
	},
	.probe		= ov10635_probe,
	.remove		= ov10635_remove,
	.id_table	= ov10635_id,
};

module_i2c_driver(ov10635_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for OV10635");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
#endif
