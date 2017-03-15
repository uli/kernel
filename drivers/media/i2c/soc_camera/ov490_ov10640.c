/*
 * OmniVision ov490-ov10640 sensor camera driver
 *
 * Copyright (C) 2016-2017 Cogent Embedded, Inc.
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
#include "ov490_ov10640.h"

#define OV490_I2C_ADDR		0x24

#define OV490_PID		0x300a
#define OV490_VER		0x300b
#define OV490_VERSION_REG	0x0490
#define OV490_VERSION(pid, ver)	(((pid) << 8) | ((ver) & 0xff))
#define OV490_REV		0x0007

#define OV490_ISP_HSIZE_LOW	0x60
#define OV490_ISP_HSIZE_HIGH	0x61
#define OV490_ISP_VSIZE_LOW	0x62
#define OV490_ISP_VSIZE_HIGH	0x63

struct ov490_priv {
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
	char				is_fixed_sensor;
	int				init_complete;
	u8				id[6];
	int				exposure;
	int				gain;
	int				autogain;
	int				dvp_order;
};

static inline struct ov490_priv *to_ov490(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov490_priv, sd);
}

static int ov490_set_regs(struct i2c_client *client,
			  const struct ov490_reg *regs, int nr_regs)
{
	int i;

	for (i = 0; i < nr_regs; i++) {
		if (reg16_write(client, regs[i].reg, regs[i].val)) {
			usleep_range(100, 150); /* wait 100 us */
			reg16_write(client, regs[i].reg, regs[i].val);
		}
	}

	return 0;
}

static int ov490_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int ov490_get_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov490_priv *priv = to_ov490(client);

	if (format->pad)
		return -EINVAL;

	mf->width = priv->width;
	mf->height = priv->height;
	mf->code = MEDIA_BUS_FMT_YUYV8_2X8;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int ov490_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov490_priv *priv = to_ov490(client);

	mf->code = MEDIA_BUS_FMT_YUYV8_2X8;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		cfg->try_fmt = *mf;
		return 0;
	}

	if (priv->width != mf->width || priv->height != mf->height) {
		dev_info(&client->dev, "update firmware to use res %dx%d (current res %dx%d)\n",
				       mf->width, mf->height, priv->width, priv->height);

		mf->width = priv->width;
		mf->height = priv->height;
	}

	return 0;
}

static int ov490_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_YUYV8_2X8;

	return 0;
}

static int ov490_get_edid(struct v4l2_subdev *sd, struct v4l2_edid *edid)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov490_priv *priv = to_ov490(client);

	memcpy(edid->edid, priv->id, 6);

	edid->edid[6] = 0xff;
	edid->edid[7] = client->addr;
	edid->edid[8] = OV490_VERSION_REG >> 8;
	edid->edid[9] = OV490_VERSION_REG & 0xff;

	return 0;
}

#if 0
static int ov490_s_crop(struct v4l2_subdev *sd, const struct v4l2_crop *a)
{
	struct v4l2_crop a_writable = *a;
	struct v4l2_rect *rect = &a_writable.c;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov490_priv *priv = to_ov490(client);

	rect->left = 0;
	rect->top = 0;
	rect->width = priv->width;
	rect->height = priv->height;

	return 0;
}

static int ov490_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov490_priv *priv = to_ov490(client);

	a->c.left = 0;
	a->c.top = 0;
	a->c.width = priv->width;
	a->c.height = priv->height;
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}

static int ov490_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov490_priv *priv = to_ov490(client);

	a->bounds.left = 0;
	a->bounds.top = 0;
	a->bounds.width = priv->width;
	a->bounds.height = priv->height;
	a->defrect = a->bounds;
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}
#endif

static int ov490_g_mbus_config(struct v4l2_subdev *sd,
			       struct v4l2_mbus_config *cfg)
{
	cfg->flags = V4L2_MBUS_CSI2_1_LANE | V4L2_MBUS_CSI2_CHANNEL_0 |
		     V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	cfg->type = V4L2_MBUS_CSI2;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov490_g_register(struct v4l2_subdev *sd,
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

static int ov490_s_register(struct v4l2_subdev *sd,
			    const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = reg16_write(client, (u16)reg->reg, (u8)reg->val);
	if ((u8)reg->reg == 0xFFFE)
		usleep_range(100, 150); /* wait 100 us */
	return ret;
}
#endif

static struct v4l2_subdev_core_ops ov490_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov490_g_register,
	.s_register = ov490_s_register,
#endif
};

static int ov490_s_gamma(int a, int ref)
{
	if ((a + ref) > 0xff)
		return 0xff;

	if ((a + ref) < 0)
		return 0;

	return a + ref;
}

static int ov490_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov490_priv *priv = to_ov490(client);
	int ret = -EINVAL;

	if (!priv->init_complete)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		/* SDE (rough) brightness */
		ret = reg16_write(client, 0xFFFD, 0x80);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, 0x00);
		ret |= reg16_write(client, 0x5001, ctrl->val);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xf1);
		break;
	case V4L2_CID_CONTRAST:
		ret = reg16_write(client, 0xFFFD, 0x80);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, ctrl->val);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xfd);
		break;
	case V4L2_CID_SATURATION:
		ret = reg16_write(client, 0xFFFD, 0x80);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, ctrl->val);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xf3);
		break;
	case V4L2_CID_HUE:
		ret = reg16_write(client, 0xFFFD, 0x80);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, ctrl->val);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xf5);
		break;
	case V4L2_CID_GAMMA:
		ret = reg16_write(client, 0xFFFD, 0x80);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, ov490_s_gamma(ctrl->val, 0x12));
		ret |= reg16_write(client, 0x5001, ov490_s_gamma(ctrl->val, 0x20));
		ret |= reg16_write(client, 0x5002, ov490_s_gamma(ctrl->val, 0x3b));
		ret |= reg16_write(client, 0x5003, ov490_s_gamma(ctrl->val, 0x5d));
		ret |= reg16_write(client, 0x5004, ov490_s_gamma(ctrl->val, 0x6a));
		ret |= reg16_write(client, 0x5005, ov490_s_gamma(ctrl->val, 0x76));
		ret |= reg16_write(client, 0x5006, ov490_s_gamma(ctrl->val, 0x81));
		ret |= reg16_write(client, 0x5007, ov490_s_gamma(ctrl->val, 0x8b));
		ret |= reg16_write(client, 0x5008, ov490_s_gamma(ctrl->val, 0x96));
		ret |= reg16_write(client, 0x5009, ov490_s_gamma(ctrl->val, 0x9e));
		ret |= reg16_write(client, 0x500a, ov490_s_gamma(ctrl->val, 0xae));
		ret |= reg16_write(client, 0x500b, ov490_s_gamma(ctrl->val, 0xbc));
		ret |= reg16_write(client, 0x500c, ov490_s_gamma(ctrl->val, 0xcf));
		ret |= reg16_write(client, 0x500d, ov490_s_gamma(ctrl->val, 0xde));
		ret |= reg16_write(client, 0x500e, ov490_s_gamma(ctrl->val, 0xec));
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xf9);
		break;
	case V4L2_CID_SHARPNESS:
		ret = reg16_write(client, 0xFFFD, 0x80);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, ctrl->val);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xfb);
		break;
	case V4L2_CID_AUTOGAIN:
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
		if (ctrl->id == V4L2_CID_AUTOGAIN)
			priv->autogain = ctrl->val;
		if (ctrl->id == V4L2_CID_GAIN)
			priv->gain = ctrl->val;
		if (ctrl->id == V4L2_CID_EXPOSURE)
			priv->exposure = ctrl->val;

		ret = reg16_write(client, 0xFFFD, 0x80);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, !priv->autogain);
		ret |= reg16_write(client, 0x5001, priv->exposure >> 8);
		ret |= reg16_write(client, 0x5002, priv->exposure & 0xff);
		ret |= reg16_write(client, 0x5003, priv->exposure >> 8);
		ret |= reg16_write(client, 0x5004, priv->exposure & 0xff);
		ret |= reg16_write(client, 0x5005, priv->exposure >> 8);
		ret |= reg16_write(client, 0x5006, priv->exposure & 0xff);
		ret |= reg16_write(client, 0x5007, priv->gain >> 8);
		ret |= reg16_write(client, 0x5008, priv->gain & 0xff);
		ret |= reg16_write(client, 0x5009, priv->gain >> 8);
		ret |= reg16_write(client, 0x500a, priv->gain & 0xff);
		ret |= reg16_write(client, 0x500b, priv->gain >> 8);
		ret |= reg16_write(client, 0x500c, priv->gain & 0xff);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xea);
		break;
	case V4L2_CID_HFLIP:
#if 1
		ret = reg16_write(client, 0xFFFD, 0x80);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, ctrl->val);
		ret |= reg16_write(client, 0x5001, 0x00);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xdc);
#else
		ret = reg16_write(client, 0xFFFD, 0x80);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, 0x01); // read 0x3128
		ret |= reg16_write(client, 0x5001, 0x31);
		ret |= reg16_write(client, 0x5002, 0x28);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xc1);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_read(client, 0x5000, &val);
		val &= ~(0x1 << 0);
		val |= (ctrl->val << 0);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, 0x00); // write 0x3128
		ret |= reg16_write(client, 0x5001, 0x31);
		ret |= reg16_write(client, 0x5002, 0x28);
		ret |= reg16_write(client, 0x5003, val);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xc1);

		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, 0x01); // read 0x3291
		ret |= reg16_write(client, 0x5001, 0x32);
		ret |= reg16_write(client, 0x5002, 0x91);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xc1);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_read(client, 0x5000, &val);
		val &= ~(0x1 << 1);
		val |= (ctrl->val << 1);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, 0x00); // write 0x3291
		ret |= reg16_write(client, 0x5001, 0x32);
		ret |= reg16_write(client, 0x5002, 0x91);
		ret |= reg16_write(client, 0x5003, val);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xc1);

		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, 0x01); // read 0x3090
		ret |= reg16_write(client, 0x5001, 0x30);
		ret |= reg16_write(client, 0x5002, 0x90);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xc1);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_read(client, 0x5000, &val);
		val &= ~(0x1 << 2);
		val |= (ctrl->val << 2);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, 0x00); // write 0x3090
		ret |= reg16_write(client, 0x5001, 0x30);
		ret |= reg16_write(client, 0x5002, 0x90);
		ret |= reg16_write(client, 0x5003, val);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xc1);
#endif
		break;
	case V4L2_CID_VFLIP:
#if 1
		ret = reg16_write(client, 0xFFFD, 0x80);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, ctrl->val);
		ret |= reg16_write(client, 0x5001, 0x01);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xdc);
#else
		ret = reg16_write(client, 0xFFFD, 0x80);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, 0x01); // read 0x3128
		ret |= reg16_write(client, 0x5001, 0x31);
		ret |= reg16_write(client, 0x5002, 0x28);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xc1);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_read(client, 0x5000, &val);
		val &= ~(0x1 << 1);
		val |= (ctrl->val << 1);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, 0x00); // write 0x3128
		ret |= reg16_write(client, 0x5001, 0x31);
		ret |= reg16_write(client, 0x5002, 0x28);
		ret |= reg16_write(client, 0x5003, val);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xc1);

		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, 0x01); // read 0x3291
		ret |= reg16_write(client, 0x5001, 0x32);
		ret |= reg16_write(client, 0x5002, 0x91);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xc1);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_read(client, 0x5000, &val);
		val &= ~(0x1 << 2);
		val |= (ctrl->val << 2);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, 0x00); // write 0x3291
		ret |= reg16_write(client, 0x5001, 0x32);
		ret |= reg16_write(client, 0x5002, 0x91);
		ret |= reg16_write(client, 0x5003, val);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xc1);

		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, 0x01); // read 0x3090
		ret |= reg16_write(client, 0x5001, 0x30);
		ret |= reg16_write(client, 0x5002, 0x90);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xc1);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_read(client, 0x5000, &val);
		val &= ~(0x1 << 3);
		val |= (ctrl->val << 3);
		ret |= reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x5000, 0x00); // write 0x3090
		ret |= reg16_write(client, 0x5001, 0x30);
		ret |= reg16_write(client, 0x5002, 0x90);
		ret |= reg16_write(client, 0x5003, val);
		ret |= reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		ret |= reg16_write(client, 0x00C0, 0xc1);
#endif
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static const struct v4l2_ctrl_ops ov490_ctrl_ops = {
	.s_ctrl = ov490_s_ctrl,
};

static struct v4l2_subdev_video_ops ov490_video_ops = {
	.s_stream	= ov490_s_stream,
//	.cropcap	= ov490_cropcap,
//	.g_crop		= ov490_g_crop,
//	.s_crop		= ov490_s_crop,
	.g_mbus_config	= ov490_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops ov490_subdev_pad_ops = {
	.get_edid	= ov490_get_edid,
	.enum_mbus_code	= ov490_enum_mbus_code,
	.get_fmt	= ov490_get_fmt,
	.set_fmt	= ov490_set_fmt,
};

static struct v4l2_subdev_ops ov490_subdev_ops = {
	.core	= &ov490_core_ops,
	.video	= &ov490_video_ops,
	.pad	= &ov490_subdev_pad_ops,
};

static void ov490_otp_id_read(struct i2c_client *client)
{
	struct ov490_priv *priv = to_ov490(client);
	int i;

#if 0
	/* read camera id from ov490 OTP memory */
	reg16_write(client, 0xFFFD, 0x80);
	reg16_write(client, 0xFFFE, 0x28);
	usleep_range(100, 150); /* wait 100 us */
	reg16_write(client, 0xE084, 0x40); /* manual mode, bank#0 */
	reg16_write(client, 0xE081, 1); /* start OTP read */

	usleep_range(25000, 26000); /* wait 25 ms */

	for (i = 0; i < 6; i++)
		reg16_read(client, 0xe000 + i + 4, &priv->id[i]);
#else
	/* read camera id from ov10640 OTP memory */
	reg16_write(client, 0xFFFD, 0x80);
	reg16_write(client, 0xFFFE, 0x19);
	usleep_range(100, 150); /* wait 100 us */
	reg16_write(client, 0x5000, 0x00); /* write 0x349C -> 1 */
	reg16_write(client, 0x5001, 0x34);
	reg16_write(client, 0x5002, 0x9C);
	reg16_write(client, 0x5003, 1);
	reg16_write(client, 0xFFFE, 0x80);
	usleep_range(100, 150); /* wait 100 us */
	reg16_write(client, 0x00C0, 0xc1);

	usleep_range(25000, 25500); /* wait 25 ms */

	for (i = 0; i < 6; i++) {
		reg16_write(client, 0xFFFE, 0x19);
		usleep_range(100, 150); /* wait 100 us */
		reg16_write(client, 0x5000, 0x01); /* read (0x349E + i) */
		reg16_write(client, 0x5001, 0x34);
		reg16_write(client, 0x5002, 0x9e + i + 6); /* first 6 bytes are equal on all ov10640 */
		reg16_write(client, 0xFFFE, 0x80);
		usleep_range(100, 150); /* wait 100 us */
		reg16_write(client, 0x00C0, 0xc1);
		reg16_write(client, 0xFFFE, 0x19);
		usleep_range(1000, 1500); /* wait 1 ms */
		reg16_read(client, 0x5000, &priv->id[i]);
	}
#endif
}

static ssize_t ov490_otp_id_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(to_i2c_client(dev));
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov490_priv *priv = to_ov490(client);

	return snprintf(buf, 32, "%02x:%02x:%02x:%02x:%02x:%02x\n",
			priv->id[0], priv->id[1], priv->id[2], priv->id[3], priv->id[4], priv->id[5]);
}

static DEVICE_ATTR(otp_id_ov490, S_IRUGO, ov490_otp_id_show, NULL);

static int ov490_initialize(struct i2c_client *client)
{
	struct ov490_priv *priv = to_ov490(client);
	u8 val = 0;
	u8 pid = 0, ver = 0, rev = 0;
	int ret;

	if (priv->is_fixed_sensor) {
		dev_info(&client->dev, "ov490/ov10640 fixed-sensor res %dx%d\n", priv->width, priv->height);
		return 0;
	}

	/* check and show product ID and manufacturer ID */
	ret = reg16_write(client, 0xFFFD, 0x80);
	if (ret)
		return ret;
	ret = reg16_write(client, 0xFFFE, 0x80);
	if (ret)
		return ret;
	usleep_range(100, 150); /* wait 100 us */
	ret = reg16_read(client, OV490_PID, &pid);
	if (ret)
		return ret;
	ret = reg16_read(client, OV490_VER, &ver);
	if (ret)
		return ret;
	ret = reg16_read(client, OV490_REV, &rev);
	if (ret)
		return ret;

	if (OV490_VERSION(pid, ver) != OV490_VERSION_REG) {
		dev_dbg(&client->dev, "Product ID error %x:%x\n", pid, ver);
		return -ENODEV;
	}

	/* read resolution used by current firmware */
	reg16_write(client, 0xFFFD, 0x80);
	reg16_write(client, 0xFFFE, 0x82);
	usleep_range(100, 150); /* wait 100 us */
	reg16_read(client, OV490_ISP_HSIZE_HIGH, &val);
	priv->width = val;
	reg16_read(client, OV490_ISP_HSIZE_LOW, &val);
	priv->width = (priv->width << 8) | val;

	reg16_read(client, OV490_ISP_VSIZE_HIGH, &val);
	priv->height = val;
	reg16_read(client, OV490_ISP_VSIZE_LOW, &val);
	priv->height = (priv->height << 8) | val;

	ret = ov490_set_regs(client, ov490_regs_wizard,
			     ARRAY_SIZE(ov490_regs_wizard));
	if (ret)
		return ret;

	/* Set DVP bit swap */
	reg16_write(client, 0xFFFD, 0x80);
	reg16_write(client, 0xFFFE, 0x28);
	usleep_range(100, 150); /* wait 100 us */
	reg16_write(client, 0x6009, priv->dvp_order << 4);

	ov490_otp_id_read(client);

	dev_info(&client->dev, "ov490/ov10640 Product ID %x Manufacturer ID %x, rev 1%x, res %dx%d, OTP_ID %02x:%02x:%02x:%02x:%02x:%02x\n",
		 pid, ver, 0xa + rev, priv->width, priv->height, priv->id[0], priv->id[1], priv->id[2], priv->id[3], priv->id[4], priv->id[5]);

	return 0;
}

static int ov490_parse_dt(struct device_node *np, struct ov490_priv *priv)
{
	struct i2c_client *client = v4l2_get_subdevdata(&priv->sd);
	int err, i;
	const char *fixed_sensor;
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
		reg8_write(client, 0x0A, OV490_I2C_ADDR << 1);		/* Sensor native I2C address */
		usleep_range(2000, 2500);				/* wait 2ms */
	};
	if (priv->ti964_addr) {
		client->addr = priv->ti964_addr;			/* Deserializer I2C address */

		reg8_write(client, 0x4c, (priv->ti913_port << 4) | (1 << priv->ti913_port)); /* Select RX port number */
		usleep_range(2000, 2500);				/* wait 2ms */
		reg8_write(client, 0x65, tmp_addr << 1);		/* Sensor translated I2C address */
		reg8_write(client, 0x5d, OV490_I2C_ADDR << 1);		/* Sensor native I2C address */

		reg8_write(client, 0x6e, 0x9a);				/* GPIO0 - resetb, GPIO1 - fsin */
		/* TODO: why too long? move logic to workqueue? */
		mdelay(350);						/* time needed to boot all sensor IPs */
	}
	client->addr = tmp_addr;

	err = of_property_read_string(np, "maxim,fixed-sensor", &fixed_sensor);
	if (err)
		return 0;

	if (strcmp(fixed_sensor, "ov490") == 0) {
		err = of_property_read_u32(np, "maxim,width", &priv->width);
		if (err) {
			dev_err(&client->dev, "maxim,width must be set for fixed-sensor\n");
			goto out;
		}

		err = of_property_read_u32(np, "maxim,height", &priv->height);
		if (err) {
			dev_err(&client->dev, "maxim,height must be set for fixed-sensor\n");
			goto out;
		}

		priv->is_fixed_sensor = true;
	}

out:
	return err;
}

static int ov490_probe(struct i2c_client *client,
		       const struct i2c_device_id *did)
{
	struct ov490_priv *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&priv->sd, client, &ov490_subdev_ops);
	priv->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;

	priv->exposure = 0x100;
	priv->gain = 0x100;
	priv->autogain = 1;
	v4l2_ctrl_handler_init(&priv->hdl, 4);
	v4l2_ctrl_new_std(&priv->hdl, &ov490_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, 0, 16, 1, 7);
	v4l2_ctrl_new_std(&priv->hdl, &ov490_ctrl_ops,
			  V4L2_CID_CONTRAST, 0, 16, 1, 7);
	v4l2_ctrl_new_std(&priv->hdl, &ov490_ctrl_ops,
			  V4L2_CID_SATURATION, 0, 7, 1, 2);
	v4l2_ctrl_new_std(&priv->hdl, &ov490_ctrl_ops,
			  V4L2_CID_HUE, 0, 23, 1, 12);
	v4l2_ctrl_new_std(&priv->hdl, &ov490_ctrl_ops,
			  V4L2_CID_GAMMA, -128, 128, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov490_ctrl_ops,
			  V4L2_CID_SHARPNESS, 0, 10, 1, 3);
	v4l2_ctrl_new_std(&priv->hdl, &ov490_ctrl_ops,
			  V4L2_CID_AUTOGAIN, 0, 1, 1, priv->autogain);
	v4l2_ctrl_new_std(&priv->hdl, &ov490_ctrl_ops,
			  V4L2_CID_GAIN, 0, 0xffff, 1, priv->gain);
	v4l2_ctrl_new_std(&priv->hdl, &ov490_ctrl_ops,
			  V4L2_CID_EXPOSURE, 0, 0xffff, 1, priv->exposure);
	v4l2_ctrl_new_std(&priv->hdl, &ov490_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 1);
	v4l2_ctrl_new_std(&priv->hdl, &ov490_ctrl_ops,
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

	ret = ov490_parse_dt(client->dev.of_node, priv);
	if (ret)
		goto cleanup;

	ret = ov490_initialize(client);
	if (ret < 0)
		goto cleanup;

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret)
		goto cleanup;

	if (device_create_file(&client->dev, &dev_attr_otp_id_ov490) != 0) {
		dev_err(&client->dev, "sysfs otp_id entry creation failed\n");
		goto cleanup;
	}

	priv->init_complete = 1;

	return 0;

cleanup:
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);
#ifdef CONFIG_SOC_CAMERA_OV490_OV10640
	v4l_err(client, "failed to probe @ 0x%02x (%s)\n",
		client->addr, client->adapter->name);
#endif
	return ret;
}

static int ov490_remove(struct i2c_client *client)
{
	struct ov490_priv *priv = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_otp_id_ov490);
	v4l2_async_unregister_subdev(&priv->sd);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);

	return 0;
}

#ifdef CONFIG_SOC_CAMERA_OV490_OV10640
static const struct i2c_device_id ov490_id[] = {
	{ "ov490-ov10640", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov490_id);

static const struct of_device_id ov490_of_ids[] = {
	{ .compatible = "ovti,ov490-ov10640", },
	{ }
};
MODULE_DEVICE_TABLE(of, ov490_of_ids);

static struct i2c_driver ov490_i2c_driver = {
	.driver	= {
		.name		= "ov490-ov10640",
		.of_match_table	= ov490_of_ids,
	},
	.probe		= ov490_probe,
	.remove		= ov490_remove,
	.id_table	= ov490_id,
};

module_i2c_driver(ov490_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for OV490-OV10640");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
#endif
