/*
 * OmniVision ov10635/ov490-ov10640 sensor camera driver
 *
 * Copyright (C) 2016-2017 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include "ov10635.c"
#include "ov490_ov10640.c"

static enum {
	ID_OV10635,
	ID_OV490_OV10640,
} chip_id;

static int ov106xx_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	int ret;
	chip_id = -EINVAL;

	ret = ov10635_probe(client, did);
	if (!ret) {
		chip_id = ID_OV10635;
		goto out;
	}

	ret = ov490_probe(client, did);
	if (!ret) {
		chip_id = ID_OV490_OV10640;
		goto out;
	}

	v4l_err(client, "failed to probe @ 0x%02x (%s)\n",
			client->addr, client->adapter->name);
out:
	return ret;
}

static int ov106xx_remove(struct i2c_client *client)
{
	switch (chip_id) {
	case ID_OV10635:
		ov10635_remove(client);
		break;
	case ID_OV490_OV10640:
		ov490_remove(client);
		break;
	};

	return 0;
}

static const struct i2c_device_id ov106xx_id[] = {
	{ "ov106xx", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov106xx_id);

static const struct of_device_id ov106xx_of_ids[] = {
	{ .compatible = "ovti,ov106xx", },
	{ }
};
MODULE_DEVICE_TABLE(of, ov106xx_of_ids);

static struct i2c_driver ov106xx_i2c_driver = {
	.driver	= {
		.name		= "ov106xx",
		.of_match_table	= ov106xx_of_ids,
	},
	.probe		= ov106xx_probe,
	.remove		= ov106xx_remove,
	.id_table	= ov106xx_id,
};

module_i2c_driver(ov106xx_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for OV10635 or OV490/OV10640");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
