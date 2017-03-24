/*
 * MAXIM R-Car H2 Demo board setup include file
 *
 * Copyright (C) 2016 Renesas Electronics Corporation
 * Copyright (C) 2015 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

/* #define DEBUG */
	/* If you use this define, please remove this commment out. */
#ifdef DEBUG
/* #define WRITE_VERIFY */
#define MAXIM_DUMP
#undef dev_dbg
#define dev_dbg dev_info
#endif
/* #define USE_P1 */
	/* If you use P1 board, please remove this commment out. */
/* #define USE_CH1 */
/* #define USE_CH2 */
	/* If you change channel setting, please remove this commment out. */

#if  defined(USE_CH1)
#define MAXIM_NUM		1	/* number of cameras */
#elif defined(USE_CH2)
#define MAXIM_NUM		2	/* number of cameras */
#else
#define MAXIM_NUM		4	/* number of cameras */
#endif

#define MAXIM_NUM_RETRIES	1	/* number of read/write retries */

#define MAXIM_I2C_I2C_SPEED_400KHZ	(0x5 << 2) /* 339 kbps */
#define MAXIM_I2C_I2C_SPEED_100KHZ	(0x3 << 2) /* 105 kbps */
#define MAXIM_I2C_SPEED			MAXIM_I2C_I2C_SPEED_100KHZ

/* MCU powered IMI cameras require delay between power-on and
 * RCar access to avoid i2c bus conflicts
 */
#define MAXIM_IMI_MCU_V0_DELAY	8000	/* delay for powered MCU firmware v0 */
#define MAXIM_IMI_MCU_V1_DELAY	3000	/* delay for powered MCU firmware v1 */
#define MAXIM_IMI_MCU_NO_DELAY	0	/* delay for unpowered MCU  */
#define MAXIM_IMI_MCU_DELAY	MAXIM_IMI_MCU_V0_DELAY

# if 0
#define MAXIM_IMI_MCU_POWERED
	/* skip ov10635 setup for IMI powered MCU (only fw later then v1) */

#define YUV_10BIT
#define OV10635_DISPLAY_PATTERN
#endif

/*
 * I2C MAP.
 *
 *		CAM0	CAM1	CAM2	CAM3
 * MAX9286	0x48+1	0x48+2	0x48+3	0x48+4	- deserializer
 * MAX9271	0x40+1	0x40+2	0x40+3	0x40+4	- serializer
 * OV10635	0x30+1	0x30+2	0x30+3	0x30+4	- sensor
 */

#ifdef USE_P1
#define DES0		0x48
#else
#define DES0		0x4c
#endif
#define DES1		0x6c
#define SER		0x51
#define CAM		0x60
#define BROADCAST	0x6f

static u8 maxim_map[][8] = {
	{ DES0 + 0, DES0 + 0, DES0 + 0, DES0 + 0,
		DES1 + 0, DES1 + 0, DES1 + 0, DES1 + 0 },
	{ SER  + 0, SER  + 1, SER  + 2, SER  + 3,
		SER  + 4, SER  + 5, SER  + 6, SER  + 7 },
	{ CAM  + 0, CAM  + 1, CAM  + 2, CAM  + 3,
		CAM  + 4, CAM  + 5, CAM  + 6, CAM  + 7 },
};

static inline int maxim_reg8_read(struct i2c_client *client, u8 reg, u8 *val)
{
	int ret;
	int retries = MAXIM_NUM_RETRIES;

	for (retries = MAXIM_NUM_RETRIES; retries; retries--) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (!(ret < 0))
			break;
	}

	if (ret < 0) {
		dev_err(&client->dev,
			"read fail: chip 0x%x register 0x%x\n", client->addr,
								reg);
	} else {
		*val = ret;
	}

	return ret;
}

static inline int maxim_reg8_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	int retries = MAXIM_NUM_RETRIES;
#ifdef WRITE_VERIFY
	u8 val2;
#endif

	for (retries = MAXIM_NUM_RETRIES; retries; retries--) {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (!(ret < 0))
			break;
	}

	if (ret < 0) {
		/* dbg here for skipping error messaging on reset/reboot case */
		dev_dbg(&client->dev,
			"write fail: chip 0x%x register 0x%x\n", client->addr,
								 reg);
	} else {
#ifdef WRITE_VERIFY
		maxim_reg8_read(client, reg, &val2);
		if (val != val2)
			dev_err(&client->dev,
			"write verify mismatch: chip 0x%x reg=0x%x 0x%x->0x%x\n",
				client->addr, reg, val, val2);
#endif
	}

	return ret;
}

static inline int maxim_reg16_read(struct i2c_client *client, u16 reg, u8 *val)
{
	int ret;
	int retries = MAXIM_NUM_RETRIES;
	u8 buf[2] = {reg >> 8, reg & 0xff};

	for (retries = MAXIM_NUM_RETRIES; retries; retries--) {
		ret = i2c_master_send(client, buf, 2);
		if (ret == 2) {
			ret = i2c_master_recv(client, buf, 1);
			if (ret == 1)
				break;
		}
	}

	if (ret < 0) {
		dev_err(&client->dev,
			"read fail: chip 0x%x register 0x%x\n", client->addr,
								reg);
	} else {
		*val = buf[0];
	}

	return ret < 0 ? ret : 0;
}

static inline int maxim_reg16_write(struct i2c_client *client, u16 reg, u8 val)
{
	int ret;
	int retries = MAXIM_NUM_RETRIES;
	u8 buf[3] = {reg >> 8, reg & 0xff, val};
#ifdef WRITE_VERIFY
	u8 val2;
#endif

	for (retries = MAXIM_NUM_RETRIES; retries; retries--) {
		ret = i2c_master_send(client, buf, 3);
		if (ret == 3)
			break;
	}

	if (ret < 0) {
		dev_err(&client->dev,
			"write fail: chip 0x%x register 0x%x\n", client->addr,
								 reg);
	} else {
#ifdef WRITE_VERIFY
		maxim_reg16_read(client, reg, &val2);
		if (val != val2)
			dev_err(&client->dev,
			"write verify mismatch: chip 0x%x reg=0x%x 0x%x->0x%x\n",
			client->addr, reg, val, val2);
#endif
	}

	return ret < 0 ? ret : 0;
}

static inline int maxim_reg16_write16
	(struct i2c_client *client, u16 reg, u16 val)
{
	int ret;

	ret = maxim_reg16_write(client, reg, val >> 8);
	if (ret)
		return ret;

	ret = maxim_reg16_write(client, reg + 1, val & 0xff);
	if (ret)
		return ret;

	return 0;
}

#ifdef MAXIM_DUMP
static void maxim_ov10635_dump_regs(struct i2c_client *client)
{
	int ret, i;
	u8 val = 0;
	u16 regs[] = {0x300a, 0x300b, 0x300c};

	dev_dbg(&client->dev, "dump regs 0x%x\n", client->addr);

	for (i = 0; i < sizeof(regs) / 2; i++) {
		ret = maxim_reg16_read(client, regs[i], &val);
		if (ret < 0)
			dev_err(&client->dev,
				"read fail: chip 0x%x register 0x%02x\n",
				client->addr, regs[i]);
		dev_dbg(&client->dev, "0x%02x -> 0x%x\n", regs[i], val);
	}
}

static void maxim_ov10635_dump_format_regs(struct i2c_client *client)
{
	int ret, i;
	u8 val;
	u16 regs[] = {0x3003, 0x3004, 0x4300,
		      0x4605, 0x3621, 0x3702, 0x3703, 0x3704,
		      0x3802, 0x3803, 0x3806, 0x3807, 0x3808, 0x3809, 0x380a,
		      0x380b, 0x380c, 0x380d, 0x380e, 0x380f,
		      0x4606, 0x4607, 0x460a, 0x460b,
		      0xc488, 0xc489, 0xc48a, 0xc48b,
		      0xc4cc, 0xc4cd, 0xc4ce, 0xc4cf, 0xc512, 0xc513,
		      0xc518, 0xc519, 0xc51a, 0xc51b,
	};

	dev_dbg(&client->dev, "dump regs 0x%x\n", client->addr);

	for (i = 0; i < sizeof(regs) / 2; i++) {
		ret = maxim_reg16_read(client, regs[i], &val);
		if (ret < 0)
			dev_err(&client->dev,
				"read fail: chip 0x%x register 0x%02x\n",
				client->addr, regs[i]);
		printk("0x%02x -> 0x%x\n", regs[i], val);
	}
}

static void maxim_max927x_dump_regs(struct i2c_client *client)
{
	int ret;
	u8 reg;

	dev_dbg(&client->dev, "dump regs 0x%x\n", client->addr);

	for (reg = 0; reg < 0x20; reg++) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0)
			dev_err(&client->dev,
				"read fail: chip 0x%x register 0x%x\n",
				client->addr, reg);
		printk("0x%02x ", ret);
		if (((reg + 1) % 0x10) == 0)
			printk("\n");
	}
}
#endif /* MAXIM_DUMP */
