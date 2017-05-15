/*
 * Maxim MAX9260 GMSL Deserializer Driver
 *
 * Copyright (C) 2017 Ulrich Hecht
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <linux/tty.h>

struct max9260_device {
	struct serdev_device *serdev;
	u8 data;
	int data_ready;
	int gpio_oe;
	int gpio_s[2];
	wait_queue_head_t rx_wq;
};

static int max9260_read_reg(struct max9260_device *dev, int reg)
{
	u8 request[] = { 0x79, 0x91, reg, 1 };

	dev->data_ready = 0;
	serdev_device_write_buf(dev->serdev, request, 4);
	wait_event_interruptible_timeout(dev->rx_wq, dev->data_ready, HZ/2);

	return dev->data_ready ? dev->data : -1;
}

/* XXX: serdev and ttyport lack a way to set the parity. */
struct serport {
	struct tty_port *port;
	struct tty_struct *tty;
	struct tty_driver *tty_drv;
	int tty_idx;
	unsigned long flags;
};

static void ttyport_set_parity_even(struct serdev_controller *ctrl)
{
	struct serport *serport = serdev_controller_get_drvdata(ctrl);
	printk("==sp2a port %p tty %p tty_drv %p tty_idx %d flags %ld\n",
		serport->port, serport->tty, serport->tty_drv, serport->tty_idx,
		serport->flags);
	struct tty_struct *tty = serport->tty;
	struct ktermios ktermios = tty->termios;

	ktermios.c_cflag |= PARENB;
	ktermios.c_cflag &= ~PARODD;

	tty_set_termios(tty, &ktermios);
}

static int max9260_setup(struct max9260_device *dev)
{
	printk("==s1 c %p\n", dev->serdev->ctrl);

	serdev_device_set_baudrate(dev->serdev, 115200);
	ttyport_set_parity_even(dev->serdev->ctrl);

	if (max9260_read_reg(dev, 0x1e) != 0x02) {
		dev_err(&dev->serdev->dev, "device does not identify as MAX9260\n");
		return -EINVAL;
	}

	return 0;
}

static void max9260_uart_write_wakeup(struct serdev_device *serdev)
{
	printk("==muww %p\n", serdev);
}

static int max9260_uart_receive_buf(struct serdev_device *serdev, const u8 *data,
				   size_t count)
{
	struct max9260_device *dev = serdev_device_get_drvdata(serdev);
	int i;

	printk("==murb %p %p %ld\n", serdev, data, count);
	for (i = 0; i < count; ++i) {
		printk("%02X\n", data[i]);
	}
	
	if (count > 1 && data[0] == 0xc3) {
		dev->data = data[1];
		dev->data_ready = 1;
		wake_up_interruptible(&dev->rx_wq);
	}
		
	return 0;
}

struct serdev_device_ops max9260_serdev_client_ops = {
	.receive_buf = max9260_uart_receive_buf,
	.write_wakeup = max9260_uart_write_wakeup,
};

static int max9260_probe(struct serdev_device *serdev)
{
	struct device_node *np;
	struct max9260_device *dev;
	int ret, i;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* XXX: Handle multiplexing here because it's not supported by
	 * serdev (yet).
	 */
	np = serdev->dev.of_node;
	dev->gpio_oe = of_get_named_gpio(np, "mux-enable-gpio", 0);
	gpio_request_one(dev->gpio_oe, GPIOF_DIR_OUT | GPIOF_INIT_LOW, "mux-enable-gpio");
	for (i = 0; i < 2; ++i) {
		dev->gpio_s[i] = of_get_named_gpio(np, "mux-select-gpios", i);
		gpio_request_one(dev->gpio_s[i], GPIOF_DIR_OUT | GPIOF_INIT_LOW, "mux-select-gpio");
	}
	printk("gpio oe %d s %d/%d\n", dev->gpio_oe, dev->gpio_s[0], dev->gpio_s[1]);

	init_waitqueue_head(&dev->rx_wq);

	dev->serdev = serdev;
	serdev_device_open(serdev);
	serdev_device_set_drvdata(serdev, dev);

	serdev_device_set_client_ops(serdev, &max9260_serdev_client_ops);
	
	ret = max9260_setup(dev);

	gpio_free(dev->gpio_oe);
	for (i = 0; i < 2; ++i)
		gpio_free(dev->gpio_s[i]);

	if (ret < 0)
		goto err_free;
	printk("==ok\n");

	return 0;

err_free:
	printk("==crap\n");
	kfree(dev);
	return ret;
}

static void max9260_remove(struct serdev_device *serdev)
{
	struct max9260_device *dev = serdev_device_get_drvdata(serdev);

	serdev_device_close(dev->serdev);

	kfree(dev);
}

static const struct of_device_id max9260_dt_ids[] = {
	{ .compatible = "maxim,max9260" },
	{},
};

MODULE_DEVICE_TABLE(of, max9260_dt_ids);

static struct serdev_device_driver max9260_driver = {
	.probe = max9260_probe,
	.remove = max9260_remove,
	.driver = {
		.name = "max9260",
		.of_match_table = of_match_ptr(max9260_dt_ids),
	},
};

module_serdev_device_driver(max9260_driver);

MODULE_DESCRIPTION("Maxim MAX9260 GMSL Deserializer Driver");
MODULE_AUTHOR("Ulrich Hecht");
MODULE_LICENSE("GPL");
