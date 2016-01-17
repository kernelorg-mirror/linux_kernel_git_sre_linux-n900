/*
 *  Nokia H4P driver
 *
 *  Copyright (C) 2015  Sebastian Reichel <sre@kernel.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#define DEBUG

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <net/bluetooth/bluetooth.h>
#include "hci_h4p.h"

static int nokia_h4p_probe(struct platform_device *pdev)
{
	struct h4p_dev_struct *h4p;
	struct device *bcmdev = &pdev->dev;
	struct clk *sysclk;
	int err = 0;

	if(!bcmdev->parent) {
		dev_err(bcmdev, "parent device missing!\n");
		return -ENODEV;
	}

	h4p = devm_kmalloc(bcmdev, sizeof(*h4p), GFP_KERNEL);
	if(!h4p)
		return -ENOMEM;

	h4p->dev = bcmdev;
	dev_set_drvdata(bcmdev, h4p);

	h4p->port = dev_get_drvdata(bcmdev->parent);
	if(!h4p->port) {
		dev_err(bcmdev, "port data missing in parent device!\n");
		return -ENODEV;
	}

	h4p->reset = devm_gpiod_get(bcmdev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(h4p->reset)) {
		err = PTR_ERR(h4p->reset);
		dev_err(bcmdev, "could not get reset gpio: %d\n", err);
		return err;
	}

	h4p->wakeup_host = devm_gpiod_get(bcmdev, "host-wakeup", GPIOD_IN);
	if (IS_ERR(h4p->wakeup_host)) {
		err = PTR_ERR(h4p->wakeup_host);
		dev_err(bcmdev, "could not get host wakeup gpio: %d\n", err);
		return err;
	}


	h4p->wakeup_bt = devm_gpiod_get(bcmdev, "bluetooth-wakeup",
					    GPIOD_OUT_LOW);
	if (IS_ERR(h4p->wakeup_bt)) {
		err = PTR_ERR(h4p->wakeup_bt);
		dev_err(bcmdev, "could not get BT wakeup gpio: %d\n", err);
		return err;
	}

	sysclk = devm_clk_get(bcmdev, "sysclk");
	if (IS_ERR(sysclk)) {
		err = PTR_ERR(sysclk);
		dev_err(bcmdev, "could not get sysclk: %d\n", err);
		return err;
	}

	clk_prepare_enable(sysclk);
	h4p->sysclk_speed = clk_get_rate(sysclk);
	clk_disable_unprepare(sysclk);

	dev_dbg(bcmdev, "parent uart: %s\n", dev_name(bcmdev->parent));
	dev_dbg(bcmdev, "sysclk speed: %ld kHz\n", h4p->sysclk_speed / 1000);

	/* TODO: open tty and setup line disector */

	return err;
}

static const struct of_device_id nokia_h4p_of_match[] = {
	{ .compatible = "nokia,h4p-bluetooth", },
	{},
};
MODULE_DEVICE_TABLE(of, nokia_h4p_of_match);

static struct platform_driver platform_nokia_h4p_driver = {
	.driver = {
		.name = "nokia-h4p-bluetooth",
		.of_match_table = nokia_h4p_of_match,
	},
	.probe = nokia_h4p_probe,
};

module_platform_driver(platform_nokia_h4p_driver);

MODULE_ALIAS("platform:nokia-h4p");
MODULE_AUTHOR("Sebastian Reichel <sre@kernel.org>");
MODULE_DESCRIPTION("Serial Driver for Nokia H4+ bluetooth devices");
MODULE_LICENSE("GPL v2");
