/*
 * NXP SC18IS600 SPI to I2C bus interface driver
 *
 * Copyright (C) 2017 Sebastian Reichel <sre@kernel.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * Datasheets:
 *  - http://www.nxp.com/documents/data_sheet/SC18IS600.pdf
 *  - https://www.silabs.com/documents/public/data-sheets/CP2120.pdf
 */

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#define SC18IS600_I2C_PM_TIMEOUT 1000 /* ms */
#define SC18IS600_DEFAULT_FREQ 100000
#define SC18IS600_CLK_DIVIDER 4

#define SC18IS600_CMD_WR	0x00 /* write */
#define SC18IS600_CMD_RD	0x01 /* read */
#define SC18IS600_CMD_WR_RD	0x02 /* read after write */
#define SC18IS600_CMD_WR_WR	0x03 /* write after write */
#define SC18IS600_CMD_RDBUF	0x06 /* read buffer */
#define CP2120_CMD_WRMULTI	0x09 /* write to multiple slaves */
#define SC18IS600_CMD_SPICON	0x18 /* spi endianess configuration */
#define SC18IS600_CMD_REG_WR	0x20 /* write register */
#define SC18IS600_CMD_REG_RD	0x21 /* read register */
#define SC18IS600_CMD_PWRDWN	0x30 /* power down */
#define CP2120_CMD_REVISION	0x40 /* read revision */

#define SC18IS600_REG_IO_CONFIG		0x00
#define SC18IS600_REG_IO_STATE		0x01
#define SC18IS600_REG_I2C_CLOCK		0x02
#define SC18IS600_REG_I2C_TIMEOUT	0x03
#define SC18IS600_REG_I2C_STAT		0x04
#define SC18IS600_REG_I2C_ADDR		0x05
#define SC18IS600_REG_I2C_BUFFER	0x06 /* only cp2120 */
#define SC18IS600_REG_IO_CONFIG2	0x07 /* only cp2120 */
#define SC18IS600_REG_EDGEINT		0x08 /* only cp2120 */
#define SC18IS600_REG_I2C_TIMEOUT2	0x09 /* only cp2120 */

#define SC18IS600_STAT_OK		0xF0
#define SC18IS600_STAT_NAK_ADDR		0xF1
#define SC18IS600_STAT_NAK_DATA		0xF2
#define SC18IS600_STAT_BUSY		0xF3
#define SC18IS600_STAT_TIMEOUT		0xF8
#define SC18IS600_STAT_SIZE		0xF9
#define SC18IS600_STAT_TIMEOUT2		0xFA /* only cp2120 */
#define SC18IS600_STAT_BLOCKED		0xFB /* only cp2120 */

#define CMD_BUFFER_SIZE 5

enum chiptype {
	SPI2I2C_SC18IS600,
	SPI2I2C_SC18IS601,
	SPI2I2C_CP2120,
};

struct chipdesc {
	u8  type;
	u32 max_spi_speed;
	u32 buffer_size;
	u32 clock_base;
	u32 timeout_base;
	const struct regmap_config *regmap_cfg;
};

static bool sc18is600_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SC18IS600_REG_I2C_STAT:
	case SC18IS600_REG_I2C_BUFFER:
		return false;
	default:
		return true;
	}
}

static const struct regmap_config sc18is600_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0x05,
	.writeable_reg = sc18is600_writeable_reg,
};

static const struct regmap_config cp2120_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0x09,
	.writeable_reg = sc18is600_writeable_reg,
};

/*
 * Note: The sc18is600's datasheet promises 1.2MHz SPI support, but my chip did
 * not behave correctly at that speed. It received the bytes correctly, but
 * just sent them back instead of interpreting them correctly. At 800 KHz I
 * still got a few errors (about 1%) and at 700 KHz everything works smoothly.
 */
static const struct chipdesc chip_sc18is600 = {
	.type = SPI2I2C_SC18IS600,
	.max_spi_speed = 700000,
	.buffer_size = 96,
	.clock_base = 7372800 / SC18IS600_CLK_DIVIDER,
	.timeout_base = 1125, /* 112.5 Hz */
	.regmap_cfg = &sc18is600_regmap_config,
};

static const struct chipdesc chip_sc18is601 = {
	.type = SPI2I2C_SC18IS601,
	.max_spi_speed = 3000000,
	.buffer_size = 96,
	.clock_base = 0,
	.timeout_base = 1125, /* 112.5 Hz */
	.regmap_cfg = &sc18is600_regmap_config,
};

static const struct chipdesc chip_cp2120 = {
	.type = SPI2I2C_CP2120,
	.max_spi_speed = 1000000,
	.buffer_size = 255,
	.clock_base = 2000000,
	.timeout_base = 1280, /* 128 Hz */
	.regmap_cfg = &cp2120_regmap_config,
};

struct sc18is600dev {
	struct i2c_adapter adapter;
	struct completion completion;
	struct spi_device *spi;
	struct regmap *regmap;
	const struct chipdesc *chip;
	struct gpio_desc *reset;
	struct regulator *vdd;
	struct clk *clk;
	u32 clock_base;
	u32 i2c_clock_frequency;
	int state;
};

static irqreturn_t sc18is600_irq_handler(int this_irq, void *data)
{
	struct sc18is600dev *s600dev = data;
	int err;

	err = regmap_read(s600dev->regmap, SC18IS600_REG_I2C_STAT,
			  &s600dev->state);
	if (err)
		return IRQ_NONE;

	dev_vdbg(&s600dev->spi->dev, "irq received, stat=%08x", s600dev->state);

	/* no irq is generated for busy state, so ignore this irq */
	if (s600dev->state == SC18IS600_STAT_BUSY)
		return IRQ_NONE;

	complete(&s600dev->completion);
	return IRQ_HANDLED;
}

static int reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	u8 txbuffer[2] = { SC18IS600_CMD_REG_RD, reg & 0xff };
	u8 rxbuffer[1];
	int err;

	err = spi_write_then_read(spi, txbuffer, sizeof(txbuffer),
				       rxbuffer, sizeof(rxbuffer));
	if (err)
		return err;

	*val = rxbuffer[0];

	return 0;
}

static int reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	u8 txbuffer[3] = { SC18IS600_CMD_REG_WR, reg & 0xff, val & 0xff };

	return spi_write(spi, txbuffer, sizeof(txbuffer));
}

static struct regmap_bus regmap_sc18is600_bus = {
	.reg_write = reg_write,
	.reg_read = reg_read,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

static void sc18is600_setup_clock_frequency(struct sc18is600dev *s600dev)
{
	int reg = DIV_ROUND_UP(s600dev->clock_base,
			       s600dev->i2c_clock_frequency);

	clamp_val(reg, 5, 255);

	dev_dbg(&s600dev->spi->dev, "i2c clock frequency: %08x", reg);
	regmap_write(s600dev->regmap, SC18IS600_REG_I2C_CLOCK, reg);
}

static void sc18is600_setup_timeout(struct sc18is600dev *s600dev,
				    bool enable, int timeout_ms)
{
	int timeout = DIV_ROUND_UP(timeout_ms * s600dev->chip->timeout_base,
				   10000);
	u8 reg;

	clamp_val(timeout, 1, 255);

	reg = ((timeout & 0x7F) << 1) | !!enable;

	dev_dbg(&s600dev->spi->dev, "i2c timeout: %08x", reg);
	regmap_write(s600dev->regmap, SC18IS600_REG_I2C_TIMEOUT, reg);
}

static void sc18is600_reset(struct sc18is600dev *s600dev)
{
	if (s600dev->reset) {
		gpiod_set_value_cansleep(s600dev->reset, 1);
		usleep_range(50, 100);
		gpiod_set_value_cansleep(s600dev->reset, 0);
		usleep_range(50, 100);
	}

	sc18is600_setup_clock_frequency(s600dev);
	sc18is600_setup_timeout(s600dev, true, 500);
}

static int sc18is600_read(struct sc18is600dev *s600dev, struct i2c_msg *msg)
{
	u8 header[] = { SC18IS600_CMD_RD, msg->len, msg->addr << 1 };
	struct spi_transfer xfer[1] = { 0 };

	xfer[0].tx_buf = header;
	xfer[0].len = sizeof(header);

	return spi_sync_transfer(s600dev->spi, xfer, 1);
}

static int sc18is600_write(struct sc18is600dev *s600dev, struct i2c_msg *msg)
{
	u8 header[] = { SC18IS600_CMD_WR, msg->len, msg->addr << 1 };
	struct spi_transfer xfer[2] = { 0 };

	xfer[0].tx_buf = header;
	xfer[0].len = sizeof(header);

	xfer[1].tx_buf = msg->buf;
	xfer[1].len = msg->len;

	return spi_sync_transfer(s600dev->spi, xfer, 2);
}

static int sc18is600_read_write(struct sc18is600dev *s600dev,
				struct i2c_msg *msg1,
				struct i2c_msg *msg2)
{
	u8 header1[] =
		{ SC18IS600_CMD_WR_RD, msg1->len, msg2->len, msg1->addr << 1 };
	u8 header2[] = { msg2->addr << 1 };
	struct spi_transfer xfer[3] = { 0 };

	xfer[0].tx_buf = header1;
	xfer[0].len = sizeof(header1);

	xfer[1].tx_buf = msg1->buf;
	xfer[1].len = msg1->len;

	xfer[2].tx_buf = header2;
	xfer[2].len = sizeof(header2);

	return spi_sync_transfer(s600dev->spi, xfer, 3);
}

static int sc18is600_write_write(struct sc18is600dev *s600dev,
				 struct i2c_msg *msg1,
				 struct i2c_msg *msg2)
{
	u8 header1[] =
		{ SC18IS600_CMD_WR_WR, msg1->len, msg2->len, msg1->addr << 1 };
	u8 header2[] = { msg2->addr << 1 };
	struct spi_transfer xfer[4] = { 0 };

	xfer[0].tx_buf = header1;
	xfer[0].len = sizeof(header1);

	xfer[1].tx_buf = msg1->buf;
	xfer[1].len = msg1->len;

	xfer[2].tx_buf = header2;
	xfer[2].len = sizeof(header2);

	xfer[3].tx_buf = msg2->buf;
	xfer[3].len = msg2->len;

	return spi_sync_transfer(s600dev->spi, xfer, 4);
}

static int sc18is600_read_buffer(struct sc18is600dev *s600dev,
				 struct i2c_msg *msg)
{
	static const u8 read_buffer_cmd = SC18IS600_REG_I2C_BUFFER;

	return spi_write_then_read(s600dev->spi, &read_buffer_cmd, 1,
				   msg->buf, msg->len);
}

static int sc18is600_xfer(struct i2c_adapter *adapter,
			  struct i2c_msg *msgs, int num)
{
	struct sc18is600dev *s600dev = adapter->algo_data;
	int read_operations = 0;
	bool ignore_nak = false;
	int i, err;

	for (i = 0; i < num; i++) {
		if (msgs[i].len > s600dev->chip->buffer_size)
			return -EOPNOTSUPP;

		/* chip only support standard read & write */
		if (msgs[i].flags & ~I2C_M_RD)
			return -EOPNOTSUPP;

		if (msgs[i].flags & I2C_M_RD)
			read_operations++;
	}

	reinit_completion(&s600dev->completion);

	if (num == 1) {
		if (msgs[0].flags & I2C_M_IGNORE_NAK)
			ignore_nak = true;

		if (read_operations == 1)
			err = sc18is600_read(s600dev, &msgs[0]);
		else
			err = sc18is600_write(s600dev, &msgs[0]);
	} else if (num == 2) {
		if (read_operations == 1)
			err = sc18is600_read_write(s600dev, &msgs[0], &msgs[1]);
		else
			err = sc18is600_write_write(s600dev, &msgs[0], &msgs[1]);
	} else {
		return -EOPNOTSUPP;
	}

	if (err) {
		dev_err(&s600dev->spi->dev, "spi transfer failed: %d", err);
		return err;
	}

	err = wait_for_completion_timeout(&s600dev->completion,
					  adapter->timeout);
	if (!err) {
		dev_warn(&s600dev->spi->dev,
			 "timeout waiting for irq, poll status register");
		s600dev->state = SC18IS600_STAT_BUSY;
		regmap_read(s600dev->regmap, SC18IS600_REG_I2C_STAT,
			    &s600dev->state);
	}

	switch (s600dev->state) {
	case SC18IS600_STAT_OK:
		break;
	case SC18IS600_STAT_NAK_ADDR:
		return -ENXIO;
	case SC18IS600_STAT_NAK_DATA:
		if (ignore_nak)
			break;
		return -EREMOTEIO;
	case SC18IS600_STAT_SIZE:
		return -EINVAL;
	case SC18IS600_STAT_TIMEOUT:
	case SC18IS600_STAT_TIMEOUT2:
	case SC18IS600_STAT_BLOCKED:
		return -ETIMEDOUT;
	default:
	case SC18IS600_STAT_BUSY:
		dev_err(&s600dev->spi->dev, "device hangup detected, reset!");
		sc18is600_reset(s600dev);
		return -EAGAIN;
	}

	if (!read_operations)
		return 0;

	err = sc18is600_read_buffer(s600dev, &msgs[num-1]);
	if (err)
		return err;

	return num;
}

static u32 sc18is600_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm sc18is600_algorithm = {
	.master_xfer	= sc18is600_xfer,
	.functionality	= sc18is600_func,
};

static int sc18is600_probe(struct spi_device *spi)
{
	struct sc18is600dev *s600dev;
	int err;

	s600dev = devm_kzalloc(&spi->dev, sizeof(*s600dev), GFP_KERNEL);
	if (!s600dev)
		return -ENOMEM;
	spi_set_drvdata(spi, s600dev);

	init_completion(&s600dev->completion);

	s600dev->spi = spi;
	s600dev->adapter.owner = THIS_MODULE;
	s600dev->adapter.class = I2C_CLASS_DEPRECATED;
	s600dev->adapter.algo = &sc18is600_algorithm;
	s600dev->adapter.algo_data = s600dev;
	s600dev->adapter.dev.parent = &spi->dev;
	s600dev->chip = of_device_get_match_data(&spi->dev);

	if (!s600dev->chip)
		return -ENODEV;

	snprintf(s600dev->adapter.name, sizeof(s600dev->adapter.name),
		 "SC18IS600 at SPI %02d device %02d",
		 spi->master->bus_num, spi->chip_select);

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_3;
	spi->max_speed_hz = s600dev->chip->max_spi_speed;

	err = spi_setup(spi);
	if (err)
		return err;

	s600dev->reset = devm_gpiod_get_optional(&spi->dev, "reset",
						 GPIOD_OUT_LOW);
	if (IS_ERR(s600dev->reset)) {
		err = PTR_ERR(s600dev->reset);
		dev_err(&spi->dev, "Failed to reset gpio, err: %d\n", err);
		return err;
	}

	err = devm_request_threaded_irq(&spi->dev, spi->irq, NULL,
					sc18is600_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"sc18is600", s600dev);
	if (err) {
		dev_err(&spi->dev, "Failed to request irq, err: %d\n", err);
		return err;
	}

	s600dev->regmap = devm_regmap_init(&spi->dev,
			       &regmap_sc18is600_bus, &spi->dev,
			       s600dev->chip->regmap_cfg);
	if (IS_ERR(s600dev->regmap)) {
		err = PTR_ERR(s600dev->regmap);
		dev_err(&spi->dev, "Failed to init regmap, err: %d\n", err);
		return err;
	}

	err = device_property_read_u32(&spi->dev, "clock-frequency",
				       &s600dev->i2c_clock_frequency);
	if (err) {
		s600dev->i2c_clock_frequency = SC18IS600_DEFAULT_FREQ;
		dev_dbg(&spi->dev, "using default frequency %u\n",
			s600dev->i2c_clock_frequency);
	}

	s600dev->vdd = devm_regulator_get(&spi->dev, "vdd");
	if (IS_ERR(s600dev->vdd)) {
		err = PTR_ERR(s600dev->vdd);
		dev_err(&spi->dev, "could not acquire Vdd: %d\n", err);
		return err;
	}

	if (!s600dev->chip->clock_base) {
		s600dev->clk = devm_clk_get(&spi->dev, "clkin");
		if (IS_ERR(s600dev->clk)) {
			err = PTR_ERR(s600dev->clk);
			dev_err(&spi->dev, "could not acquire clk: %d\n", err);
			return err;
		}

		err = clk_prepare_enable(s600dev->clk);
		if (err) {
			dev_err(&spi->dev, "could not enable clk: %d\n", err);
			return err;
		}

		s600dev->clock_base =
			clk_get_rate(s600dev->clk) / SC18IS600_CLK_DIVIDER;
	} else {
		s600dev->clock_base = s600dev->chip->clock_base;
	}

	err = regulator_enable(s600dev->vdd);
	if (err) {
		dev_err(&spi->dev, "could not enable vdd: %d\n", err);
		return err;
	}

	sc18is600_reset(s600dev);

	err = i2c_add_adapter(&s600dev->adapter);
	if (err)
		goto out_disable_regulator;

	return 0;

out_disable_regulator:
	regulator_disable(s600dev->vdd);
	return err;
}

static int sc18is600_remove(struct spi_device *spi)
{
	struct sc18is600dev *s600dev = spi_get_drvdata(spi);

	i2c_del_adapter(&s600dev->adapter);

	regulator_disable(s600dev->vdd);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sc18is600_of_match[] = {
	{ .compatible = "nxp,sc18is600", .data = &chip_sc18is600 },
	{ .compatible = "nxp,sc18is601", .data = &chip_sc18is601 },
	{ .compatible = "silabs,cp2120", .data = &chip_cp2120 },
	{},
};
MODULE_DEVICE_TABLE(of, sc18is600_of_match);
#endif

static struct spi_driver sc18is600_driver = {
	.probe		= sc18is600_probe,
	.remove		= sc18is600_remove,
	.driver		= {
		.name	= "i2c-sc18is600",
		.of_match_table = of_match_ptr(sc18is600_of_match),
	},
};
module_spi_driver(sc18is600_driver);

MODULE_AUTHOR("Sebastian Reichel <sre@kernel.org>");
MODULE_DESCRIPTION("NXP SC18IS600 I2C bus adapter");
MODULE_LICENSE("GPL");
