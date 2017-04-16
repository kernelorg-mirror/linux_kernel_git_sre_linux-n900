/*
 * A iio driver for the light sensor ISL 29030.
 *
 * IIO driver for monitoring ambient light intensity in lux and
 * proximity sensing
 *
 * Copyright (c) 2017, Sebastian Reichel <sre@kernel.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * Datasheet:
 *  * http://www.intersil.com/content/dam/Intersil/documents/isl2/isl29030.pdf
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

enum isl29030_registers {
	ISL29030_REG_CFG = 0x01,
	ISL29030_REG_IRQ,
	ISL29030_REG_PROX_LT,
	ISL29030_REG_PROX_HT,
	ISL29030_REG_ALSIR_TH1,
	ISL29030_REG_ALSIR_TH2,
	ISL29030_REG_ALSIR_TH3,
	ISL29030_REG_PROX_DATA,
	ISL29030_REG_ALSIR_LSB,
	ISL29030_REG_ALSIR_MSB,
	ISL29030_REG_TEST1,
	ISL29030_REG_TEST2,
};

static const u8 isl29030_default_data[] = {
	0x60, /* CFG */
	0x00, /* IRQ */
	0x00, 0xff, /* proximity thresholds */
	0x00, 0xf0, 0xff, /* ALSIR thresholds */
};

/* Bit Masks for ISL29030_REG_CFG */
#define ISL29030_CFG_PROX_EN 0x80
#define ISL29030_CFG_PROX_SLP 0x70
#define ISL29030_CFG_PROX_DR 0x08
#define ISL29030_CFG_ALSIR_EN 0x04
#define ISL29030_CFG_ALSIR_RANGE 0x02
#define ISL29030_CFG_ALSIR_MODE 0x01

/* Bit Masks for ISL29030_REG_IRQ */
#define ISL29030_IRQ_PROX_FLAG 0x80
#define ISL29030_IRQ_PROX_PRST 0x60
#define ISL29030_IRQ_ALSIR_FLAG 0x08
#define ISL29030_IRQ_ALSIR_PRST 0x06
#define ISL29030_IRQ_INT_CTRL 0x01

#define ISL29030_SENSING_RANGE_0 32600
#define ISL29030_SENSING_RANGE_1 522000

struct isl29030_chip {
	struct regmap		*regmap;
	struct regulator	*vdd;
	struct device		*dev;
	struct mutex		lock;
	int			alsir_enabled;
	int			prox_enabled;
	bool			alsir_mode;
};

static const struct regmap_range isl29030_volatile_regs_ranges[] = {
	/* ISL29030_IRQ_PROX_FLAG & ISL29030_IRQ_ALS_FLAG */
	regmap_reg_range(ISL29030_REG_IRQ, ISL29030_REG_IRQ),
	/* adc data registers */
	regmap_reg_range(ISL29030_REG_PROX_DATA, ISL29030_REG_ALSIR_MSB),
};

static const struct regmap_access_table isl29030_volatile_regs = {
	.yes_ranges = isl29030_volatile_regs_ranges,
	.n_yes_ranges = ARRAY_SIZE(isl29030_volatile_regs_ranges),
};

static const struct regmap_config isl29030_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = ISL29030_REG_TEST2,
	.cache_type = REGCACHE_RBTREE,
	.volatile_table = &isl29030_volatile_regs,
};

struct iio_event_spec threshold_event = {
	.type = IIO_EV_TYPE_THRESH,
	.dir  = IIO_EV_DIR_EITHER,
	.mask_separate = BIT(IIO_EV_INFO_ENABLE) | BIT(IIO_EV_INFO_VALUE),
};

static const struct iio_chan_spec isl29030_channels[] = {
	{
		.type = IIO_LIGHT,
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.event_spec = &threshold_event,
		.num_event_specs = 1,
	},
	{
		.type = IIO_INTENSITY,
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
		},
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_IR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.event_spec = &threshold_event,
		.num_event_specs = 1,
	},
	{
		.type = IIO_PROXIMITY,
		.scan_type = {
			.sign = 'u',
			.realbits = 8,
			.storagebits = 8,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.event_spec = &threshold_event,
		.num_event_specs = 1,
	}
};

static IIO_CONST_ATTR(scale_available, "0.0326 0.522");

static struct attribute *isl29030_attributes[] = {
	&iio_const_attr_scale_available.dev_attr.attr,
	NULL
};

static const struct attribute_group isl29030_group = {
	.attrs = isl29030_attributes,
};

int isl29030_set_alsir_mode(struct isl29030_chip *chip, bool ir)
{
	int err;

	err = regmap_update_bits(chip->regmap, ISL29030_REG_CFG,
				 ISL29030_CFG_ALSIR_MODE, ir);
	if (err < 0)
		return err;

	chip->alsir_mode = !!ir;

	return 0;
}

int isl29030_enable_alsir(struct isl29030_chip *chip, bool irmode, bool enable)
{
	int err;

	/* setup mode */
	if (chip->alsir_mode != irmode) {
		if (!chip->alsir_enabled)
			err = isl29030_set_alsir_mode(chip, irmode);
		else
			err = -EBUSY;
		if (err)
			return err;
	}

	/* power device */
	if (enable) {
		err = pm_runtime_get_sync(chip->dev);
		if (err) {
			dev_err(chip->dev, "runtime pm failure: %d", err);
			pm_runtime_put(chip->dev);
			return err;
		}
	} else {
		pm_runtime_put(chip->dev);
	}

	if (enable)
		chip->alsir_enabled++;
	else
		chip->alsir_enabled--;

	err = regmap_update_bits(chip->regmap, ISL29030_REG_CFG,
				 ISL29030_CFG_ALSIR_EN, !!chip->alsir_enabled);
	if (err < 0)
		return err;

	/* ALSIR is updated every 100ms */
	if (chip->alsir_enabled == 1)
		msleep(100);

	return 0;
}

int isl29030_enable_prox(struct isl29030_chip *chip, bool enable)
{
	int err;

	if (enable)
		chip->prox_enabled++;
	else
		chip->prox_enabled--;

	/* power device */
	if (enable) {
		err = pm_runtime_get_sync(chip->dev);
		if (err) {
			dev_err(chip->dev, "runtime pm failure: %d", err);
			pm_runtime_put(chip->dev);
			return err;
		}
	} else {
		pm_runtime_put(chip->dev);
	}

	err = regmap_update_bits(chip->regmap, ISL29030_REG_CFG,
				 ISL29030_CFG_PROX_EN, !!chip->prox_enabled);
	if (err < 0)
		return err;

	/* duration is PROX_SLP, driver currently only supports 800ms */
	if (chip->prox_enabled == 1)
		msleep(800);

	return 0;
}

int isl29030_read_alsir_reg(struct isl29030_chip *chip, int *val)
{
	int err;
	unsigned int lsb;
	unsigned int msb;

	err = regmap_read(chip->regmap, ISL29030_REG_ALSIR_LSB, &lsb);
	if (err < 0) {
		dev_err(chip->dev, "Couldn't read ALSIR LSB with err %d", err);
		return err;
	}

	err = regmap_read(chip->regmap, ISL29030_REG_ALSIR_MSB, &msb);
	if (err < 0) {
		dev_err(chip->dev, "Couldn't read ALSIR MSB with err %d", err);
		return err;
	}

	*val = (msb << 8) | lsb;
	dev_vdbg(chip->dev, "ALSIR 0x%04x", *val);

	return 0;
}

int isl29030_read_alsir(struct isl29030_chip *chip, bool ir, int *val)
{
	int err;

	err = isl29030_enable_alsir(chip, ir, true);
	if (err)
		return err;

	err = isl29030_read_alsir_reg(chip, val);
	if (err)
		return err;

	err = isl29030_enable_alsir(chip, ir, false);
	if (err)
		return err;

	return IIO_VAL_INT;
}

int isl29030_read_proximity(struct isl29030_chip *chip, int *val)
{
	int err;

	err = isl29030_enable_prox(chip, true);
	if (err)
		return err;

	err = regmap_read(chip->regmap, ISL29030_REG_PROX_DATA, val);
	if (err < 0) {
		dev_err(chip->dev, "PROX DATA read failed with error %d", err);
		return err;
	}

	err = isl29030_enable_prox(chip, false);
	if (err)
		return err;

	return IIO_VAL_INT;
}

static int isl29030_read_als_range(struct isl29030_chip *chip,
				   int *val, int *val2) {
	int err = regmap_read(chip->regmap, ISL29030_REG_CFG, val);

	if (err < 0) {
		dev_err(chip->dev, "ALS range read failed with error %d", err);
		return err;
	}

	*val = 0;

	if (*val & ISL29030_CFG_ALSIR_RANGE)
		*val2 = ISL29030_SENSING_RANGE_1;
	else
		*val2 = ISL29030_SENSING_RANGE_0;

	return IIO_VAL_INT_PLUS_MICRO;
}

static int isl29030_write_als_range(struct isl29030_chip *chip, int val)
{
	int err;

	switch (val) {
	case ISL29030_SENSING_RANGE_0:
		val = 0;
		break;
	case ISL29030_SENSING_RANGE_1:
		val = 1;
		break;
	default:
		return -EINVAL;
	}

	err = regmap_update_bits(chip->regmap, ISL29030_REG_CFG,
				 ISL29030_CFG_ALSIR_RANGE, val);
	if (err < 0) {
		dev_err(chip->dev, "ALS range write failed with error %d", err);
		return err;
	}

	return 0;
}

static int isl29030_write_prox_threshold(struct isl29030_chip *chip,
					 bool falling, int val)
{
	int err = regmap_write(chip->regmap, falling ? ISL29030_REG_PROX_LT :
			       ISL29030_REG_PROX_HT, val);
	if (err < 0)
		dev_err(chip->dev, "PROX threshold write failed with error %d",
			err);
	return err;
}

static int isl29030_read_prox_threshold(struct isl29030_chip *chip,
					bool falling, int *val)
{
	int err = regmap_read(chip->regmap, falling ? ISL29030_REG_PROX_LT :
			       ISL29030_REG_PROX_HT, val);
	if (err < 0)
		dev_err(chip->dev, "PROX threshold read failed with error %d",
			err);
	return err;
}

static int isl29030_write_alsir_threshold(struct isl29030_chip *chip,
					  bool falling, int val)
{
	if (falling) {
		int err;
		u8 lsb = (val & 0x00FF);
		u8 msb = (val & 0x0F00) >> 8;

		err = regmap_write(chip->regmap, ISL29030_REG_ALSIR_TH1, lsb);
		if (err < 0) {
			dev_err(chip->dev, "ALSIR_TH1 write failed: %d", err);
			return err;
		}

		err = regmap_update_bits(chip->regmap, ISL29030_REG_ALSIR_TH2,
					 0x0F, msb);
		if (err < 0) {
			dev_err(chip->dev, "ALSIR_TH2 update failed: %d", err);
			return err;
		}
	} else {
		int err;
		u8 lsb = (val & 0x000F) << 4;
		u8 msb = (val & 0x0FF0) >> 4;

		err = regmap_update_bits(chip->regmap, ISL29030_REG_ALSIR_TH2,
					 0xF0, lsb);
		if (err < 0) {
			dev_err(chip->dev, "ALSIR_TH2 update failed: %d", err);
			return err;
		}

		err = regmap_write(chip->regmap, ISL29030_REG_ALSIR_TH3, msb);
		if (err < 0) {
			dev_err(chip->dev, "ALSIR_TH3 write failed: %d", err);
			return err;
		}
	}

	return 0;
}

static int isl29030_read_alsir_threshold(struct isl29030_chip *chip,
					 bool falling, int *val)
{
	u8 regs[3];
	int err;

	err = regmap_bulk_read(chip->regmap, ISL29030_REG_ALSIR_TH1,
			       regs, sizeof(regs));
	if (err) {
		dev_err(chip->dev, "ALSIR threshold read failed: %d", err);
		return err;
	}

	if (falling)
		*val = (regs[1] & 0x0F) << 8 | regs[0];
	else
		*val = (regs[2] & 0xFF) << 4 | (regs[1] & 0xF0) >> 4;

	return 0;
}

static int isl29030_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct isl29030_chip *chip = iio_priv(indio_dev);
	int ret = -EINVAL;

	mutex_lock(&chip->lock);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_LIGHT:
			ret = isl29030_read_alsir(chip, false, val);
			break;
		case IIO_INTENSITY:
			ret = isl29030_read_alsir(chip, true, val);
			break;
		case IIO_PROXIMITY:
			ret = isl29030_read_proximity(chip, val);
			break;
		default:
			break;
		}
		break;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_LIGHT:
			ret = isl29030_read_als_range(chip, val, val2);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	mutex_unlock(&chip->lock);

	return ret;
}

static int isl29030_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long mask)
{
	struct isl29030_chip *chip = iio_priv(indio_dev);
	int ret = -EINVAL;

	if (mask == IIO_CHAN_INFO_SCALE && chan->type == IIO_LIGHT)
		ret = isl29030_write_als_range(chip, val2);

	return ret;
}

static int isl29030_read_event_thresh(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir,
				      enum iio_event_info info,
				      int *val, int *val2)
{
	struct isl29030_chip *chip = iio_priv(indio_dev);
	bool rising = (dir == IIO_EV_DIR_RISING);
	int err = -EINVAL;

	mutex_lock(&chip->lock);
	switch (chan->type) {
	case IIO_PROXIMITY:
		err = isl29030_read_prox_threshold(chip, !rising, val);
		break;
	case IIO_LIGHT:
	case IIO_INTENSITY:
		err = isl29030_read_alsir_threshold(chip, !rising, val);
		break;
	default:
		break;
	}
	mutex_unlock(&chip->lock);

	return err;
}

static int isl29030_write_event_thresh(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info,
				    int val, int val2)
{
	struct isl29030_chip *chip = iio_priv(indio_dev);
	bool rising = (dir == IIO_EV_DIR_RISING);
	int err = -EINVAL;

	mutex_lock(&chip->lock);
	switch (chan->type) {
	case IIO_PROXIMITY:
		err = isl29030_write_prox_threshold(chip, !rising, val);
		break;
	case IIO_LIGHT:
	case IIO_INTENSITY:
		err = isl29030_write_alsir_threshold(chip, !rising, val);
		break;
	default:
		break;
	}
	mutex_unlock(&chip->lock);

	return err;
}

static int isl29030_read_event_config(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir)
{
	struct isl29030_chip *chip = iio_priv(indio_dev);
	int err = -EINVAL;

	mutex_lock(&chip->lock);

	switch (chan->type) {
	case IIO_PROXIMITY:
		err = !!chip->prox_enabled;
		break;
	case IIO_LIGHT:
		err = (chip->alsir_enabled && !chip->alsir_mode);
		break;
	case IIO_INTENSITY:
		err = (chip->alsir_enabled && chip->alsir_mode);
		break;
	default:
		break;
	}

	mutex_unlock(&chip->lock);

	return err;
}

static int isl29030_write_event_config(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan,
				       enum iio_event_type type,
				       enum iio_event_direction dir,
				       int state)
{
	struct isl29030_chip *chip = iio_priv(indio_dev);
	int err = -EINVAL;

	mutex_lock(&chip->lock);

	switch (chan->type) {
	case IIO_PROXIMITY:
		err = isl29030_enable_prox(chip, !!state);
		break;
	case IIO_LIGHT:
		err = isl29030_enable_alsir(chip, 0, !!state);
		break;
	case IIO_INTENSITY:
		err = isl29030_enable_alsir(chip, 1, !!state);
		break;
	default:
		break;
	}

	mutex_unlock(&chip->lock);

	return err;
}

static const struct iio_info isl29030_info = {
	.attrs = &isl29030_group,
	.driver_module = THIS_MODULE,
	.read_raw = isl29030_read_raw,
	.write_raw = isl29030_write_raw,
	.read_event_value = &isl29030_read_event_thresh,
	.write_event_value = &isl29030_write_event_thresh,
	.read_event_config = &isl29030_read_event_config,
	.write_event_config = &isl29030_write_event_config,
};

static irqreturn_t isl29030_interrupt_handler(int irq, void *private)
{
	struct iio_dev *dev_info = private;
	struct isl29030_chip *chip = iio_priv(dev_info);
	int irqreg, err;

	err = regmap_read(chip->regmap, ISL29030_REG_IRQ, &irqreg);
	if (err) {
		dev_err(chip->dev, "failed to read irq register: %d", err);
		return err;
	}

	if (irqreg & ISL29030_IRQ_PROX_FLAG) {
		iio_push_event(dev_info,
			       IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY, 0,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_EITHER),
			       iio_get_time_ns(dev_info));
	}

	if (irqreg & ISL29030_IRQ_ALSIR_FLAG) {
		u8 channel = (chip->alsir_mode) ? IIO_INTENSITY : IIO_LIGHT;

		iio_push_event(dev_info,
			       IIO_UNMOD_EVENT_CODE(channel, 0,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_EITHER),
			       iio_get_time_ns(dev_info));
	}

	err = regmap_update_bits(chip->regmap, ISL29030_REG_IRQ,
				 ISL29030_IRQ_PROX_FLAG |
				 ISL29030_IRQ_ALSIR_FLAG,
				 0x00);
	if (err) {
		dev_err(chip->dev, "failed to clear irq register: %d", err);
		return err;
	}

	return IRQ_HANDLED;
}

static int isl29030_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct isl29030_chip *chip;
	struct iio_dev *indio_dev;
	int err, ledcurrent = 110000;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*chip));
	if (!indio_dev)
		return -ENOMEM;

	chip = iio_priv(indio_dev);
	chip->dev = &client->dev;
	i2c_set_clientdata(client, indio_dev);

	mutex_init(&chip->lock);

	chip->vdd = devm_regulator_get(chip->dev, "vdd");
	if (IS_ERR(chip->vdd)) {
		err = PTR_ERR(chip->vdd);
		dev_err(chip->dev, "Could not acquire vdd: %d", err);
		return err;
	}

	pm_runtime_enable(chip->dev);
	device_set_wakeup_capable(chip->dev, 1);

	chip->regmap = devm_regmap_init_i2c(client, &isl29030_regmap_config);
	if (IS_ERR(chip->regmap)) {
		err = PTR_ERR(chip->regmap);
		dev_err(chip->dev, "regmap init failed: %d", err);
		return err;
	}

	err = regmap_bulk_write(chip->regmap, ISL29030_REG_CFG,
				isl29030_default_data,
				sizeof(isl29030_default_data));
	if (err) {
		dev_err(chip->dev, "failed to reset chip config: %d", err);
		return err;
	}

	err = device_property_read_u32(chip->dev, "", &ledcurrent);
	if (!err && ledcurrent == 220000) {
		err = regmap_update_bits(chip->regmap, ISL29030_REG_CFG,
				   ISL29030_CFG_PROX_DR, ISL29030_CFG_PROX_DR);
		if (err) {
			dev_err(chip->dev, "failed to setup current: %d", err);
			return err;
		}
	}

	indio_dev->info = &isl29030_info;
	indio_dev->channels = isl29030_channels;
	indio_dev->num_channels = ARRAY_SIZE(isl29030_channels);
	indio_dev->name = "isl29030";
	indio_dev->dev.parent = &client->dev;
	indio_dev->modes = INDIO_DIRECT_MODE;

	err = devm_iio_device_register(&client->dev, indio_dev);
	if (err)
		return err;

	err = devm_request_threaded_irq(chip->dev, client->irq, NULL,
					isl29030_interrupt_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"isl29030-event", indio_dev);
	if (err) {
		dev_err(chip->dev, "failed to request irq: %d", err);
		return err;
	}

	return 0;
}

static int isl29030_remove(struct i2c_client *client)
{
	struct iio_dev *dev_info = i2c_get_clientdata(client);
	struct isl29030_chip *chip = iio_priv(dev_info);

	pm_runtime_disable(chip->dev);

	return 0;
}

static int isl29030_rpm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *dev_info = i2c_get_clientdata(client);
	struct isl29030_chip *chip = iio_priv(dev_info);
	int err;

	regcache_cache_only(chip->regmap, true);

	err = regulator_disable(chip->vdd);

	return err;
}

static int isl29030_rpm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *dev_info = i2c_get_clientdata(client);
	struct isl29030_chip *chip = iio_priv(dev_info);
	int err;

	err = regulator_enable(chip->vdd);
	if (err)
		return err;

	regcache_cache_only(chip->regmap, false);
	regcache_mark_dirty(chip->regmap);

	err = regcache_sync(chip->regmap);
	if (err)
		regulator_disable(chip->vdd);

	return err;
}

static int isl29030_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *dev_info = i2c_get_clientdata(client);
	struct isl29030_chip *chip = iio_priv(dev_info);

	if (device_may_wakeup(dev)) {
		enable_irq_wake(client->irq);
	} else {
		mutex_lock(&chip->lock);

		regmap_update_bits(chip->regmap, ISL29030_REG_CFG,
				   ISL29030_CFG_ALSIR_EN | ISL29030_CFG_PROX_EN,
				   0x00);

		mutex_unlock(&chip->lock);
	}

	return 0;
}

static int isl29030_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *dev_info = i2c_get_clientdata(client);
	struct isl29030_chip *chip = iio_priv(dev_info);
	u8 state = 0x00;

	if (device_may_wakeup(dev)) {
		disable_irq_wake(client->irq);
	} else {
		mutex_lock(&chip->lock);

		state |= chip->alsir_enabled ? ISL29030_CFG_ALSIR_EN : 0;
		state |= chip->prox_enabled ? ISL29030_CFG_PROX_EN : 0;
		regmap_update_bits(chip->regmap, ISL29030_REG_CFG,
				   ISL29030_CFG_ALSIR_EN | ISL29030_CFG_PROX_EN,
				   state);

		mutex_unlock(&chip->lock);
	}

	return 0;
}

static const struct dev_pm_ops isl29030_pm_ops = {
	SET_RUNTIME_PM_OPS(isl29030_rpm_suspend, isl29030_rpm_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(isl29030_pm_suspend, isl29030_pm_resume)
};

static const struct i2c_device_id isl29030_id[] = {
	{"isl29030", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, isl29030_id);

static const struct of_device_id isl29030_of_match[] = {
	{ .compatible = "isil,isl29030", },
	{ },
};
MODULE_DEVICE_TABLE(of, isl29030_of_match);

static struct i2c_driver isl29030_driver = {
	.driver	 = {
			.name = "isl29030",
			.pm		= &isl29030_pm_ops,
			.of_match_table = isl29030_of_match,
		    },
	.probe = isl29030_probe,
	.remove = isl29030_remove,
	.id_table = isl29030_id,
};
module_i2c_driver(isl29030_driver);

MODULE_DESCRIPTION("ISL29030 Ambient Light Sensor driver");
MODULE_LICENSE("GPL");
