/*
 * TI LMU (Lighting Management Unit) Backlight Driver
 *
 * Copyright 2015 Texas Instruments
 *
 * Author: Milo Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/backlight.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mfd/ti-lmu.h>
#include <linux/mfd/ti-lmu-register.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>

#include "ti-lmu-backlight-data.h"

enum ti_lmu_bl_ctrl_mode {
	BL_REGISTER_BASED,
	BL_PWM_BASED,
};

enum ti_lmu_bl_ramp_mode {
	BL_RAMP_UP,
	BL_RAMP_DOWN,
};

struct ti_lmu_bl;

/**
 * struct ti_lmu_bl_chip
 *
 * @dev:		Parent device pointer
 * @lmu:		LMU structure.
 *			Used for register R/W access and notification.
 * @cfg:		Device configuration data
 * @lmu_bl:		Multiple backlight channels
 * @num_backlights:	Number of backlight channels
 * @nb:			Notifier block for handling LMU fault monitor event
 *
 * One backlight chip can have multiple backlight channels, 'ti_lmu_bl'.
 */
struct ti_lmu_bl_chip {
	struct device *dev;
	struct ti_lmu *lmu;
	const struct ti_lmu_bl_cfg *cfg;
	struct ti_lmu_bl *lmu_bl;
	int num_backlights;
	struct notifier_block nb;
};

/**
 * struct ti_lmu_bl
 *
 * @chip:		Pointer to parent backlight device
 * @bl_dev:		Backlight subsystem device structure
 * @bank_id:		Backlight bank ID
 * @name:		Backlight channel name
 * @mode:		Backlight control mode
 * @led_sources:	Backlight output channel configuration.
 *			Bit mask is set on parsing DT.
 * @default_brightness:	[Optional] Initial brightness value
 * @ramp_up_msec:	[Optional] Ramp up time
 * @ramp_down_msec:	[Optional] Ramp down time
 * @pwm_period:		[Optional] PWM period
 * @pwm:		[Optional] PWM subsystem structure
 *
 * Each backlight device has its own channel configuration.
 * For chip control, parent chip data structure is used.
 */
struct ti_lmu_bl {
	struct ti_lmu_bl_chip *chip;
	struct backlight_device *bl_dev;

	int bank_id;
	const char *name;
	enum ti_lmu_bl_ctrl_mode mode;
	unsigned long led_sources;

	unsigned int default_brightness;

	/* Used for lighting effect */
	unsigned int ramp_up_msec;
	unsigned int ramp_down_msec;

	/* Only valid in PWM mode */
	unsigned int pwm_period;
	struct pwm_device *pwm;
};

#define NUM_DUAL_CHANNEL			2
#define LMU_BACKLIGHT_DUAL_CHANNEL_USED		(BIT(0) | BIT(1))
#define LMU_BACKLIGHT_11BIT_LSB_MASK		(BIT(0) | BIT(1) | BIT(2))
#define LMU_BACKLIGHT_11BIT_MSB_SHIFT		3
#define DEFAULT_PWM_NAME			"lmu-backlight"

static int ti_lmu_backlight_enable(struct ti_lmu_bl *lmu_bl, bool enable)
{
	struct ti_lmu_bl_chip *chip = lmu_bl->chip;
	struct regmap *regmap = chip->lmu->regmap;
	unsigned long enable_time = chip->cfg->reginfo->enable_usec;
	u8 *reg = chip->cfg->reginfo->enable;
	u8 mask = BIT(lmu_bl->bank_id);
	u8 val = (enable == true) ? mask : 0;
	int ret;

	if (!reg)
		return -EINVAL;

	ret = regmap_update_bits(regmap, *reg, mask, val);
	if (ret)
		return ret;

	if (enable_time > 0)
		usleep_range(enable_time, enable_time + 100);

	return 0;
}

static int ti_lmu_backlight_pwm_ctrl(struct ti_lmu_bl *lmu_bl, int brightness,
				      int max_brightness)
{
	struct pwm_state state = { };
	int ret;

	if (!lmu_bl->pwm) {
		lmu_bl->pwm = devm_pwm_get(lmu_bl->chip->dev, DEFAULT_PWM_NAME);
		if (IS_ERR(lmu_bl->pwm)) {
			ret = PTR_ERR(lmu_bl->pwm);
			lmu_bl->pwm = NULL;
			dev_err(lmu_bl->chip->dev,
				"Can not get PWM device, err: %d\n", ret);
			return ret;
		}
	}

	pwm_init_state(lmu_bl->pwm, &state);
	state.period = lmu_bl->pwm_period;
	state.duty_cycle = brightness * state.period / max_brightness;

	if (state.duty_cycle)
		state.enabled = true;
	else
		state.enabled = false;

	ret = pwm_apply_state(lmu_bl->pwm, &state);
	if (ret)
		dev_err(lmu_bl->chip->dev, "Failed to configure PWM: %d", ret);

	return ret;
}

static int ti_lmu_backlight_update_brightness_register(struct ti_lmu_bl *lmu_bl,
						       int brightness)
{
	const struct ti_lmu_bl_cfg *cfg = lmu_bl->chip->cfg;
	const struct ti_lmu_bl_reg *reginfo = cfg->reginfo;
	struct regmap *regmap = lmu_bl->chip->lmu->regmap;
	u8 reg, val;
	int ret;

	/*
	 * Brightness register update
	 *
	 * 11 bit dimming: update LSB bits and write MSB byte.
	 *		   MSB brightness should be shifted.
	 *  8 bit dimming: write MSB byte.
	 */
	if (cfg->max_brightness == MAX_BRIGHTNESS_11BIT) {
		reg = reginfo->brightness_lsb[lmu_bl->bank_id];
		ret = regmap_update_bits(regmap, reg,
					 LMU_BACKLIGHT_11BIT_LSB_MASK,
					 brightness);
		if (ret)
			return ret;

		val = brightness >> LMU_BACKLIGHT_11BIT_MSB_SHIFT;
	} else {
		val = brightness;
	}

	reg = reginfo->brightness_msb[lmu_bl->bank_id];
	return regmap_write(regmap, reg, val);
}

static int ti_lmu_backlight_update_status(struct backlight_device *bl_dev)
{
	struct ti_lmu_bl *lmu_bl = bl_get_data(bl_dev);
	const struct ti_lmu_bl_cfg *cfg = lmu_bl->chip->cfg;
	int brightness = bl_dev->props.brightness;
	bool enable = brightness > 0;
	int ret;

	if (bl_dev->props.state & BL_CORE_SUSPENDED)
		brightness = 0;

	ret = ti_lmu_backlight_enable(lmu_bl, enable);
	if (ret)
		return ret;

	if (lmu_bl->mode == BL_PWM_BASED) {
		ti_lmu_backlight_pwm_ctrl(lmu_bl, brightness,
					  bl_dev->props.max_brightness);

		switch (cfg->pwm_action) {
		case UPDATE_PWM_ONLY:
			/* No register update is required */
			return 0;
		case UPDATE_MAX_BRT:
			/*
			 * PWM can start from any non-zero code and dim down
			 * to zero. So, brightness register should be updated
			 * even in PWM mode.
			 */
			if (brightness > 0)
				brightness = MAX_BRIGHTNESS_11BIT;
			else
				brightness = 0;
			break;
		default:
			break;
		}
	}

	return ti_lmu_backlight_update_brightness_register(lmu_bl, brightness);
}

static const struct backlight_ops lmu_backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = ti_lmu_backlight_update_status,
};

static int ti_lmu_backlight_of_get_ctrl_bank(struct device_node *np,
					     struct ti_lmu_bl *lmu_bl)
{
	const char *name;
	u32 *sources;
	int num_channels = lmu_bl->chip->cfg->num_channels;
	int ret, num_sources;

	sources = devm_kzalloc(lmu_bl->chip->dev, num_channels, GFP_KERNEL);
	if (!sources)
		return -ENOMEM;

	if (!of_property_read_string(np, "label", &name))
		lmu_bl->name = name;
	else
		lmu_bl->name = np->name;

	ret = of_property_count_u32_elems(np, "led-sources");
	if (ret < 0 || ret > num_channels)
		return -EINVAL;

	num_sources = ret;
	ret = of_property_read_u32_array(np, "led-sources", sources,
					 num_sources);
	if (ret)
		return ret;

	lmu_bl->led_sources = 0;
	while (num_sources--)
		set_bit(sources[num_sources], &lmu_bl->led_sources);

	return 0;
}

static void ti_lmu_backlight_of_get_light_properties(struct device_node *np,
						     struct ti_lmu_bl *lmu_bl)
{
	of_property_read_u32(np, "default-brightness-level",
			     &lmu_bl->default_brightness);

	of_property_read_u32(np, "ramp-up-msec",  &lmu_bl->ramp_up_msec);
	of_property_read_u32(np, "ramp-down-msec", &lmu_bl->ramp_down_msec);
}

static void ti_lmu_backlight_of_get_brightness_mode(struct device_node *np,
						    struct ti_lmu_bl *lmu_bl)
{
	of_property_read_u32(np, "pwm-period", &lmu_bl->pwm_period);

	if (lmu_bl->pwm_period > 0)
		lmu_bl->mode = BL_PWM_BASED;
	else
		lmu_bl->mode = BL_REGISTER_BASED;
}

static int ti_lmu_backlight_of_create(struct ti_lmu_bl_chip *chip,
				      struct device_node *np)
{
	struct device_node *child;
	struct ti_lmu_bl *lmu_bl, *each;
	int ret, num_backlights;
	int i = 0;

	num_backlights = of_get_child_count(np);
	if (num_backlights == 0) {
		dev_err(chip->dev, "No backlight strings\n");
		return -ENODEV;
	}

	/* One chip can have mulitple backlight strings */
	lmu_bl = devm_kzalloc(chip->dev, sizeof(*lmu_bl) * num_backlights,
			      GFP_KERNEL);
	if (!lmu_bl)
		return -ENOMEM;

	/* Child is mapped to LMU backlight control bank */
	for_each_child_of_node(np, child) {
		each = lmu_bl + i;
		each->bank_id = i;
		each->chip = chip;

		ret = ti_lmu_backlight_of_get_ctrl_bank(child, each);
		if (ret) {
			of_node_put(np);
			return ret;
		}

		ti_lmu_backlight_of_get_light_properties(child, each);
		ti_lmu_backlight_of_get_brightness_mode(child, each);

		i++;
	}

	chip->lmu_bl = lmu_bl;
	chip->num_backlights = num_backlights;

	return 0;
}

static int ti_lmu_backlight_check_channel(struct ti_lmu_bl *lmu_bl)
{
	const struct ti_lmu_bl_cfg *cfg = lmu_bl->chip->cfg;
	const struct ti_lmu_bl_reg *reginfo = lmu_bl->chip->cfg->reginfo;

	if (!reginfo->brightness_msb)
		return -EINVAL;

	if (cfg->max_brightness > MAX_BRIGHTNESS_8BIT) {
		if (!reginfo->brightness_lsb)
			return -EINVAL;
	}

	return 0;
}

static int ti_lmu_backlight_create_channel(struct ti_lmu_bl *lmu_bl)
{
	struct regmap *regmap = lmu_bl->chip->lmu->regmap;
	const struct lmu_bl_reg_data *regdata =
		lmu_bl->chip->cfg->reginfo->channel;
	int num_channels = lmu_bl->chip->cfg->num_channels;
	int i, ret;
	u8 shift;

	/*
	 * How to create backlight output channels:
	 *   Check 'led_sources' bit and update registers.
	 *
	 *   1) Dual channel configuration
	 *     The 1st register data is used for single channel.
	 *     The 2nd register data is used for dual channel.
	 *
	 *   2) Multiple channel configuration
	 *     Each register data is mapped to bank ID.
	 *     Bit shift operation is defined in channel registers.
	 *
	 * Channel register data consists of address, mask, value.
	 */

	if (num_channels == NUM_DUAL_CHANNEL) {
		if (lmu_bl->led_sources == LMU_BACKLIGHT_DUAL_CHANNEL_USED)
			regdata++;

		return regmap_update_bits(regmap, regdata->reg, regdata->mask,
					  regdata->val);
	}

	for (i = 0; regdata && i < num_channels; i++) {
		/*
		 * Note that the result of regdata->val is shift bit.
		 * The bank_id should be shifted for the channel configuration.
		 */
		if (test_bit(i, &lmu_bl->led_sources)) {
			shift = regdata->val;
			ret = regmap_update_bits(regmap, regdata->reg,
						 regdata->mask,
						 lmu_bl->bank_id << shift);
			if (ret)
				return ret;
		}

		regdata++;
	}

	return 0;
}

static int ti_lmu_backlight_update_ctrl_mode(struct ti_lmu_bl *lmu_bl)
{
	struct regmap *regmap = lmu_bl->chip->lmu->regmap;
	const struct lmu_bl_reg_data *regdata =
		lmu_bl->chip->cfg->reginfo->mode + lmu_bl->bank_id;
	u8 val = regdata->val;

	if (!regdata)
		return 0;

	/*
	 * Update PWM configuration register.
	 * If the mode is register based, then clear the bit.
	 */
	if (lmu_bl->mode != BL_PWM_BASED)
		val = 0;

	return regmap_update_bits(regmap, regdata->reg, regdata->mask, val);
}

static int ti_lmu_backlight_convert_ramp_to_index(struct ti_lmu_bl *lmu_bl,
						  enum ti_lmu_bl_ramp_mode mode)
{
	const int *ramp_table = lmu_bl->chip->cfg->ramp_table;
	const int size = lmu_bl->chip->cfg->size_ramp;
	unsigned int msec;
	int i;

	if (!ramp_table)
		return -EINVAL;

	switch (mode) {
	case BL_RAMP_UP:
		msec = lmu_bl->ramp_up_msec;
		break;
	case BL_RAMP_DOWN:
		msec = lmu_bl->ramp_down_msec;
		break;
	default:
		return -EINVAL;
	}

	if (msec <= ramp_table[0])
		return 0;

	if (msec > ramp_table[size - 1])
		return size - 1;

	for (i = 1; i < size; i++) {
		if (msec == ramp_table[i])
			return i;

		/* Find an approximate index by looking up the table */
		if (msec > ramp_table[i - 1] && msec < ramp_table[i]) {
			if (msec - ramp_table[i - 1] < ramp_table[i] - msec)
				return i - 1;
			else
				return i;
		}
	}

	return -EINVAL;
}

static int ti_lmu_backlight_set_ramp(struct ti_lmu_bl *lmu_bl)
{
	struct regmap *regmap = lmu_bl->chip->lmu->regmap;
	const struct ti_lmu_bl_reg *reginfo = lmu_bl->chip->cfg->reginfo;
	int offset = reginfo->ramp_reg_offset;
	int i, ret, index;
	struct lmu_bl_reg_data regdata;

	for (i = BL_RAMP_UP; i <= BL_RAMP_DOWN; i++) {
		index = ti_lmu_backlight_convert_ramp_to_index(lmu_bl, i);
		if (index > 0) {
			if (!reginfo->ramp)
				break;

			regdata = reginfo->ramp[i];
			if (lmu_bl->bank_id != 0)
				regdata.val += offset;

			/* regdata.val is shift bit */
			ret = regmap_update_bits(regmap, regdata.reg,
						 regdata.mask,
						 index << regdata.val);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int ti_lmu_backlight_configure(struct ti_lmu_bl *lmu_bl)
{
	int ret;

	ret = ti_lmu_backlight_check_channel(lmu_bl);
	if (ret)
		return ret;

	ret = ti_lmu_backlight_create_channel(lmu_bl);
	if (ret)
		return ret;

	ret = ti_lmu_backlight_update_ctrl_mode(lmu_bl);
	if (ret)
		return ret;

	return ti_lmu_backlight_set_ramp(lmu_bl);
}

static int ti_lmu_backlight_init(struct ti_lmu_bl_chip *chip)
{
	struct regmap *regmap = chip->lmu->regmap;
	const struct lmu_bl_reg_data *regdata =
		chip->cfg->reginfo->init;
	int num_init = chip->cfg->reginfo->num_init;
	int i, ret;

	for (i = 0; regdata && i < num_init; i++) {
		ret = regmap_update_bits(regmap, regdata->reg, regdata->mask,
					 regdata->val);
		if (ret)
			return ret;

		regdata++;
	}

	return 0;
}

static int ti_lmu_backlight_reload(struct ti_lmu_bl_chip *chip)
{
	struct ti_lmu_bl *each;
	int i, ret;

	ret = ti_lmu_backlight_init(chip);
	if (ret)
		return ret;

	for (i = 0; i < chip->num_backlights; i++) {
		each = chip->lmu_bl + i;
		ret = ti_lmu_backlight_configure(each);
		if (ret)
			return ret;

		ret = backlight_update_status(each->bl_dev);
		if (ret)
			return ret;
	}

	return 0;
}

static int ti_lmu_backlight_add_device(struct device *dev,
				       struct ti_lmu_bl *lmu_bl)
{
	struct backlight_device *bl_dev;
	struct backlight_properties props;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_PLATFORM;
	props.brightness = lmu_bl->default_brightness;
	props.max_brightness = lmu_bl->chip->cfg->max_brightness;

	bl_dev = devm_backlight_device_register(dev, lmu_bl->name,
						lmu_bl->chip->dev, lmu_bl,
						&lmu_backlight_ops, &props);
	if (IS_ERR(bl_dev))
		return PTR_ERR(bl_dev);

	lmu_bl->bl_dev = bl_dev;

	return 0;
}

static struct ti_lmu_bl_chip *
ti_lmu_backlight_register(struct device *dev, struct ti_lmu *lmu,
			  const struct ti_lmu_bl_cfg *cfg)
{
	struct ti_lmu_bl_chip *chip;
	struct ti_lmu_bl *each;
	int i, ret;

	if (!cfg) {
		dev_err(dev, "Operation is not configured\n");
		return ERR_PTR(-EINVAL);
	}

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return ERR_PTR(-ENOMEM);

	chip->dev = dev;
	chip->lmu = lmu;
	chip->cfg = cfg;

	ret = ti_lmu_backlight_of_create(chip, dev->of_node);
	if (ret)
		return ERR_PTR(ret);

	ret = ti_lmu_backlight_init(chip);
	if (ret) {
		dev_err(dev, "Backlight init err: %d\n", ret);
		return ERR_PTR(ret);
	}

	for (i = 0; i < chip->num_backlights; i++) {
		each = chip->lmu_bl + i;

		ret = ti_lmu_backlight_configure(each);
		if (ret) {
			dev_err(dev, "Backlight config err: %d\n", ret);
			return ERR_PTR(ret);
		}

		ret = ti_lmu_backlight_add_device(dev, each);
		if (ret) {
			dev_err(dev, "Backlight device err: %d\n", ret);
			return ERR_PTR(ret);
		}

		ret = backlight_update_status(each->bl_dev);
		if (ret) {
			dev_err(dev, "Backlight update err: %d\n", ret);
			return ERR_PTR(ret);
		}
	}

	return chip;
}

static void ti_lmu_backlight_unregister(struct ti_lmu_bl_chip *chip)
{
	struct ti_lmu_bl *each;
	int i;

	/* Turn off the brightness */
	for (i = 0; i < chip->num_backlights; i++) {
		each = chip->lmu_bl + i;
		each->bl_dev->props.brightness = 0;
		backlight_update_status(each->bl_dev);
	}
}

static int ti_lmu_backlight_monitor_notifier(struct notifier_block *nb,
					     unsigned long action, void *unused)
{
	struct ti_lmu_bl_chip *chip = container_of(nb, struct ti_lmu_bl_chip,
						   nb);

	if (action == LMU_EVENT_MONITOR_DONE) {
		if (ti_lmu_backlight_reload(chip))
			return NOTIFY_STOP;
	}

	return NOTIFY_OK;
}

static int ti_lmu_backlight_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ti_lmu *lmu = dev_get_drvdata(dev->parent);
	struct ti_lmu_bl_chip *chip;
	int ret;

	chip = ti_lmu_backlight_register(dev, lmu, &lmu_bl_cfg[pdev->id]);
	if (IS_ERR(chip))
		return PTR_ERR(chip);

	/*
	 * Notifier callback is required because backlight device needs
	 * reconfiguration after fault detection procedure is done by
	 * ti-lmu-fault-monitor driver.
	 */
	if (chip->cfg->fault_monitor_used) {
		chip->nb.notifier_call = ti_lmu_backlight_monitor_notifier;
		ret = blocking_notifier_chain_register(&chip->lmu->notifier,
						       &chip->nb);
		if (ret)
			return ret;
	}

	platform_set_drvdata(pdev, chip);

	return 0;
}

static int ti_lmu_backlight_remove(struct platform_device *pdev)
{
	struct ti_lmu_bl_chip *chip = platform_get_drvdata(pdev);

	if (chip->cfg->fault_monitor_used)
		blocking_notifier_chain_unregister(&chip->lmu->notifier,
						   &chip->nb);

	ti_lmu_backlight_unregister(chip);

	return 0;
}

static const struct of_device_id ti_lmu_backlight_of_match[] = {
	{ .compatible = "ti,lm3532-backlight" },
	{ .compatible = "ti,lm3631-backlight" },
	{ .compatible = "ti,lm3632-backlight" },
	{ .compatible = "ti,lm3633-backlight" },
	{ .compatible = "ti,lm3695-backlight" },
	{ .compatible = "ti,lm3697-backlight" },
	{ }
};
MODULE_DEVICE_TABLE(of, ti_lmu_backlight_of_match);

static struct platform_driver ti_lmu_backlight_driver = {
	.probe  = ti_lmu_backlight_probe,
	.remove = ti_lmu_backlight_remove,
	.driver = {
		.name = "ti-lmu-backlight",
		.of_match_table = ti_lmu_backlight_of_match,
	},
};

module_platform_driver(ti_lmu_backlight_driver)

MODULE_DESCRIPTION("TI LMU Backlight Driver");
MODULE_AUTHOR("Milo Kim");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ti-lmu-backlight");
