/*
 *  PWM vibrator driver
 *
 *  Copyright (C) 2017 Sebastian Reichel <sre@kernel.org>
 *
 *  Based on previous work from:
 *  Copyright (C) 2012 Dmitry Torokhov <dmitry.torokhov@gmail.com>
 *
 *  Based on PWM beeper driver:
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 */

#include <linux/input.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>

struct pwm_vibrator {
	struct input_dev *input;
	struct pwm_device *pwm;
	struct pwm_device *pwm_dir;

	unsigned int level;
};

static int pwm_vibrator_start(struct pwm_vibrator *vibrator)
{
	struct pwm_state state;
	int err;

	pwm_get_state(vibrator->pwm, &state);
	state.enabled = true;
	pwm_set_relative_duty_cycle(&state, vibrator->level, 100);

	err = pwm_apply_state(vibrator->pwm, &state);
	if (err)
		return err;

	if (vibrator->pwm_dir) {
		pwm_get_state(vibrator->pwm_dir, &state);
		state.enabled = true;
		pwm_set_relative_duty_cycle(&state, 50, 100);

		err = pwm_apply_state(vibrator->pwm_dir, &state);
		if (err) {
			pwm_disable(vibrator->pwm);
			return err;
		}
	}

	return 0;
}

static void pwm_vibrator_stop(struct pwm_vibrator *vibrator)
{
	pwm_disable(vibrator->pwm);
	if (vibrator->pwm_dir)
		pwm_disable(vibrator->pwm_dir);
}

static int pwm_vibrator_play_effect(struct input_dev *dev, void *data,
				    struct ff_effect *effect)
{
	struct pwm_vibrator *vibrator = input_get_drvdata(dev);
	struct device *pdev = dev->dev.parent;
	int err;

	vibrator->level = effect->u.rumble.strong_magnitude;
	if (!vibrator->level)
		vibrator->level = effect->u.rumble.weak_magnitude;

	if (vibrator->level) {
		err = pwm_vibrator_start(vibrator);
		if (err) {
			dev_err(pdev, "failed to start vibrator: %d\n", err);
			return err;
		}
	} else {
		pwm_vibrator_stop(vibrator);
	}

	return 0;
}

static int pwm_vibrator_probe(struct platform_device *pdev)
{
	struct pwm_vibrator *vibrator;
	struct input_dev *input;
	struct pwm_state state;
	int err;

	vibrator = devm_kzalloc(&pdev->dev, sizeof(*vibrator), GFP_KERNEL);
	if (!vibrator)
		return -ENOMEM;

	input = devm_input_allocate_device(&pdev->dev);
	if (!vibrator || !input)
		return -ENOMEM;

	vibrator->input = input;

	vibrator->pwm = devm_pwm_get(&pdev->dev, NULL);
	err = PTR_ERR_OR_ZERO(vibrator->pwm);
	if (err) {
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to request pwm: %d\n", err);
		return err;
	}

	/* Sync up PWM state and ensure it is off. */
	pwm_init_state(vibrator->pwm, &state);
	state.enabled = false;
	err = pwm_apply_state(vibrator->pwm, &state);
	if (err) {
		dev_err(&pdev->dev, "failed to apply initial PWM state: %d\n",
			err);
		return err;
	}

	vibrator->pwm_dir = devm_pwm_get(&pdev->dev, "direction");
	err = PTR_ERR_OR_ZERO(vibrator->pwm);
	if (err == -ENODEV) {
		/* ignore optional PWM channel */
		vibrator->pwm_dir = NULL;
	} else if (err) {
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to request pwm: %d\n", err);
		return err;
	} else {
		/* Sync up PWM state and ensure it is off. */
		pwm_init_state(vibrator->pwm_dir, &state);
		state.enabled = false;
		err = pwm_apply_state(vibrator->pwm_dir, &state);
		if (err) {
			dev_err(&pdev->dev, "failed to apply initial PWM state: %d\n",
				err);
			return err;
		}
	}

	input->name = "pwm-vibrator";
	input->id.bustype = BUS_HOST;
	input->dev.parent = &pdev->dev;

	input_set_drvdata(input, vibrator);
	input_set_capability(input, EV_FF, FF_RUMBLE);

	err = input_ff_create_memless(input, NULL, pwm_vibrator_play_effect);
	if (err) {
		dev_err(&pdev->dev, "Couldn't create FF dev: %d\n", err);
		return err;
	}

	err = input_register_device(input);
	if (err) {
		dev_err(&pdev->dev, "Couldn't register input dev: %d\n", err);
		return err;
	}

	platform_set_drvdata(pdev, vibrator);

	return 0;
}

static int pwm_vibrator_remove(struct platform_device *pdev)
{
	struct pwm_vibrator *vibrator = platform_get_drvdata(pdev);
	pwm_disable(vibrator->pwm);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pwm_vibrator_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pwm_vibrator *vibrator = platform_get_drvdata(pdev);
	struct input_dev *input = vibrator->input;
	unsigned long flags;

	spin_lock_irqsave(&input->event_lock, flags);
	if (vibrator->level)
		pwm_vibrator_stop(vibrator);
	spin_unlock_irqrestore(&input->event_lock, flags);

	return 0;
}

static int pwm_vibrator_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pwm_vibrator *vibrator = platform_get_drvdata(pdev);
	struct input_dev *input = vibrator->input;
	unsigned long flags;

	spin_lock_irqsave(&input->event_lock, flags);
	if (vibrator->level)
		pwm_vibrator_start(vibrator);
	spin_unlock_irqrestore(&input->event_lock, flags);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pwm_vibrator_pm_ops,
			 pwm_vibrator_suspend, pwm_vibrator_resume);

static struct platform_driver pwm_vibrator_driver = {
	.probe	= pwm_vibrator_probe,
	.remove	= pwm_vibrator_remove,
	.driver	= {
		.name	= "pwm-vibrator",
		.pm	= &pwm_vibrator_pm_ops,
	},
};
module_platform_driver(pwm_vibrator_driver);

MODULE_AUTHOR("Sebastian Reichel <sre@kernel.org>");
MODULE_DESCRIPTION("PWM vibrator driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-vibrator");
