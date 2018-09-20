/*
 * Copyright 2015 Free Electrons
 * Copyright 2015 NextThing Co.
 *
 * Author: Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define COUPLED_REGULATOR_MAX_SUPPLIES	16

struct coupled_regulator {
	struct regulator	**regulators;
	int			n_regulators;

	int			*voltages;
	int			n_voltages;
};

static int coupled_regulator_disable(struct regulator_dev *rdev)
{
	struct coupled_regulator *creg = rdev_get_drvdata(rdev);
	int ret, i;

	for (i = 0; i < creg->n_regulators; i++) {
		ret = regulator_disable(creg->regulators[i]);
		if (ret)
			break;
	}

	return ret;
}

static int coupled_regulator_enable(struct regulator_dev *rdev)
{
	struct coupled_regulator *creg = rdev_get_drvdata(rdev);
	int ret, i;

	for (i = 0; i < creg->n_regulators; i++) {
		ret = regulator_enable(creg->regulators[i]);
		if (ret)
			break;
	}

	return ret;
}

static int coupled_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct coupled_regulator *creg = rdev_get_drvdata(rdev);
	int ret = 0, i;

	for (i = 0; i < creg->n_regulators; i++) {
		ret &= regulator_is_enabled(creg->regulators[i]);
		if (ret < 0)
			break;
	}

	return ret;
}

static int coupled_regulator_list_voltage(struct regulator_dev *rdev,
					  unsigned int selector)
{
	struct coupled_regulator *creg = rdev_get_drvdata(rdev);

	if (selector >= creg->n_voltages)
		return -EINVAL;

	return creg->voltages[selector];
}

static struct regulator_ops coupled_regulator_ops = {
	.enable			= coupled_regulator_enable,
	.disable		= coupled_regulator_disable,
	.is_enabled		= coupled_regulator_is_enabled,
	.list_voltage		= coupled_regulator_list_voltage,
};

static struct regulator_desc coupled_regulator_desc = {
	.name		= "coupled-voltage-regulator",
	.type		= REGULATOR_VOLTAGE,
	.ops		= &coupled_regulator_ops,
	.owner		= THIS_MODULE,
};

static int coupled_regulator_probe(struct platform_device *pdev)
{
	const struct regulator_init_data *init_data;
	struct coupled_regulator *creg;
	struct regulator_config config = { };
	struct regulator_dev *regulator;
	struct regulator_desc *desc;
	struct device_node *np = pdev->dev.of_node;
	int max_voltages, i;

	if (!np) {
		dev_err(&pdev->dev, "Device Tree node missing\n");
		return -EINVAL;
	}

	creg = devm_kzalloc(&pdev->dev, sizeof(*creg), GFP_KERNEL);
	if (!creg)
		return -ENOMEM;

	init_data = of_get_regulator_init_data(&pdev->dev, np,
					       &coupled_regulator_desc);
	if (!init_data)
		return -ENOMEM;

	config.of_node = np;
	config.dev = &pdev->dev;
	config.driver_data = creg;
	config.init_data = init_data;

	for (i = 0; i < COUPLED_REGULATOR_MAX_SUPPLIES; i++) {
		char *propname = kasprintf(GFP_KERNEL, "vin%d-supply", i);
		const void *prop = of_get_property(np, propname, NULL);
		kfree(propname);

		if (!prop) {
			creg->n_regulators = i;
			break;
		}
	}

	dev_dbg(&pdev->dev, "Found %d parent regulators\n",
		creg->n_regulators);

	if (!creg->n_regulators) {
		dev_err(&pdev->dev, "No parent regulators listed\n");
		return -EINVAL;
	}

	creg->regulators = devm_kcalloc(&pdev->dev, creg->n_regulators,
					sizeof(*creg->regulators), GFP_KERNEL);
	if (!creg->regulators)
		return -ENOMEM;

	for (i = 0; i < creg->n_regulators; i++) {
		char *propname = kasprintf(GFP_KERNEL, "vin%d", i);

		dev_dbg(&pdev->dev, "Trying to get supply %s\n", propname);

		creg->regulators[i] = devm_regulator_get(&pdev->dev, propname);
		kfree(propname);

		if (IS_ERR(creg->regulators[i])) {
			dev_err(&pdev->dev, "Couldn't get regulator vin%d\n",
				i);
			return PTR_ERR(creg->regulators[i]);
		}
	}

	/*
	 * Since we want only to expose voltages that can be set on
	 * all the regulators, we won't have more voltages supported
	 * than the number of voltages supported by the first
	 * regulator in our list
	 */
	max_voltages = regulator_count_voltages(creg->regulators[0]);

	creg->voltages = devm_kcalloc(&pdev->dev, max_voltages, sizeof(int),
				      GFP_KERNEL);

	/* Build up list of supported voltages */
	for (i = 0; i < max_voltages; i++) {
		int voltage = regulator_list_voltage(creg->regulators[0], i);
		bool usable = true;
		int j;

		if (voltage <= 0)
			continue;

		dev_dbg(&pdev->dev, "Checking voltage %d...\n", voltage);

		for (j = 1; j < creg->n_regulators; j++) {
			if (!regulator_is_supported_voltage(creg->regulators[j],
							    voltage, voltage)) {
				usable = false;
				break;
			}
		}

		if (usable) {
			creg->voltages[creg->n_voltages++] = voltage;
			dev_dbg(&pdev->dev,
				 "Adding voltage %d to the list of supported voltages\n",
				 voltage);
		}
	}

	dev_dbg(&pdev->dev, "Supporting %d voltages\n", creg->n_voltages);

	desc = devm_kmemdup(&pdev->dev, &coupled_regulator_desc,
			    sizeof(coupled_regulator_desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	desc->n_voltages = creg->n_voltages;

	regulator = devm_regulator_register(&pdev->dev, desc, &config);
	if (IS_ERR(regulator)) {
		dev_err(&pdev->dev, "Failed to register regulator %s\n",
			coupled_regulator_desc.name);
		return PTR_ERR(regulator);
	}

	return 0;
}

static struct of_device_id coupled_regulator_of_match[] = {
	{ .compatible = "coupled-voltage-regulator" },
	{ /* Sentinel */ },
};

static struct platform_driver coupled_regulator_driver = {
	.probe	= coupled_regulator_probe,

	.driver = {
		.name		= "coupled-voltage-regulator",
		.of_match_table	= coupled_regulator_of_match,
	},
};
module_platform_driver(coupled_regulator_driver);

MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com>");
MODULE_DESCRIPTION("Coupled Regulator Driver");
MODULE_LICENSE("GPL");
