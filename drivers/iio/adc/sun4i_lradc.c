/*
 * Driver for the LRADC present on the  Allwinner sun4i
 *
 * Copyright 2016 Free Electrons
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/regulator/consumer.h>


#define SUN4I_LRADC_CTRL		0x00
#define SUN4I_LRADC_INTC		0x04
#define SUN4I_LRADC_INTS		0x08
#define SUN4I_LRADC_DATA0		0x0c
#define SUN4I_LRADC_DATA1		0x10

/* LRADC_CTRL bits */
#define FIRST_CONVERT_DLY(x)	((x) << 24) /* 8 bits */
#define CHAN_SELECT(x)		((x) << 22) /* 2 bits */
#define CONTINUE_TIME_SEL(x)	((x) << 16) /* 4 bits */
#define KEY_MODE_SEL(x)		((x) << 12) /* 2 bits */
#define LEVELA_B_CNT(x)		((x) << 8)  /* 4 bits */
#define LRADC_HOLD_EN		BIT(6)
#define LEVELB_VOL(x)		((x) << 4)  /* 2 bits */
#define LRADC_SAMPLE_RATE(x)	((x) << 2)  /* 2 bits */
#define LRADC_EN		BIT(0)

/* LRADC_INTC and LRADC_INTS bits */
#define CHAN1_KEYUP_IRQ		BIT(12)
#define CHAN1_ALRDY_HOLD_IRQ	BIT(11)
#define CHAN1_HOLD_IRQ		BIT(10)
#define	CHAN1_KEYDOWN_IRQ	BIT(9)
#define CHAN1_DATA_IRQ		BIT(8)
#define CHAN0_KEYUP_IRQ		BIT(4)
#define CHAN0_ALRDY_HOLD_IRQ	BIT(3)
#define CHAN0_HOLD_IRQ		BIT(2)
#define	CHAN0_KEYDOWN_IRQ	BIT(1)
#define CHAN0_DATA_IRQ		BIT(0)

#define NUM_CHANS		2
#define NUM_TRIGGERS		4

struct sun4i_lradc_state {
	void __iomem *base;
	struct regulator *vref_supply;
	u32 vref_mv;
	struct iio_trigger *trig[NUM_TRIGGERS];
	struct completion data_ok[NUM_CHANS];
	u32 last_event;
	spinlock_t lock;
};

#define SUN4I_LRADC_CHANNEL(chan) {					\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = (chan),					\
	.scan_index = (chan),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ)	\
}

static const struct iio_chan_spec sun4i_lradc_chan_array[] = {
	SUN4I_LRADC_CHANNEL(0),
	SUN4I_LRADC_CHANNEL(1),
};

static const struct {
	int val;
	int val2;
} sun4i_lradc_sample_freq_avail[] = {
	{250, 0},
	{125, 0},
	{62, 500000},
	{32, 250000},
};

static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("32.25 62.5 125 250");

static struct attribute *sun4i_lradc_attributes[] = {
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const struct attribute_group sun4i_lradc_attribute_group = {
	.attrs = sun4i_lradc_attributes,
};

static irqreturn_t sun4i_lradc_irq(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct sun4i_lradc_state *st = iio_priv(indio_dev);
	u32 ints, intc;

	spin_lock(&st->lock);

	ints = readl(st->base + SUN4I_LRADC_INTS);
	intc = readl(st->base + SUN4I_LRADC_INTC);

	if (ints & CHAN0_DATA_IRQ)
		complete_all(&st->data_ok[0]);

	if (ints & CHAN1_DATA_IRQ)
		complete_all(&st->data_ok[1]);

	st->last_event = ints & (CHAN1_KEYUP_IRQ | CHAN1_KEYDOWN_IRQ |
				 CHAN0_KEYUP_IRQ | CHAN0_KEYDOWN_IRQ);
	if (indio_dev->trig && st->last_event)
		iio_trigger_poll(indio_dev->trig);

	intc &= ~ints;
	writel(intc, st->base + SUN4I_LRADC_INTC);
	writel(ints, st->base + SUN4I_LRADC_INTS);

	spin_unlock(&st->lock);

	return IRQ_HANDLED;
}

static int sun4i_lradc_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct sun4i_lradc_state *st = iio_priv(indio_dev);
	int ret, tmp, idx;
	unsigned long flags;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		reinit_completion(&st->data_ok[chan->channel]);
		spin_lock_irqsave(&st->lock, flags);
		tmp = readl(st->base + SUN4I_LRADC_INTC);

		if (chan->channel)
			tmp |= CHAN1_DATA_IRQ;
		else
			tmp |= CHAN0_DATA_IRQ;

		writel(tmp, st->base + SUN4I_LRADC_INTC);
		spin_unlock_irqrestore(&st->lock, flags);

		ret = wait_for_completion_interruptible_timeout(
			&st->data_ok[chan->channel],
			msecs_to_jiffies(1000));
		if (ret == 0)
			return -ETIMEDOUT;

		if (chan->channel)
			*val = readl(st->base + SUN4I_LRADC_DATA1);
		else
			*val = readl(st->base + SUN4I_LRADC_DATA0);

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = st->vref_mv;
		*val2 = 6;

		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_SAMP_FREQ:
		tmp = readl(st->base + SUN4I_LRADC_CTRL);
		idx = (tmp >> 2) & 0x3;
		*val = sun4i_lradc_sample_freq_avail[idx].val;
		*val2 = sun4i_lradc_sample_freq_avail[idx].val2;
		return IIO_VAL_INT_PLUS_MICRO;

	default:
		break;
	}

	return -EINVAL;
}

static int sun4i_lradc_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct sun4i_lradc_state *st = iio_priv(indio_dev);
	u32 ctrl;
	int i;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		for (i = 0; i < ARRAY_SIZE(sun4i_lradc_sample_freq_avail); i++)
			if (sun4i_lradc_sample_freq_avail[i].val == val &&
			    sun4i_lradc_sample_freq_avail[i].val2 == val2)
				break;
		if (i == ARRAY_SIZE(sun4i_lradc_sample_freq_avail))
			return -EINVAL;

		ctrl = readl(st->base + SUN4I_LRADC_CTRL);
		ctrl &= ~LRADC_SAMPLE_RATE(0x3);
		writel(ctrl | LRADC_SAMPLE_RATE(i),
		       st->base + SUN4I_LRADC_CTRL);

		return 0;

	default:
		break;
	}

	return -EINVAL;
}

static int sun4i_lradc_write_raw_get_fmt(struct iio_dev *indio_dev,
					 struct iio_chan_spec const *chan,
					 long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return IIO_VAL_INT_PLUS_MICRO;

	default:
		break;
	}
	return IIO_VAL_INT_PLUS_NANO;
}

static const struct iio_info sun4i_lradc_info = {
	.driver_module = THIS_MODULE,
	.read_raw = sun4i_lradc_read_raw,
	.write_raw = sun4i_lradc_write_raw,
	.write_raw_get_fmt = sun4i_lradc_write_raw_get_fmt,
	.attrs = &sun4i_lradc_attribute_group,
};

static int sun4i_lradc_configure_trigger(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct sun4i_lradc_state *st = iio_priv(indio_dev);
	u32 ctrl;
	int i = 0;

	ctrl = readl(st->base + SUN4I_LRADC_CTRL) & (CHAN_SELECT(0x3)
						     | LRADC_SAMPLE_RATE(0x3)
						     | LRADC_EN);

	if (state)
		ctrl |= FIRST_CONVERT_DLY(2) | LEVELA_B_CNT(1) | LRADC_HOLD_EN
			| KEY_MODE_SEL(0) | LEVELB_VOL(i);
	else
		ctrl |= KEY_MODE_SEL(0x2);

	writel(ctrl, st->base + SUN4I_LRADC_CTRL);

	return 0;
}

static const struct iio_trigger_ops sun4i_lradc_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &sun4i_lradc_configure_trigger,
};

static char *trigger_names[] =
{
	"1.9V",
	"1.8V",
	"1.7V",
	"1.6V",
};

static int sun4i_lradc_trigger_init(struct iio_dev *indio_dev)
{
	int i, ret;
	struct iio_trigger *trig;
	struct sun4i_lradc_state *st = iio_priv(indio_dev);

	for (i = 0; i < NUM_TRIGGERS; i++) {
		trig = iio_trigger_alloc("%s-dev%d-%s", indio_dev->name,
					 indio_dev->id, trigger_names[i]);
		if (trig == NULL) {
			ret = -ENOMEM;
			goto error_trigger;
		}

		trig->dev.parent = indio_dev->dev.parent;
		iio_trigger_set_drvdata(trig, indio_dev);
		trig->ops = &sun4i_lradc_trigger_ops;

		ret = iio_trigger_register(trig);
		if (ret)
			goto error_trigger;

		st->trig[i] = trig;
	}

	return 0;

error_trigger:
	for (i--; i >= 0; i--) {
		iio_trigger_unregister(st->trig[i]);
		iio_trigger_free(st->trig[i]);
	}

	return ret;
}

static void sun4i_lradc_trigger_remove(struct iio_dev *indio_dev)
{
	struct sun4i_lradc_state *st = iio_priv(indio_dev);
	int i;

	for (i = 0; i < NUM_TRIGGERS; i++) {
		iio_trigger_unregister(st->trig[i]);
		iio_trigger_free(st->trig[i]);
	}
}

static int sun4i_lradc_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct device *dev = &pdev->dev;
	struct sun4i_lradc_state *st;
	int err;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->dev.parent = dev;
	indio_dev->name = dev_name(dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &sun4i_lradc_info;

	st->vref_supply = devm_regulator_get(dev, "vref");
	if (IS_ERR(st->vref_supply))
		return PTR_ERR(st->vref_supply);

	st->base = devm_ioremap_resource(dev,
			platform_get_resource(pdev, IORESOURCE_MEM, 0));
	if (IS_ERR(st->base))
		return PTR_ERR(st->base);

	writel(0, st->base + SUN4I_LRADC_INTC);
	writel(0xffffffff, st->base + SUN4I_LRADC_INTS);

	err = devm_request_irq(dev, platform_get_irq(pdev, 0),
				 sun4i_lradc_irq, 0,
				 "sun4i-a10-lradc", indio_dev);
	if (err)
		return err;

	/* Setup the ADC channels available on the board */
	indio_dev->num_channels = ARRAY_SIZE(sun4i_lradc_chan_array);
	indio_dev->channels = sun4i_lradc_chan_array;

	err = regulator_enable(st->vref_supply);
	if (err)
		return err;

	err = sun4i_lradc_trigger_init(indio_dev);
	if (err)
		return err;

	err = devm_iio_device_register(dev, indio_dev);
	if (err < 0) {
		dev_err(dev, "Couldn't register the device.\n");
		goto err_trig;
	}

	/* lradc Vref internally is divided by 2/3 */
	st->vref_mv = regulator_get_voltage(st->vref_supply) * 2 / 3000;

	init_completion(&st->data_ok[0]);
	init_completion(&st->data_ok[1]);
	spin_lock_init(&st->lock);

	/* Continuous mode on both channels */
	writel(CHAN_SELECT(0x3) | KEY_MODE_SEL(0x2) | LRADC_SAMPLE_RATE(0x00) |
	       LRADC_EN, st->base + SUN4I_LRADC_CTRL);

	return 0;

err_trig:
	sun4i_lradc_trigger_remove(indio_dev);

	return err;
}

static int sun4i_lradc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	devm_iio_device_unregister(&pdev->dev, indio_dev);
	sun4i_lradc_trigger_remove(indio_dev);

	return 0;
}

static const struct of_device_id sun4i_lradc_of_match[] = {
	{ .compatible = "allwinner,sun4i-a10-lradc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sun4i_lradc_of_match);

static struct platform_driver sun4i_lradc_driver = {
	.probe	= sun4i_lradc_probe,
	.remove = sun4i_lradc_remove,
	.driver = {
		.name	= "sun4i-a10-lradc",
		.of_match_table = of_match_ptr(sun4i_lradc_of_match),
	},
};

module_platform_driver(sun4i_lradc_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Allwinner sun4i low resolution ADC driver");
MODULE_AUTHOR("Alexandre Belloni <alexandre.belloni@free-electrons.com>");
