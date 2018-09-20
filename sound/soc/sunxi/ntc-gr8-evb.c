/*
 * ALSA SoC driver for NextThing Co GR8 Evaluation Board
 *
 * Copyright (C) 2016 Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clkdev.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/module.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include "../codecs/wm8978.h"

static int gr8_evb_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int rate = params_rate(params);
	int ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, WM8978_MCLK, rate * 512,
				     SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	return snd_soc_dai_set_sysclk(cpu_dai, 0, rate * 512,
				      SND_SOC_CLOCK_OUT);
}

static struct snd_soc_ops gr8_evb_dai_ops = {
	.hw_params = gr8_evb_hw_params,
};

static struct snd_soc_dai_link gr8_evb_dai_link = {
	.name		= "wm8978",
	.stream_name	= "WM8978",
	.codec_dai_name	= "wm8978-hifi",
	.ops		= &gr8_evb_dai_ops,
	.dai_fmt	= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S |
		   	  SND_SOC_DAIFMT_CBS_CFS,
};

static struct snd_soc_card gr8_evb_card = {
	.name		= "i2s-wm8978",
	.owner		= THIS_MODULE,
	.dai_link	= &gr8_evb_dai_link,
	.num_links	= 1,
};

static int gr8_evb_probe(struct platform_device *pdev)
{
	struct snd_soc_dai_link *link = &gr8_evb_dai_link;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	/* register the soc card */
	gr8_evb_card.dev = &pdev->dev;

	link->codec_of_node = of_parse_phandle(np, "allwinner,audio-codec", 0);
	if (!link->codec_of_node) {
		dev_err(&pdev->dev, "Missing audio codec\n");
		return -EINVAL;
	}

	link->cpu_of_node = of_parse_phandle(np, "allwinner,i2s-controller", 0);
	if (!link->cpu_of_node) {
		dev_err(&pdev->dev, "Missing I2S controller\n");
		return -EINVAL;
	}
	link->platform_of_node = link->cpu_of_node;

	ret = devm_snd_soc_register_card(&pdev->dev, &gr8_evb_card);
	if (ret) {
		dev_err(&pdev->dev,
			"Soc register card failed %d\n", ret);
		return ret;
	}

	return ret;
}

static const struct of_device_id gr8_evb_of_match[] = {
	{ .compatible = "nextthing,gr8-evb-audio", },
	{},
};

MODULE_DEVICE_TABLE(of, gr8_evb_of_match);

static struct platform_driver gr8_evb_driver = {
	.probe = gr8_evb_probe,
	.driver = {
		.name	= "gr8-evb-audio",
		.of_match_table = gr8_evb_of_match,
	},
};

module_platform_driver(gr8_evb_driver);

MODULE_AUTHOR("Xing Zheng <zhengxing@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip rt5645 machine ASoC driver");
MODULE_LICENSE("GPL v2");
