/************************************************************************************
** File: -
** OPLUS_ARCH_EXTENDS
** Copyright (C), 2020-2025, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**     add audio extend driver
** Version: 1.0
** --------------------------- Revision History: --------------------------------
**               <author>                                <date>          <desc>
**
************************************************************************************/

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>

#define AUDIO_EXTEND_DRIVER_NAME "audio-extend-drv"

enum {
	CODEC_VENDOR_NXP = 0,
	CODEC_VENDOR_MAXIM,
	CODEC_VENDOR_END,
	CODEC_VENDOR_MAX = CODEC_VENDOR_END,
};

enum {
	CODEC_I2S_ID = 0,
	CODEC_NAME,
	CODEC_DAI_NAME,
	CODEC_VENDOR,
	CODEC_PROP_END,
	CODEC_PROP_MAX = CODEC_PROP_END,
};

static const char *extend_speaker_prop[CODEC_PROP_MAX] = {
	[CODEC_I2S_ID] = "oplus,speaker-i2s-id",
	[CODEC_NAME] = "oplus,speaker-codec-name",
	[CODEC_DAI_NAME] = "oplus,speaker-codec-dai-name",
	[CODEC_VENDOR] = "oplus,speaker-vendor",
};

static const char *extend_dac_prop[CODEC_PROP_MAX] = {
	[CODEC_I2S_ID] = "oplus,dac-i2s-id",
	[CODEC_NAME] = "oplus,dac-codec-name",
	[CODEC_DAI_NAME] = "oplus,dac-codec-dai-name",
	[CODEC_VENDOR] = "oplus,dac-vendor",
};

struct codec_prop_info {
	int dev_cnt;
	u32 i2s_id;
	const char **codec_name;
	const char **codec_dai_name;
	const char *codec_vendor;
};

struct audio_switch_lock_info {
	int switch_lock_gpio;
};

struct audio_extend_data {
	struct codec_prop_info *spk_pa_info;
	struct codec_prop_info *hp_dac_info;
	struct audio_switch_lock_info *switch_lock_info;
	bool use_extern_spk;
	bool use_extern_dac;
};

static char pa_i2c_name[10] = "";
void set_pa_i2c_name(char*);
static struct platform_device *pa_dev = NULL;

static struct audio_extend_data *g_extend_pdata = NULL;

void set_pa_i2c_name (char * pa_name) {
	memcpy(pa_i2c_name,pa_name,7);
}
EXPORT_SYMBOL(set_pa_i2c_name);

static int extend_codec_prop_parse(struct device *dev, const char *codec_prop[], struct codec_prop_info *codec_info)
{
	int ret = 0;

	ret = of_property_read_string(dev->of_node, codec_prop[CODEC_VENDOR], &codec_info->codec_vendor);
	if (ret) {
		pr_warn("%s: Looking up '%s' property in node %s failed\n",
			__func__, codec_prop[CODEC_VENDOR], dev->of_node->full_name);
		return -EINVAL;
	} else {
		pr_info("%s: codec vendor: %s\n", __func__, codec_info->codec_vendor);
	}

	ret = of_property_read_u32(dev->of_node, codec_prop[CODEC_I2S_ID], &codec_info->i2s_id);
	if (ret) {
		pr_warn("%s: Looking up '%s' property in node %s failed\n",
			__func__, codec_prop[CODEC_I2S_ID], dev->of_node->full_name);
		return -EINVAL;
	} else {
		pr_info("%s: i2s id: %d\n", __func__, codec_info->i2s_id);
	}

	ret = of_property_count_strings(dev->of_node, codec_prop[CODEC_NAME]);
	if (ret <= 0) {
		pr_warn("%s: Invalid number of codecs, ret=%d\n",
			__func__, dev->of_node->full_name, ret);
		return -EINVAL;
	} else {
		codec_info->dev_cnt = ret;
		pr_info("%s: dev_cnt %d\n", __func__, codec_info->dev_cnt);
	}

	codec_info->codec_name = devm_kzalloc(dev, codec_info->dev_cnt * sizeof(char *), GFP_KERNEL);
	if (!codec_info->codec_name) {
		pr_warn("%s: kzalloc fail for codec_name!\n", __func__);
		return -ENOMEM;
	}
	ret = of_property_read_string_array(dev->of_node, codec_prop[CODEC_NAME], codec_info->codec_name, codec_info->dev_cnt);
	if (ret < 0) {
		pr_warn("%s: Looking up '%s' property in node %s failed\n",
			__func__, codec_prop[CODEC_NAME], dev->of_node->full_name);
		return -EINVAL;
	}

	codec_info->codec_dai_name = devm_kzalloc(dev, codec_info->dev_cnt * sizeof(char *), GFP_KERNEL);
	if (!codec_info->codec_dai_name) {
		pr_warn("%s: kzalloc fail for codec_dai_name!\n", __func__);
		return -ENOMEM;
	}
	ret = of_property_read_string_array(dev->of_node, codec_prop[CODEC_DAI_NAME], codec_info->codec_dai_name, codec_info->dev_cnt);
	if (ret < 0) {
		pr_warn("%s: Looking up '%s' property in node %s failed\n",
			__func__, codec_prop[CODEC_DAI_NAME], dev->of_node->full_name);
		return -EINVAL;
	}

	return 0;
}

static void extend_codec_be_dailink(struct device *dev, struct codec_prop_info *codec_info, struct snd_soc_dai_link *dailink, size_t size)
{
	int i2s_id = 0;
	int i = 0;
	int ret = 0;
	struct snd_soc_dai_link_component *codecs_comp = NULL;
	bool dais_register_status = false;

	if (!codec_info) {
		pr_err("%s: codec_info param invalid!\n", __func__);
		return;
	}

	if (!dailink) {
		pr_err("%s: dailink param invalid!\n", __func__);
		return;
	}

	if ((pa_dev != NULL) && (strcmp("0-0034",pa_i2c_name) == 0) && (strcmp("tfa98xx.2-0034",*codec_info->codec_name) == 0) &&
		(strcmp("tfa98xx-aif-2-34",*codec_info->codec_dai_name) == 0) && (codec_info->dev_cnt == 1)) {
		ret = of_property_read_string_array(pa_dev->dev.of_node, "oplus,speaker-codec-name0",
			codec_info->codec_name, codec_info->dev_cnt);
		if (ret < 0) {
			pr_err("%s: Looking up oplus,speaker-codec-name0  property in node %s failed\n",
				 __func__, pa_dev->dev.of_node->full_name);
		}
		ret = of_property_read_string_array(pa_dev->dev.of_node, "oplus,speaker-codec-dai-name0",
			codec_info->codec_dai_name, codec_info->dev_cnt);
		if (ret < 0) {
			pr_err("%s: Looking up oplus,speaker-codec-dai-name0  property in node %s failed\n",
				__func__, pa_dev->dev.of_node->full_name);
		}
	}

	if ((pa_dev != NULL) && (strcmp("1-0034",pa_i2c_name) == 0) && (strcmp("tfa98xx.2-0034",*codec_info->codec_name) == 0) &&
		(strcmp("tfa98xx-aif-2-34",*codec_info->codec_dai_name) == 0) && (codec_info->dev_cnt == 1)) {
		ret = of_property_read_string_array(pa_dev->dev.of_node, "oplus,speaker-codec-name1",
			codec_info->codec_name, codec_info->dev_cnt);
		if (ret < 0) {
			pr_err("%s: Looking up oplus,speaker-codec-name1  property in node %s failed\n",
				__func__ ,pa_dev->dev.of_node->full_name);
		}
		ret = of_property_read_string_array(pa_dev->dev.of_node, "oplus,speaker-codec-dai-name1",
			codec_info->codec_dai_name, codec_info->dev_cnt);
		if (ret < 0) {
			pr_err("%s: Looking up oplus,speaker-codec-dai-name1 property in node %s failed\n",
				__func__,pa_dev->dev.of_node->full_name);
		}
	}

	i2s_id = codec_info->i2s_id;
	pr_info("%s: i2s_id=%d, size=%d!\n", __func__, i2s_id, size);
	if ((i2s_id * 2 + 1) >= size) {
		pr_err("%s: i2s_id param invalid!\n", __func__);
		return;
	}
	pr_info("%s: codec vendor: %s, dev_cnt: %d.\n", __func__, codec_info->codec_vendor, codec_info->dev_cnt);
	codecs_comp = devm_kzalloc(dev, sizeof(struct snd_soc_dai_link_component) * codec_info->dev_cnt, GFP_KERNEL);
	if (!codecs_comp) {
		dev_err(dev, "%s: codec component alloc failed\n", __func__);
		return;
	}

	for (i = 0;i < codec_info->dev_cnt; i++) {
		codecs_comp[i].name = codec_info->codec_name[i];
		codecs_comp[i].dai_name = codec_info->codec_dai_name[i];
		if (!snd_soc_find_dai(&codecs_comp[i])) {
			pr_err("%s: dai %s not register, so extend be dailink failed\n", __func__, codecs_comp[i].name);
			dais_register_status = false;
			break;
		} else {
			dais_register_status = true;
			pr_info("%s: dais[%d] name:%s, dai_name:%s\n", __func__, i, codecs_comp[i].name, codecs_comp[i].dai_name);
		}
	}

	if (dais_register_status) {
		pr_info("%s: use %s rx dailink replace\n", __func__, codec_info->codec_vendor);
		/* RX dailink */
		dailink[i2s_id*2].codecs = codecs_comp;
		dailink[i2s_id*2].num_codecs = codec_info->dev_cnt;
		pr_info("%s: use %s tx dailink replace\n", __func__, codec_info->codec_vendor);
		/* TX dailink */
		dailink[i2s_id*2+1].codecs = codecs_comp;
		dailink[i2s_id*2+1].num_codecs = codec_info->dev_cnt;
	}
}

void extend_codec_i2s_be_dailinks(struct device *dev, struct snd_soc_dai_link *dailink, size_t size)
{
	if (!g_extend_pdata) {
		pr_err("%s: No extend data, do nothing.\n", __func__);
		return;
	}

	pr_info("%s: use_extern_spk %d\n", __func__, g_extend_pdata->use_extern_spk);
	if (g_extend_pdata->use_extern_spk && g_extend_pdata->spk_pa_info) {
		extend_codec_be_dailink(dev, g_extend_pdata->spk_pa_info, dailink, size);
	}

	pr_info("%s: use_extern_dac %d\n", __func__, g_extend_pdata->use_extern_dac);
	if (g_extend_pdata->use_extern_dac && g_extend_pdata->hp_dac_info) {
		extend_codec_be_dailink(dev, g_extend_pdata->hp_dac_info, dailink, size);
	}
}
EXPORT_SYMBOL(extend_codec_i2s_be_dailinks);

bool extend_codec_audio_amic_switch_is_locked(void) {
	if (!g_extend_pdata) {
		pr_err("%s: No extend data, do nothing.\n", __func__);
		return false;
	}

	if (!g_extend_pdata->switch_lock_info) {
		pr_err("%s: No extend switch_lock_info, do nothing.\n", __func__);
		return false;
	}

	if (gpio_is_valid(g_extend_pdata->switch_lock_info->switch_lock_gpio)) {
		int value = gpio_get_value(g_extend_pdata->switch_lock_info->switch_lock_gpio);
		if (value == 1) {
			pr_info("%s: amic switch is locked.\n", __func__);
			return true;
		} else {
			return false;
		}
	} else {
		pr_warn("%s: switch_lock_gpio is invalid.\n", __func__);
	}

	return false;
}
EXPORT_SYMBOL(extend_codec_audio_amic_switch_is_locked);

bool extend_codec_audio_spk_switch_is_locked(void) {
	if (!g_extend_pdata) {
		pr_err("%s: No extend data, do nothing.\n", __func__);
		return false;
	}

	if (!g_extend_pdata->switch_lock_info) {
		pr_err("%s: No extend switch_lock_info, do nothing.\n", __func__);
		return false;
	}

	if (gpio_is_valid(g_extend_pdata->switch_lock_info->switch_lock_gpio)) {
		int value = gpio_get_value(g_extend_pdata->switch_lock_info->switch_lock_gpio);
		if (value == 1) {
			pr_info("%s: spk switch is locked.\n", __func__);
			return true;
		} else {
			return false;
		}
	} else {
		pr_warn("%s: switch_lock_gpio is invalid.\n", __func__);
	}

	return false;
}
EXPORT_SYMBOL(extend_codec_audio_spk_switch_is_locked);

static int audio_extend_probe(struct platform_device *pdev)
{
	int ret = 0;

	dev_info(&pdev->dev, "%s: dev name %s\n", __func__,
		dev_name(&pdev->dev));

	if (!pdev->dev.of_node) {
		pr_err("%s: No dev node from device tree\n", __func__);
		return -EINVAL;
	}

	pa_dev = pdev;
	g_extend_pdata = devm_kzalloc(&pdev->dev, sizeof(struct audio_extend_data), GFP_KERNEL);
	if (!g_extend_pdata) {
		pr_err("%s: kzalloc mem fail!\n", __func__);
		return -ENOMEM;
	}

	g_extend_pdata->spk_pa_info =  devm_kzalloc(&pdev->dev, sizeof(struct codec_prop_info), GFP_KERNEL);
	if (g_extend_pdata->spk_pa_info) {
		ret = extend_codec_prop_parse(&pdev->dev, extend_speaker_prop, g_extend_pdata->spk_pa_info);
		if (ret == 0) {
			g_extend_pdata->use_extern_spk = true;
		} else {
			g_extend_pdata->use_extern_spk = false;
		}
	} else {
		g_extend_pdata->use_extern_spk = false;
		pr_warn("%s: kzalloc for spk pa info fail!\n", __func__);
	}

	g_extend_pdata->hp_dac_info =  devm_kzalloc(&pdev->dev, sizeof(struct codec_prop_info), GFP_KERNEL);
	if (g_extend_pdata->hp_dac_info) {
		ret = extend_codec_prop_parse(&pdev->dev, extend_dac_prop, g_extend_pdata->hp_dac_info);
		if (ret == 0) {
			g_extend_pdata->use_extern_dac = true;
		} else {
			g_extend_pdata->use_extern_dac = false;
		}
	} else {
		g_extend_pdata->use_extern_dac = false;
		pr_warn("%s: kzalloc for hp dac info fail!\n", __func__);
	}

	g_extend_pdata->switch_lock_info = devm_kzalloc(&pdev->dev, sizeof(struct audio_switch_lock_info), GFP_KERNEL);
	if (g_extend_pdata->switch_lock_info) {
		g_extend_pdata->switch_lock_info->switch_lock_gpio = of_get_named_gpio(pdev->dev.of_node, "oplus,switch-lock-gpio", 0);
		if (gpio_is_valid(g_extend_pdata->switch_lock_info->switch_lock_gpio)) {
			ret = devm_gpio_request_one(&pdev->dev, g_extend_pdata->switch_lock_info->switch_lock_gpio,
				GPIOF_DIR_IN, "AUDIO_SWITCH_LOCK");
			if (ret != 0) {
				pr_warn("%s Failed to request oplus,switch-lock-gpio", __func__);
				g_extend_pdata->switch_lock_info->switch_lock_gpio = -1;
			}
		} else {
			g_extend_pdata->switch_lock_info->switch_lock_gpio = -1;
			pr_warn("%s No audio switch lock GPIO provided, will not HW audio switch lock\n",__func__);
		}
	} else {
		pr_warn("%s: kzalloc for audio switch info fail!\n", __func__);
	}
	return 0;
}

static int audio_extend_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "%s: dev name %s\n", __func__,
		dev_name(&pdev->dev));

	return 0;
}

static const struct of_device_id audio_extend_of_match[] = {
	{.compatible = "oplus,asoc-audio"},
	{ }
};
MODULE_DEVICE_TABLE(of, audio_extend_of_match);

static struct platform_driver audio_extend_driver = {
	.probe          = audio_extend_probe,
	.remove         = audio_extend_remove,
	.driver         = {
		.name   = AUDIO_EXTEND_DRIVER_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = audio_extend_of_match,
		.suppress_bind_attrs = true,
	},
};

static int __init audio_extend_init(void)
{
	return platform_driver_register(&audio_extend_driver);
}

static void __exit audio_extend_exit(void)
{
	platform_driver_unregister(&audio_extend_driver);
}

module_init(audio_extend_init);
module_exit(audio_extend_exit);
MODULE_DESCRIPTION("ASoC Oplus Audio Driver");
MODULE_LICENSE("GPL v2");
