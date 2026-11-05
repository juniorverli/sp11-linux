// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2023, Linaro Limited

#include <dt-bindings/sound/qcom,q6afe.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soundwire/sdw.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/jack.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/sp11-guard.h>

#include "common.h"
#include "qdsp6/q6afe.h"
#include "qdsp6/q6dsp-common.h"
#include "sdw.h"

struct x1e80100_snd_data {
	bool stream_prepared[AFE_PORT_MAX];
	struct snd_soc_card *card;
	struct snd_soc_jack jack;
	struct snd_soc_jack dp_jack[8];
	bool jack_setup;
};

static int x1e80100_snd_init(struct snd_soc_pcm_runtime *rtd)
{
	struct x1e80100_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_jack *dp_jack = NULL;
	int dp_pcm_id = 0;

	switch (cpu_dai->id) {
	case WSA_CODEC_DMA_RX_0:
	case WSA_CODEC_DMA_RX_1:
		/*
		 * Set limit of -3 dB on Digital Volume and 0 dB on PA Volume
		 * to reduce the risk of speaker damage until we have active
		 * speaker protection in place.
		 */
		snd_soc_limit_volume(card, "WSA WSA_RX0 Digital Volume", 81);
		snd_soc_limit_volume(card, "WSA WSA_RX1 Digital Volume", 81);
		snd_soc_limit_volume(card, "WSA2 WSA_RX0 Digital Volume", 81);
		snd_soc_limit_volume(card, "WSA2 WSA_RX1 Digital Volume", 81);
		/* The MIX paths feed the same amplifier; do not leave them at +40 dB */
		snd_soc_limit_volume(card, "WSA WSA_RX0_MIX Digital Volume", 81);
		snd_soc_limit_volume(card, "WSA WSA_RX1_MIX Digital Volume", 81);
		snd_soc_limit_volume(card, "WSA2 WSA_RX0_MIX Digital Volume", 81);
		snd_soc_limit_volume(card, "WSA2 WSA_RX1_MIX Digital Volume", 81);
		snd_soc_limit_volume(card, "SpkrLeft PA Volume", 6);
		snd_soc_limit_volume(card, "SpkrRight PA Volume", 6);
		snd_soc_limit_volume(card, "WooferLeft PA Volume", 6);
		snd_soc_limit_volume(card, "TweeterLeft PA Volume", 6);
		snd_soc_limit_volume(card, "WooferRight PA Volume", 6);
		snd_soc_limit_volume(card, "TweeterRight PA Volume", 6);
		break;
	case DISPLAY_PORT_RX_0:
		dp_pcm_id = 0;
		dp_jack = &data->dp_jack[dp_pcm_id];
		break;
	case DISPLAY_PORT_RX_1 ... DISPLAY_PORT_RX_7:
		dp_pcm_id = cpu_dai->id - DISPLAY_PORT_RX_1 + 1;
		dp_jack = &data->dp_jack[dp_pcm_id];
		break;
	default:
		break;
	}

	if (dp_jack)
		return qcom_snd_dp_jack_setup(rtd, dp_jack, dp_pcm_id);

	return qcom_snd_wcd_jack_setup(rtd, &data->jack, &data->jack_setup);
}

static int x1e80100_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_interval *rate = hw_param_interval(params,
						      SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
							  SNDRV_PCM_HW_PARAM_CHANNELS);

	rate->min = rate->max = 48000;
	switch (cpu_dai->id) {
	case WSA_CODEC_DMA_TX_0:
		/* Speaker protection voltage/current sense: V and I of two speakers */
		rate->min = 8000;
		rate->max = 8000;
		channels->min = 2;
		channels->max = 2;
		snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
		snd_mask_set_format(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
				    SNDRV_PCM_FORMAT_S32_LE);
		break;
	case WSA_CODEC_DMA_TX_2:
		/* Speaker protection class-H (CPS) feedback of two speakers */
		rate->min = 24000;
		rate->max = 24000;
		channels->min = 2;
		channels->max = 2;
		snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
		snd_mask_set_format(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
				    SNDRV_PCM_FORMAT_S32_LE);
		break;
	case TX_CODEC_DMA_TX_0:
	case TX_CODEC_DMA_TX_1:
	case TX_CODEC_DMA_TX_2:
	case TX_CODEC_DMA_TX_3:
		channels->min = 1;
		break;
	default:
		break;
	}

	return 0;
}

static int x1e80100_snd_hw_map_channels(unsigned int *ch_map, int num)
{
	switch (num) {
	case 1:
		ch_map[0] = PCM_CHANNEL_FC;
		break;
	case 2:
		ch_map[0] = PCM_CHANNEL_FL;
		ch_map[1] = PCM_CHANNEL_FR;
		break;
	case 3:
		ch_map[0] = PCM_CHANNEL_FL;
		ch_map[1] = PCM_CHANNEL_FR;
		ch_map[2] = PCM_CHANNEL_FC;
		break;
	case 4:
		ch_map[0] = PCM_CHANNEL_FL;
		ch_map[1] = PCM_CHANNEL_LB;
		ch_map[2] = PCM_CHANNEL_FR;
		ch_map[3] = PCM_CHANNEL_RB;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * Speaker stream gate, only enforced on the Surface Pro 11. It decides if the
 * stream may be opened; the amplifier stays muted until the amplifier driver
 * sees the guard allow the unmute (V/I sense and graph running, calibrated,
 * no fault), which only happens once the stream is triggered.
 */
static int x1e80100_spk_check(struct device *dev)
{
	if (!of_machine_is_compatible("microsoft,denali"))
		return 0;

	return sp11_guard_check(dev);
}

static int x1e80100_snd_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct x1e80100_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	unsigned int channels = substream->runtime->channels;
	unsigned int rx_slot[4];
	int ret;

	switch (cpu_dai->id) {
	case WSA_CODEC_DMA_RX_0:
	case WSA_CODEC_DMA_RX_1:
		/* The state may have changed since startup, for example a fault */
		ret = x1e80100_spk_check(rtd->dev);
		if (ret)
			return ret;

		ret = x1e80100_snd_hw_map_channels(rx_slot, channels);
		if (ret)
			return ret;

		ret = snd_soc_dai_set_channel_map(cpu_dai, 0, NULL,
						  channels, rx_slot);
		if (ret)
			return ret;
		break;
	default:
		break;
	}

	return qcom_snd_sdw_prepare(substream, &data->stream_prepared[cpu_dai->id]);
}

static int x1e80100_snd_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct x1e80100_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	return qcom_snd_sdw_hw_free(substream, &data->stream_prepared[cpu_dai->id]);
}

static int x1e80100_snd_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	int ret;

	/*
	 * Surface Pro 11: refuse the stream when the closed-loop data is
	 * missing, a fault is latched or the kill switch is set. The graph, the
	 * V/I sense and the calibration only exist once the streams are
	 * prepared and started, so they are not checked here: the amplifier
	 * driver keeps the power amplifier off until they are in place.
	 * Refuse before any SoundWire or DAPM power-up.
	 */
	if (cpu_dai->id == WSA_CODEC_DMA_RX_0 || cpu_dai->id == WSA_CODEC_DMA_RX_1) {
		ret = x1e80100_spk_check(rtd->dev);
		if (ret)
			return ret;
	}

	return qcom_snd_sdw_startup(substream);
}

static const struct snd_soc_ops x1e80100_be_ops = {
	.startup = x1e80100_snd_startup,
	.shutdown = qcom_snd_sdw_shutdown,
	.hw_free = x1e80100_snd_hw_free,
	.prepare = x1e80100_snd_prepare,
};

/*
 * Called by the guard when it stops allowing the speakers (a fault, or the V/I
 * sense is gone): mute every WSA amplifier
 */
static void x1e80100_spk_mute(void *priv)
{
	struct snd_soc_card *card = priv;
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *codec_dai;
	int i;

	for_each_card_rtds(card, rtd) {
		struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

		if (cpu_dai->id != WSA_CODEC_DMA_RX_0 && cpu_dai->id != WSA_CODEC_DMA_RX_1)
			continue;

		for_each_rtd_codec_dais(rtd, i, codec_dai)
			snd_soc_dai_digital_mute(codec_dai, 1, SNDRV_PCM_STREAM_PLAYBACK);
	}
}

static void x1e80100_spk_unregister(void *priv)
{
	sp11_guard_clear_mute(priv);
}

static void x1e80100_add_be_ops(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *link;
	int i;

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm == 1) {
			link->init = x1e80100_snd_init;
			link->be_hw_params_fixup = x1e80100_be_hw_params_fixup;
			link->ops = &x1e80100_be_ops;
		}
	}
}

static int x1e80100_platform_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct x1e80100_snd_data *data;
	struct device *dev = &pdev->dev;
	int ret;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;
	/* Allocate the private data */
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	card->owner = THIS_MODULE;
	card->dev = dev;
	dev_set_drvdata(dev, card);
	snd_soc_card_set_drvdata(card, data);

	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;

	card->driver_name = of_device_get_match_data(dev);
	x1e80100_add_be_ops(card);

	if (of_machine_is_compatible("microsoft,denali") && sp11_guard_nospk())
		dev_warn(dev, "sp11.nospk is set, speakers are disabled\n");

	ret = devm_snd_soc_register_card(dev, card);
	if (ret)
		return ret;

	/*
	 * Registered after the card so that the devm cleanup unhooks the mute
	 * callback (and waits for a running one) before the card is
	 * unregistered. The amplifier driver enforces the guard by itself, this
	 * only mutes a running stream when the guard stops allowing it.
	 */
	if (of_machine_is_compatible("microsoft,denali")) {
		sp11_guard_set_mute(x1e80100_spk_mute, card);
		ret = devm_add_action_or_reset(dev, x1e80100_spk_unregister, card);
	}

	return ret;
}

static const struct of_device_id snd_x1e80100_dt_match[] = {
	{ .compatible = "qcom,x1e80100-sndcard", .data = "x1e80100" },
	{ .compatible = "qcom,glymur-sndcard", .data = "glymur" },
	{}
};
MODULE_DEVICE_TABLE(of, snd_x1e80100_dt_match);

static struct platform_driver snd_x1e80100_driver = {
	.probe  = x1e80100_platform_probe,
	.driver = {
		.name = "snd-x1e80100",
		.of_match_table = snd_x1e80100_dt_match,
	},
};
module_platform_driver(snd_x1e80100_driver);
MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_AUTHOR("Krzysztof Kozlowski <krzysztof.kozlowski@linaro.org>");
MODULE_DESCRIPTION("Qualcomm X1E80100 ASoC Machine Driver");
MODULE_LICENSE("GPL");
