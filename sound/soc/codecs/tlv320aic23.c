/*
 * ALSA SoC TLV320AIC23 codec driver
 *
 * Author:      Arun KS, <arunks@mistralsolutions.com>
 * Copyright:   (C) 2008 Mistral Solutions Pvt Ltd.,
 *
 * Based on sound/soc/codecs/wm8731.c by Richard Purdie
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Notes:
 *  The AIC23 is a driver for a low power stereo audio
 *  codec tlv320aic23
 *
 *  The machine layer should disable unsupported inputs/outputs by
 *  snd_soc_dapm_disable_pin(codec, "LHPOUT"), etc.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "tlv320aic23.h"

#define AIC23_VERSION "0.1"

/*
 * AIC23 register cache
 */
static const u16 tlv320aic23_reg[] = {
	0x0097, 0x0097, 0x00F9, 0x00F9,	/* 0 */
	0x001A, 0x0004, 0x0007, 0x0001,	/* 4 */
	0x0020, 0x0000, 0x0000, 0x0000,	/* 8 */
	0x0000, 0x0000, 0x0000, 0x0000,	/* 12 */
};

/*
 * read tlv320aic23 register cache
 */
static inline unsigned int tlv320aic23_read_reg_cache(struct snd_soc_codec
						      *codec, unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg >= ARRAY_SIZE(tlv320aic23_reg))
		return -1;
	return cache[reg];
}

/*
 * write tlv320aic23 register cache
 */
static inline void tlv320aic23_write_reg_cache(struct snd_soc_codec *codec,
					       u8 reg, u16 value)
{
	u16 *cache = codec->reg_cache;
	if (reg >= ARRAY_SIZE(tlv320aic23_reg))
		return;
	cache[reg] = value;
}

/*
 * write to the tlv320aic23 register space
 */
static int tlv320aic23_write(struct snd_soc_codec *codec, unsigned int reg,
			     unsigned int value)
{

	u8 data[2];

	/* TLV320AIC23 has 7 bit address and 9 bits of data
	 * so we need to switch one data bit into reg and rest
	 * of data into val
	 */

	if ((reg < 0 || reg > 9) && (reg != 15)) {
		printk(KERN_WARNING "%s Invalid register R%u\n", __func__, reg);
		return -1;
	}

	data[0] = (reg << 1) | (value >> 8 & 0x01);
	data[1] = value & 0xff;

	tlv320aic23_write_reg_cache(codec, reg, value);

	if (codec->hw_write(codec->control_data, data, 2) == 2)
		return 0;

	printk(KERN_ERR "%s cannot write %03x to register R%u\n", __func__,
	       value, reg);

	return -EIO;
}

static const char *rec_src_text[] = { "Line", "Mic" };
static const char *deemph_text[] = {"None", "32Khz", "44.1Khz", "48Khz"};

static const struct soc_enum rec_src_enum =
	SOC_ENUM_SINGLE(TLV320AIC23_ANLG, 2, 2, rec_src_text);

static const struct snd_kcontrol_new tlv320aic23_rec_src_mux_controls =
SOC_DAPM_ENUM("Input Select", rec_src_enum);

static const struct soc_enum tlv320aic23_rec_src =
	SOC_ENUM_SINGLE(TLV320AIC23_ANLG, 2, 2, rec_src_text);
static const struct soc_enum tlv320aic23_deemph =
	SOC_ENUM_SINGLE(TLV320AIC23_DIGT, 1, 4, deemph_text);

//static const DECLARE_TLV_DB_SCALE(out_gain_tlv, -12100, 100, 0);
//static const DECLARE_TLV_DB_SCALE(input_gain_tlv, -1725, 75, 0);
//static const DECLARE_TLV_DB_SCALE(sidetone_vol_tlv, -1800, 300, 0);

static int snd_soc_tlv320aic23_put_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u16 val, reg;

	val = (ucontrol->value.integer.value[0] & 0x07);

	/* linear conversion to userspace
	* 000	=	-6db
	* 001	=	-9db
	* 010	=	-12db
	* 011	=	-18db (Min)
	* 100	=	0db (Max)
	*/
	val = (val >= 4) ? 4  : (3 - val);

	reg = tlv320aic23_read_reg_cache(codec, TLV320AIC23_ANLG) & (~0x1C0);
	tlv320aic23_write(codec, TLV320AIC23_ANLG, reg | (val << 6));

	return 0;
}

static int snd_soc_tlv320aic23_get_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u16 val;

	val = tlv320aic23_read_reg_cache(codec, TLV320AIC23_ANLG) & (0x1C0);
	val = val >> 6;
	val = (val >= 4) ? 4  : (3 -  val);
	ucontrol->value.integer.value[0] = val;
	return 0;

}

#define SOC_TLV320AIC23_SINGLE(xname, reg, shift, max, invert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.access = (1<<4) |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.info = snd_soc_info_volsw, .get = snd_soc_tlv320aic23_get_volsw,\
	.put = snd_soc_tlv320aic23_put_volsw, \
	.private_value =  SOC_SINGLE_VALUE(reg, shift, max, invert) }

static const struct snd_kcontrol_new tlv320aic23_snd_controls[] = {
	SOC_DOUBLE_R("Digital Playback Volume", TLV320AIC23_LCHNVOL,
			 TLV320AIC23_RCHNVOL, 0, 127, 0),
	SOC_SINGLE("Digital Playback Switch", TLV320AIC23_DIGT, 3, 1, 1),
	SOC_DOUBLE_R("Line Input Switch", TLV320AIC23_LINVOL,
		     TLV320AIC23_RINVOL, 7, 1, 0),
	SOC_DOUBLE_R("Line Input Volume", TLV320AIC23_LINVOL,
			 TLV320AIC23_RINVOL, 0, 31, 0),
	SOC_SINGLE("Mic Input Switch", TLV320AIC23_ANLG, 1, 1, 1),
	SOC_SINGLE("Mic Booster Switch", TLV320AIC23_ANLG, 0, 1, 0),
	SOC_TLV320AIC23_SINGLE("Sidetone Volume", TLV320AIC23_ANLG,
				  6, 4, 0),
	SOC_ENUM("Playback De-emphasis", tlv320aic23_deemph),
};

/* add non dapm controls */
static int aic23_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(tlv320aic23_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				  snd_soc_cnew(&tlv320aic23_snd_controls[i],
					       codec, NULL));
		if (err < 0)
			return err;
	}

	return 0;
}

/* PGA Mixer controls for Line and Mic switch */
static const struct snd_kcontrol_new tlv320aic23_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("Line Bypass Switch", TLV320AIC23_ANLG, 3, 1, 0),
	SOC_DAPM_SINGLE("Mic Sidetone Switch", TLV320AIC23_ANLG, 5, 1, 0),
	SOC_DAPM_SINGLE("Playback Switch", TLV320AIC23_ANLG, 4, 1, 0),
};

static const struct snd_soc_dapm_widget tlv320aic23_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "Playback", TLV320AIC23_PWR, 3, 1),
	SND_SOC_DAPM_ADC("ADC", "Capture", TLV320AIC23_PWR, 2, 1),
	SND_SOC_DAPM_MUX("Capture Source", SND_SOC_NOPM, 0, 0,
			 &tlv320aic23_rec_src_mux_controls),
	SND_SOC_DAPM_MIXER("Output Mixer", TLV320AIC23_PWR, 4, 1,
			   &tlv320aic23_output_mixer_controls[0],
			   ARRAY_SIZE(tlv320aic23_output_mixer_controls)),
	SND_SOC_DAPM_PGA("Line Input", TLV320AIC23_PWR, 0, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Mic Input", TLV320AIC23_PWR, 1, 1, NULL, 0),

	SND_SOC_DAPM_OUTPUT("LHPOUT"),
	SND_SOC_DAPM_OUTPUT("RHPOUT"),
	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),

	SND_SOC_DAPM_INPUT("LLINEIN"),
	SND_SOC_DAPM_INPUT("RLINEIN"),

	SND_SOC_DAPM_INPUT("MICIN"),
};

static const char *intercon[][3] = {
	/* Output Mixer */
	{"Output Mixer", "Line Bypass Switch", "Line Input"},
	{"Output Mixer", "Playback Switch", "DAC"},
	{"Output Mixer", "Mic Sidetone Switch", "Mic Input"},

	/* Outputs */
	{"RHPOUT", NULL, "Output Mixer"},
	{"LHPOUT", NULL, "Output Mixer"},
	{"LOUT", NULL, "Output Mixer"},
	{"ROUT", NULL, "Output Mixer"},

	/* Inputs */
	{"Line Input", "NULL", "LLINEIN"},
	{"Line Input", "NULL", "RLINEIN"},

	{"Mic Input", "NULL", "MICIN"},

	/* input mux */
	{"Capture Source", "Line", "Line Input"},
	{"Capture Source", "Mic", "Mic Input"},
	{"ADC", NULL, "Capture Source"},

	/* terminator */
	{NULL, NULL, NULL},

};

/* AIC23 driver data */
struct aic23_priv {
	int mclk;
	int requested_adc;
	int requested_dac;
};

/*
 * Common Crystals used
 * 11.2896 Mhz /128 = *88.2k  /192 = 58.8k
 * 12.0000 Mhz /125 = *96k    /136 = 88.235K
 * 12.2880 Mhz /128 = *96k    /192 = 64k
 * 16.9344 Mhz /128 = 132.3k /192 = *88.2k
 * 18.4320 Mhz /128 = 144k   /192 = *96k
 */

/*
 * Normal BOSR 0-256/2 = 128, 1-384/2 = 192
 * USB BOSR 0-250/2 = 125, 1-272/2 = 136
 */
static const int bosr_usb_divisor_table[] = {
	128, 125, 192, 136
};
#define LOWER_GROUP ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<6) | (1<<7))
#define UPPER_GROUP ((1<<8) | (1<<9) | (1<<10) | (1<<11)        | (1<<15))
static const unsigned short sr_valid_mask[] = {
	LOWER_GROUP|UPPER_GROUP,	/* Normal, bosr - 0*/
	LOWER_GROUP,			/* Usb, bosr - 0*/
	LOWER_GROUP|UPPER_GROUP,	/* Normal, bosr - 1*/
	UPPER_GROUP,			/* Usb, bosr - 1*/
};
/*
 * Every divisor is a factor of 11*12
 */
#define SR_MULT (11*12)
#define A(x) (SR_MULT/x)
static const unsigned char sr_adc_mult_table[] = {
	A(2), A(2), A(12), A(12),  0, 0, A(3), A(1),
	A(2), A(2), A(11), A(11),  0, 0, 0, A(1)
};
static const unsigned char sr_dac_mult_table[] = {
	A(2), A(12), A(2), A(12),  0, 0, A(3), A(1),
	A(2), A(11), A(2), A(11),  0, 0, 0, A(1)
};

static unsigned get_score(int adc, int adc_l, int adc_h, int need_adc,
		int dac, int dac_l, int dac_h, int need_dac)
{
	if ((adc >= adc_l) && (adc <= adc_h) &&
			(dac >= dac_l) && (dac <= dac_h)) {
		int diff_adc = need_adc - adc;
		int diff_dac = need_dac - dac;
		return abs(diff_adc) + abs(diff_dac);
	}
	return UINT_MAX;
}

static int find_rate(int mclk, u32 need_adc, u32 need_dac)
{
	int i, j;
	int best_i = -1;
	int best_j = -1;
	int best_div = 0;
	unsigned best_score = UINT_MAX;
	int adc_l, adc_h, dac_l, dac_h;

	need_adc *= SR_MULT;
	need_dac *= SR_MULT;
	/*
	 * rates given are +/- 1/32
	 */
	adc_l = need_adc - (need_adc >> 5);
	adc_h = need_adc + (need_adc >> 5);
	dac_l = need_dac - (need_dac >> 5);
	dac_h = need_dac + (need_dac >> 5);
	for (i = 0; i < ARRAY_SIZE(bosr_usb_divisor_table); i++) {
		int base = mclk / bosr_usb_divisor_table[i];
		int mask = sr_valid_mask[i];
		for (j = 0; j < ARRAY_SIZE(sr_adc_mult_table);
				j++, mask >>= 1) {
			int adc;
			int dac;
			int score;
			if ((mask & 1) == 0)
				continue;
			adc = base * sr_adc_mult_table[j];
			dac = base * sr_dac_mult_table[j];
			score = get_score(adc, adc_l, adc_h, need_adc,
					dac, dac_l, dac_h, need_dac);
			if (best_score > score) {
				best_score = score;
				best_i = i;
				best_j = j;
				best_div = 0;
			}
			score = get_score((adc >> 1), adc_l, adc_h, need_adc,
					(dac >> 1), dac_l, dac_h, need_dac);
			/* prefer to have a /2 */
			if ((score != UINT_MAX) && (best_score >= score)) {
				best_score = score;
				best_i = i;
				best_j = j;
				best_div = 1;
			}
		}
	}
	return (best_j << 2) | best_i | (best_div << TLV320AIC23_CLKIN_SHIFT);
}

#ifdef DEBUG
static void get_current_sample_rates(struct snd_soc_codec *codec, int mclk,
		u32 *sample_rate_adc, u32 *sample_rate_dac)
{
	int src = tlv320aic23_read_reg_cache(codec, TLV320AIC23_SRATE);
	int sr = (src >> 2) & 0x0f;
	int val = (mclk / bosr_usb_divisor_table[src & 3]);
	int adc = (val * sr_adc_mult_table[sr]) / SR_MULT;
	int dac = (val * sr_dac_mult_table[sr]) / SR_MULT;
	if (src & TLV320AIC23_CLKIN_HALF) {
		adc >>= 1;
		dac >>= 1;
	}
	*sample_rate_adc = adc;
	*sample_rate_dac = dac;
}
#endif

static int set_sample_rate_control(struct snd_soc_codec *codec, int mclk,
		u32 sample_rate_adc, u32 sample_rate_dac)
{
	/* Search for the right sample rate */
	int data = find_rate(mclk, sample_rate_adc, sample_rate_dac);
	if (data < 0) {
		printk(KERN_ERR "%s:Invalid rate %u,%u requested\n",
				__func__, sample_rate_adc, sample_rate_dac);
		return -EINVAL;
	}
	tlv320aic23_write(codec, TLV320AIC23_SRATE, data);
#ifdef DEBUG
	{
		u32 adc, dac;
		get_current_sample_rates(codec, mclk, &adc, &dac);
		printk(KERN_DEBUG "actual samplerate = %u,%u reg=%x\n",
			adc, dac, data);
	}
#endif
	return 0;
}

static int tlv320aic23_add_widgets(struct snd_soc_codec *codec)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tlv320aic23_dapm_widgets); i++)
		snd_soc_dapm_new_control(codec, &tlv320aic23_dapm_widgets[i]);

	/* set up audio path interconnects */
	for (i = 0; intercon[i][0] != NULL; i++)
		snd_soc_dapm_connect_input(codec, intercon[i][0],
					   intercon[i][1], intercon[i][2]);

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

static int tlv320aic23_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	u16 iface_reg;
	int ret;
	struct aic23_priv *aic23 = codec->private_data;
	u32 sample_rate_adc = aic23->requested_adc;
	u32 sample_rate_dac = aic23->requested_dac;
	u32 sample_rate = params_rate(params);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		aic23->requested_dac = sample_rate_dac = sample_rate;
		if (!sample_rate_adc)
			sample_rate_adc = sample_rate;
	} else {
		aic23->requested_adc = sample_rate_adc = sample_rate;
		if (!sample_rate_dac)
			sample_rate_dac = sample_rate;
	}
	ret = set_sample_rate_control(codec, aic23->mclk, sample_rate_adc,
			sample_rate_dac);
	if (ret < 0)
		return ret;

	iface_reg =
	    tlv320aic23_read_reg_cache(codec,
				       TLV320AIC23_DIGT_FMT) & ~(0x03 << 2);
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface_reg |= (0x01 << 2);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface_reg |= (0x02 << 2);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		iface_reg |= (0x03 << 2);
		break;
	}
	tlv320aic23_write(codec, TLV320AIC23_DIGT_FMT, iface_reg);

	return 0;
}

static int tlv320aic23_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;

	/* set active */
	tlv320aic23_write(codec, TLV320AIC23_ACTIVE, 0x0001);

	return 0;
}

static void tlv320aic23_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	struct aic23_priv *aic23 = codec->private_data;

	/* deactivate */
	if (!codec->active) {
		udelay(50);
		tlv320aic23_write(codec, TLV320AIC23_ACTIVE, 0x0);
	}
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		aic23->requested_dac = 0;
	else
		aic23->requested_adc = 0;
}

static int tlv320aic23_mute(struct snd_soc_codec_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 reg;

	reg = tlv320aic23_read_reg_cache(codec, TLV320AIC23_DIGT);
	if (mute)
		reg |= TLV320AIC23_DACM_MUTE;

	else
		reg &= ~TLV320AIC23_DACM_MUTE;

	tlv320aic23_write(codec, TLV320AIC23_DIGT, reg);

	return 0;
}

static int tlv320aic23_set_dai_fmt(struct snd_soc_codec_dai *codec_dai,
				   unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface_reg;

	iface_reg =
	    tlv320aic23_read_reg_cache(codec, TLV320AIC23_DIGT_FMT) & (~0x03);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface_reg |= TLV320AIC23_MS_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;

	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface_reg |= TLV320AIC23_FOR_I2S;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface_reg |= TLV320AIC23_LRP_ON;
	case SND_SOC_DAIFMT_DSP_B:
		iface_reg |= TLV320AIC23_FOR_DSP;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_reg |= TLV320AIC23_FOR_LJUST;
		break;
	default:
		return -EINVAL;

	}

	tlv320aic23_write(codec, TLV320AIC23_DIGT_FMT, iface_reg);

	return 0;
}

static int tlv320aic23_set_dai_sysclk(struct snd_soc_codec_dai *codec_dai,
				      int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic23_priv *aic23 = codec->private_data;
	aic23->mclk = freq;
	return 0;
}

static int tlv320aic23_dapm_event(struct snd_soc_codec *codec, int event)
{
	u16 reg = tlv320aic23_read_reg_cache(codec, TLV320AIC23_PWR) & 0xff7f;

	switch (event) {
	case SNDRV_CTL_POWER_D0:
		/* vref/mid, osc on, dac unmute */
		tlv320aic23_write(codec, TLV320AIC23_PWR, reg);
		break;
	case SNDRV_CTL_POWER_D1:
	case SNDRV_CTL_POWER_D2:
		break;
	case SNDRV_CTL_POWER_D3hot:
		/* everything off except vref/vmid, */
		tlv320aic23_write(codec, TLV320AIC23_PWR, reg | 0x0040);
		break;
	case SNDRV_CTL_POWER_D3cold:
		/* everything off, dac mute, inactive */
		tlv320aic23_write(codec, TLV320AIC23_ACTIVE, 0x0);
		tlv320aic23_write(codec, TLV320AIC23_PWR, 0xffff);
		break;
	}
	codec->dapm_state = event;
	return 0;
}

#define AIC23_RATES	SNDRV_PCM_RATE_8000_96000
#define AIC23_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			 SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE)

struct snd_soc_codec_dai tlv320aic23_dai = {
	.name = "tlv320aic23",
	.playback = {
		     .stream_name = "Playback",
		     .channels_min = 2,
		     .channels_max = 2,
		     .rates = AIC23_RATES,
		     .formats = AIC23_FORMATS,},
	.capture = {
		    .stream_name = "Capture",
		    .channels_min = 2,
		    .channels_max = 2,
		    .rates = AIC23_RATES,
		    .formats = AIC23_FORMATS,},
	.ops = {
            .hw_params	= tlv320aic23_hw_params,
            .prepare	= tlv320aic23_pcm_prepare,
            .shutdown	= tlv320aic23_shutdown,
        },
        .dai_ops = {
            .digital_mute	= tlv320aic23_mute,
            .set_sysclk	= tlv320aic23_set_dai_sysclk,
            .set_fmt	= tlv320aic23_set_dai_fmt,
        }
};
EXPORT_SYMBOL_GPL(tlv320aic23_dai);

static int tlv320aic23_suspend(struct platform_device *pdev,
			       pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	tlv320aic23_write(codec, TLV320AIC23_ACTIVE, 0x0);
	tlv320aic23_dapm_event(codec, SNDRV_CTL_POWER_D3cold);

	return 0;
}

static int tlv320aic23_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	u16 reg;

	/* Sync reg_cache with the hardware */
	for (reg = 0; reg < TLV320AIC23_RESET; reg++) {
		u16 val = tlv320aic23_read_reg_cache(codec, reg);
		tlv320aic23_write(codec, reg, val);
	}

	tlv320aic23_dapm_event(codec, SNDRV_CTL_POWER_D3hot);
	tlv320aic23_dapm_event(codec, codec->suspend_dapm_state);

	return 0;
}

/*
 * initialise the AIC23 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int tlv320aic23_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	int ret = 0;
	u16 reg;

	codec->name = "tlv320aic23";
	codec->owner = THIS_MODULE;
	codec->read = tlv320aic23_read_reg_cache;
	codec->write = tlv320aic23_write;
	codec->dapm_event = tlv320aic23_dapm_event;
	codec->dai = &tlv320aic23_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ARRAY_SIZE(tlv320aic23_reg);
	codec->reg_cache =
	    kmemdup(tlv320aic23_reg, sizeof(tlv320aic23_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;

	/* Reset codec */
	tlv320aic23_write(codec, TLV320AIC23_RESET, 0);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "tlv320aic23: failed to create pcms\n");
		goto pcm_err;
	}

	/* power on device */
	tlv320aic23_dapm_event(codec, SNDRV_CTL_POWER_D3hot);

	tlv320aic23_write(codec, TLV320AIC23_DIGT, TLV320AIC23_DEEMP_44K);

	/* Unmute input */
	reg = tlv320aic23_read_reg_cache(codec, TLV320AIC23_LINVOL);
	tlv320aic23_write(codec, TLV320AIC23_LINVOL,
			  (reg & (~TLV320AIC23_LIM_MUTED)) |
			  (TLV320AIC23_LRS_ENABLED));

	reg = tlv320aic23_read_reg_cache(codec, TLV320AIC23_RINVOL);
	tlv320aic23_write(codec, TLV320AIC23_RINVOL,
			  (reg & (~TLV320AIC23_LIM_MUTED)) |
			  TLV320AIC23_LRS_ENABLED);
	reg = tlv320aic23_read_reg_cache(codec, TLV320AIC23_ANLG);
	tlv320aic23_write(codec, TLV320AIC23_ANLG,
			 (reg) & (~TLV320AIC23_BYPASS_ON) &
			 (~TLV320AIC23_MICM_MUTED));
	/* Default output volume */
	tlv320aic23_write(codec, TLV320AIC23_LCHNVOL,
			  TLV320AIC23_DEFAULT_OUT_VOL &
			  TLV320AIC23_OUT_VOL_MASK);
	tlv320aic23_write(codec, TLV320AIC23_RCHNVOL,
			  TLV320AIC23_DEFAULT_OUT_VOL &
			  TLV320AIC23_OUT_VOL_MASK);

	tlv320aic23_write(codec, TLV320AIC23_ACTIVE, 0x1);

	aic23_add_controls(codec);
	tlv320aic23_add_widgets(codec);
	ret = snd_soc_register_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "tlv320aic23: failed to register card\n");
		goto card_err;
	}

	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	kfree(codec->reg_cache);
	return ret;
}
static struct snd_soc_device *tlv320aic23_socdev;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)

static unsigned short normal_i2c[] = { 0, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static struct i2c_driver tlv320aic23_i2c_driver;
static struct i2c_client client_template;

/*
 * If the i2c layer weren't so broken, we could pass this kind of data
 * around
 */
static int tlv320aic23_codec_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct snd_soc_device *socdev = tlv320aic23_socdev;
	struct tlv320aic23_setup_data *setup = socdev->codec_data;
	struct snd_soc_codec *codec = socdev->codec;
	struct i2c_client *i2c;
	int ret;
        
	if (addr != setup->i2c_address)
		return -ENODEV;

	client_template.adapter = adap;
	client_template.addr = addr;

	i2c = kmemdup(&client_template, sizeof(client_template), GFP_KERNEL);
	if (i2c == NULL) {
		kfree(codec);
		return -ENOMEM;
	}
	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;

	ret = i2c_attach_client(i2c);
	if (ret < 0) {
		printk(KERN_ERR "tlv320aic23: failed to attach codec at addr %x\n",
		       addr);
		goto err;
	}

	ret = tlv320aic23_init(socdev);
	if (ret < 0) {
		printk(KERN_ERR "tlv320aic23: failed to initialise AIC3X\n");
		goto err;
	}
	return ret;

err:
	kfree(codec);
	kfree(i2c);
	return ret;
}

static int tlv320aic23_i2c_detach(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	i2c_detach_client(client);
	kfree(codec->reg_cache);
	kfree(client);
	return 0;
}

static int tlv320aic23_i2c_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, tlv320aic23_codec_probe);
}

/* machine i2c codec control layer */
static struct i2c_driver tlv320aic23_i2c_driver = {
	.driver = {
		.name = "tlv320aic23 I2C Codec",
		.owner = THIS_MODULE,
	},
	.id = I2C_DRIVERID_I2CDEV,
	.attach_adapter = tlv320aic23_i2c_attach,
	.detach_client = tlv320aic23_i2c_detach,
	.command = NULL,
};

static struct i2c_client client_template = {
	.name = "tlv320aic23",
	.driver = &tlv320aic23_i2c_driver,
};
#endif

static int tlv320aic23_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct tlv320aic23_setup_data *setup = socdev->codec_data;
	struct snd_soc_codec *codec;
	struct aic23_priv *aic23;
	int ret = 0;

	printk(KERN_INFO "AIC23 Audio Codec %s\n", AIC23_VERSION);

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	aic23 = kzalloc(sizeof(struct aic23_priv), GFP_KERNEL);
	if (aic23 == NULL)
		return -ENOMEM;
	codec->private_data = aic23;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	tlv320aic23_socdev = socdev;
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
        if (setup->i2c_address){
            normal_i2c[0] = setup->i2c_address;
            codec->hw_write = (hw_write_t) i2c_master_send;
            codec->hw_read = NULL;
            ret = i2c_add_driver(&tlv320aic23_i2c_driver);
            if (ret != 0)
                    printk(KERN_ERR "can't add i2c driver");
        }
#endif
	return ret;
}

static int tlv320aic23_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	/* power down chip */
	if (codec->control_data)
		tlv320aic23_dapm_event(codec, SNDRV_CTL_POWER_D3);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&tlv320aic23_i2c_driver);
#endif
	kfree(codec->private_data);
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_tlv320aic23 = {
	.probe = tlv320aic23_probe,
	.remove = tlv320aic23_remove,
	.suspend = tlv320aic23_suspend,
	.resume = tlv320aic23_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_tlv320aic23);

MODULE_DESCRIPTION("ASoC TLV320AIC23 codec driver");
MODULE_AUTHOR("Arun KS <arunks@mistralsolutions.com>");
MODULE_LICENSE("GPL");
