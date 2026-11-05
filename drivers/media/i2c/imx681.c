// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX681 image sensor driver.
 *
 * The IMX681 has no public register documentation, so this driver contains
 * no sensor register value. It powers the sensor, registers a V4L2 sub-device
 * with a CSI-2 C-PHY source pad and takes everything that depends on the
 * sensor's registers from a register blob (see sensor-regblob.h): the
 * identification register, the init, stream on and stream off sequences, the
 * exposure, gain and frame length registers, the modes and the media bus
 * code. The blob is named by the "firmware-name" property and is generated
 * locally from data the owner of the device already has.
 *
 * The media bus code is a field of the blob. The Windows module data states
 * the colour filter order of the sensor, RGGB, and the blob generator writes
 * the matching SRGGB10 code; the driver has no default of its own and
 * rejects a blob without a RAW10 Bayer code.
 *
 * Gain and frame rate follow what the Windows camera stack does (a reverse
 * engineering result, not measured on this device, see the comments at the
 * constants below): a Sony reciprocal analogue gain, a Q8 digital gain, and
 * a module "pixel clock" that is not the clock the line length counts in.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "sensor-regblob.h"

#define IMX681_XCLK_FREQ	19200000

/*
 * Analogue gain, Sony reciprocal law: code = 512 - 512 / gain, that is
 * gain = 512 / (512 - code), 1.0x at code 0. The Windows tuning data limits
 * the codes to 0..479 (15.5x). Per Windows analysis, unconfirmed on hardware.
 */
#define IMX681_AGAIN_MIN	0
#define IMX681_AGAIN_MAX	479
#define IMX681_AGAIN_STEP	1
#define IMX681_AGAIN_DEFAULT	0

/*
 * Digital gain, Q8 (256 = 1.0x), up to 4.0x, in the 16 bit register 0x020e
 * (the MIPI CCS global digital gain address). Per Windows analysis (the
 * "digitalGlobalGainAddr" of the tuning data), unconfirmed on hardware; the
 * Windows mode sequences write the unity value 0x0100 to this pair.
 */
#define IMX681_REG_DIGITAL_GAIN	CCI_REG16(0x020e)
#define IMX681_DGAIN_MIN	256
#define IMX681_DGAIN_MAX	1024
#define IMX681_DGAIN_STEP	1
#define IMX681_DGAIN_DEFAULT	256

/*
 * The pixel clock of the Windows module data is not the clock the line
 * length counts in. For every IMX681 mode the module data holds
 * pixel_clock / (line_length * frame_length * frame_rate) = 0.7620 (constant
 * across modes, hardware-independent arithmetic on the module values), so the
 * line-length clock is pixel_clock / 0.7620. Line length and frame length stay
 * in the units the blob gives (they equal the sensor registers); only the
 * clock is converted: V4L2_CID_PIXEL_RATE reports the line-length clock, so
 * that frame time = (width + hblank) * (height + vblank) / pixel rate is the
 * real frame time. Taking the module clock as the pixel rate would give 0.762
 * of the real frame rate. The link frequency comes from its own control.
 * Per-sensor constant, not stored in the blob.
 */
#define IMX681_PIXCLK_RATIO_NUM	762
#define IMX681_PIXCLK_RATIO_DEN	1000

/* Same order as the Windows power sequence: I/O first, then analog. */
static const char * const imx681_supply_names[] = {
	"dovdd",
	"avdd",
};

#define IMX681_NUM_SUPPLIES	ARRAY_SIZE(imx681_supply_names)

struct imx681_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts;
	u32 vts_min;
	u32 line_clock;
	u32 link_freq_index;
	struct srb_ops ops;
};

struct imx681 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *clk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[IMX681_NUM_SUPPLIES];

	const struct firmware *fw;
	struct srb srb;

	struct imx681_mode *modes;
	unsigned int num_modes;
	const struct imx681_mode *cur_mode;
	u32 mbus_code;
	unsigned int lanes;

	/* Link frequencies of the modes, without duplicates */
	s64 *link_freqs;
	unsigned int num_link_freqs;

	/* Control registers from the blob, indexed by kind */
	struct srb_ctrl ctrl_info[SRB_CTRL_FRAME_LENGTH + 1];
	bool has_ctrl[SRB_CTRL_FRAME_LENGTH + 1];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *again;
	struct v4l2_ctrl *dgain;
};

static inline struct imx681 *to_imx681(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx681, sd);
}

static int imx681_blob_write(void *ctx, u16 reg, u8 val)
{
	struct imx681 *imx681 = ctx;

	return cci_write(imx681->regmap, CCI_REG8(reg), val, NULL);
}

static int imx681_blob_read(void *ctx, u16 reg, u8 *val)
{
	struct imx681 *imx681 = ctx;
	u64 v;
	int ret;

	ret = cci_read(imx681->regmap, CCI_REG8(reg), &v, NULL);
	if (!ret)
		*val = v;

	return ret;
}

static int imx681_write_ctrl(struct imx681 *imx681, unsigned int kind, u32 val)
{
	const struct srb_ctrl *c = &imx681->ctrl_info[kind];
	u32 reg;

	if (!imx681->has_ctrl[kind])
		return 0;

	reg = CCI_REG_ADDR(c->reg) | (c->width << CCI_REG_WIDTH_SHIFT);
	if (c->little_endian)
		reg |= CCI_REG_LE;

	return cci_write(imx681->regmap, reg, val, NULL);
}

static int imx681_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx681 *imx681 = container_of(ctrl->handler, struct imx681,
					     ctrl_handler);
	struct v4l2_subdev_state *state;
	const struct v4l2_mbus_framefmt *fmt;
	int ret = 0;

	state = v4l2_subdev_get_locked_active_state(&imx681->sd);
	fmt = v4l2_subdev_state_get_format(state, 0);

	if (ctrl->id == V4L2_CID_VBLANK) {
		/* The exposure limit follows the frame length */
		s64 max = fmt->height + ctrl->val - 4;

		if (imx681->exposure)
			__v4l2_ctrl_modify_range(imx681->exposure,
						 imx681->exposure->minimum, max,
						 imx681->exposure->step,
						 min_t(s64, max,
						       imx681->exposure->default_value));
	}

	if (!pm_runtime_get_if_in_use(imx681->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = imx681_write_ctrl(imx681, SRB_CTRL_EXPOSURE, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx681_write_ctrl(imx681, SRB_CTRL_ANALOG_GAIN, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = cci_write(imx681->regmap, IMX681_REG_DIGITAL_GAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = imx681_write_ctrl(imx681, SRB_CTRL_FRAME_LENGTH,
					fmt->height + ctrl->val);
		break;
	default:
		break;
	}

	pm_runtime_put(imx681->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx681_ctrl_ops = {
	.s_ctrl = imx681_set_ctrl,
};

static int imx681_init_controls(struct imx681 *imx681)
{
	const struct imx681_mode *mode = imx681->cur_mode;
	struct v4l2_ctrl_handler *hdlr = &imx681->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	const struct srb_ctrl *c;
	int ret;

	ret = v4l2_ctrl_handler_init(hdlr, 10);
	if (ret)
		return ret;

	imx681->link_freq = v4l2_ctrl_new_int_menu(hdlr, &imx681_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   imx681->num_link_freqs - 1,
						   mode->link_freq_index,
						   imx681->link_freqs);
	if (imx681->link_freq)
		imx681->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx681->pixel_rate = v4l2_ctrl_new_std(hdlr, &imx681_ctrl_ops,
					       V4L2_CID_PIXEL_RATE, 1,
					       U32_MAX, 1, mode->line_clock);
	if (imx681->pixel_rate)
		imx681->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx681->hblank = v4l2_ctrl_new_std(hdlr, &imx681_ctrl_ops,
					   V4L2_CID_HBLANK,
					   mode->hts - mode->width,
					   mode->hts - mode->width, 1,
					   mode->hts - mode->width);
	if (imx681->hblank)
		imx681->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Without a frame length register the blanking cannot be changed */
	c = &imx681->ctrl_info[SRB_CTRL_FRAME_LENGTH];
	if (imx681->has_ctrl[SRB_CTRL_FRAME_LENGTH]) {
		imx681->vblank = v4l2_ctrl_new_std(hdlr, &imx681_ctrl_ops,
						   V4L2_CID_VBLANK,
						   mode->vts_min - mode->height,
						   c->max - mode->height, c->step,
						   mode->vts - mode->height);
	} else {
		imx681->vblank = v4l2_ctrl_new_std(hdlr, &imx681_ctrl_ops,
						   V4L2_CID_VBLANK,
						   mode->vts - mode->height,
						   mode->vts - mode->height, 1,
						   mode->vts - mode->height);
		if (imx681->vblank)
			imx681->vblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	}

	c = &imx681->ctrl_info[SRB_CTRL_EXPOSURE];
	if (imx681->has_ctrl[SRB_CTRL_EXPOSURE])
		imx681->exposure = v4l2_ctrl_new_std(hdlr, &imx681_ctrl_ops,
						     V4L2_CID_EXPOSURE, c->min,
						     min_t(u32, c->max,
							   mode->vts - 4),
						     c->step, c->def);

	/* The register comes from the blob, the codes follow the gain law */
	if (imx681->has_ctrl[SRB_CTRL_ANALOG_GAIN]) {
		imx681->again = v4l2_ctrl_new_std(hdlr, &imx681_ctrl_ops,
						  V4L2_CID_ANALOGUE_GAIN,
						  IMX681_AGAIN_MIN,
						  IMX681_AGAIN_MAX,
						  IMX681_AGAIN_STEP,
						  IMX681_AGAIN_DEFAULT);
		imx681->dgain = v4l2_ctrl_new_std(hdlr, &imx681_ctrl_ops,
						  V4L2_CID_DIGITAL_GAIN,
						  IMX681_DGAIN_MIN,
						  IMX681_DGAIN_MAX,
						  IMX681_DGAIN_STEP,
						  IMX681_DGAIN_DEFAULT);
	}

	if (hdlr->error) {
		ret = hdlr->error;
		goto error;
	}

	ret = v4l2_fwnode_device_parse(imx681->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(hdlr, &imx681_ctrl_ops, &props);
	if (ret)
		goto error;

	imx681->sd.ctrl_handler = hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(hdlr);

	return ret;
}

static void imx681_fill_format(const struct imx681 *imx681,
			       const struct imx681_mode *mode,
			       struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = imx681->mbus_code;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int imx681_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx681 *imx681 = to_imx681(sd);
	unsigned int i;
	int ret;

	/* Windows order: reset low, clock, I/O rail, analog rail, reset high */
	gpiod_set_value_cansleep(imx681->reset_gpio, 1);

	ret = clk_prepare_enable(imx681->clk);
	if (ret)
		return ret;

	for (i = 0; i < IMX681_NUM_SUPPLIES; i++) {
		ret = regulator_enable(imx681->supplies[i].consumer);
		if (ret)
			goto disable_supplies;
	}

	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(imx681->reset_gpio, 0);
	usleep_range(10000, 11000);

	return 0;

disable_supplies:
	while (i > 0)
		regulator_disable(imx681->supplies[--i].consumer);
	clk_disable_unprepare(imx681->clk);

	return ret;
}

static int imx681_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx681 *imx681 = to_imx681(sd);
	unsigned int i;

	gpiod_set_value_cansleep(imx681->reset_gpio, 1);
	usleep_range(1000, 2000);
	clk_disable_unprepare(imx681->clk);
	for (i = IMX681_NUM_SUPPLIES; i > 0; i--)
		regulator_disable(imx681->supplies[i - 1].consumer);

	return 0;
}

static int imx681_identify(struct imx681 *imx681)
{
	const struct srb *b = &imx681->srb;
	u32 reg;
	u64 id;
	int ret;

	if (!b->id_width)
		return 0;

	reg = CCI_REG_ADDR(b->id_reg) | (b->id_width << CCI_REG_WIDTH_SHIFT);
	if (b->id_little_endian)
		reg |= CCI_REG_LE;

	ret = cci_read(imx681->regmap, reg, &id, NULL);
	if (ret)
		return dev_err_probe(imx681->dev, ret, "failed to read the chip id\n");

	if (id != b->id_value)
		return dev_err_probe(imx681->dev, -ENODEV,
				     "chip id 0x%llx, the register blob expects 0x%x\n",
				     id, b->id_value);

	return 0;
}

static int imx681_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct imx681 *imx681 = to_imx681(sd);
	const struct v4l2_mbus_framefmt *fmt;
	const struct imx681_mode *mode = imx681->modes;
	unsigned int i;
	int ret;

	fmt = v4l2_subdev_state_get_format(state, 0);
	for (i = 0; i < imx681->num_modes; i++)
		if (imx681->modes[i].width == fmt->width &&
		    imx681->modes[i].height == fmt->height)
			mode = &imx681->modes[i];

	ret = pm_runtime_resume_and_get(imx681->dev);
	if (ret)
		return ret;

	ret = srb_apply(imx681->dev, &imx681->srb.init, imx681_blob_write,
			imx681_blob_read, imx681);
	if (ret) {
		dev_err(imx681->dev, "init sequence failed: %d\n", ret);
		goto err_put;
	}

	ret = srb_apply(imx681->dev, &mode->ops, imx681_blob_write,
			imx681_blob_read, imx681);
	if (ret) {
		dev_err(imx681->dev, "mode sequence failed: %d\n", ret);
		goto err_put;
	}

	ret = __v4l2_ctrl_handler_setup(imx681->sd.ctrl_handler);
	if (ret)
		goto err_put;

	ret = srb_apply(imx681->dev, &imx681->srb.on, imx681_blob_write,
			imx681_blob_read, imx681);
	if (ret) {
		dev_err(imx681->dev, "stream on sequence failed: %d\n", ret);
		goto err_put;
	}

	return 0;

err_put:
	pm_runtime_put(imx681->dev);

	return ret;
}

static int imx681_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct imx681 *imx681 = to_imx681(sd);
	int ret;

	ret = srb_apply(imx681->dev, &imx681->srb.off, imx681_blob_write,
			imx681_blob_read, imx681);
	if (ret)
		dev_err(imx681->dev, "stream off sequence failed: %d\n", ret);

	pm_runtime_put(imx681->dev);

	return ret;
}

static int imx681_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx681 *imx681 = to_imx681(sd);

	if (code->index)
		return -EINVAL;

	code->code = imx681->mbus_code;

	return 0;
}

static int imx681_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx681 *imx681 = to_imx681(sd);

	if (fse->index >= imx681->num_modes || fse->code != imx681->mbus_code)
		return -EINVAL;

	fse->min_width = imx681->modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = imx681->modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int imx681_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx681 *imx681 = to_imx681(sd);
	const struct imx681_mode *mode;
	struct v4l2_mbus_framefmt *format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	mode = v4l2_find_nearest_size(imx681->modes, imx681->num_modes, width,
				      height, fmt->format.width,
				      fmt->format.height);
	imx681_fill_format(imx681, mode, &fmt->format);

	format = v4l2_subdev_state_get_format(state, 0);
	*format = fmt->format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		const struct srb_ctrl *c = &imx681->ctrl_info[SRB_CTRL_FRAME_LENGTH];
		s64 vblank_max = mode->vts - mode->height;

		imx681->cur_mode = mode;
		__v4l2_ctrl_s_ctrl(imx681->link_freq, mode->link_freq_index);
		__v4l2_ctrl_s_ctrl_int64(imx681->pixel_rate, mode->line_clock);
		__v4l2_ctrl_modify_range(imx681->hblank, mode->hts - mode->width,
					 mode->hts - mode->width, 1,
					 mode->hts - mode->width);
		if (imx681->has_ctrl[SRB_CTRL_FRAME_LENGTH])
			vblank_max = c->max - mode->height;
		__v4l2_ctrl_modify_range(imx681->vblank,
					 mode->vts_min - mode->height,
					 vblank_max, imx681->vblank->step,
					 mode->vts - mode->height);
		__v4l2_ctrl_s_ctrl(imx681->vblank, mode->vts - mode->height);
	}

	return 0;
}

static int imx681_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	struct imx681 *imx681 = to_imx681(sd);
	unsigned int i, w = 0, h = 0;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = v4l2_subdev_state_get_format(state, 0)->width;
		sel->r.height = v4l2_subdev_state_get_format(state, 0)->height;
		return 0;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		/* The array size is not known: use the largest mode */
		for (i = 0; i < imx681->num_modes; i++) {
			w = max(w, imx681->modes[i].width);
			h = max(h, imx681->modes[i].height);
		}
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = w;
		sel->r.height = h;
		return 0;
	}

	return -EINVAL;
}

static int imx681_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_config *cfg)
{
	struct imx681 *imx681 = to_imx681(sd);

	cfg->type = V4L2_MBUS_CSI2_CPHY;
	cfg->bus.mipi_csi2.num_data_lanes = imx681->lanes;

	return 0;
}

static int imx681_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct imx681 *imx681 = to_imx681(sd);

	imx681_fill_format(imx681, imx681->modes,
			   v4l2_subdev_state_get_format(state, 0));

	return 0;
}

static const struct v4l2_subdev_video_ops imx681_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx681_pad_ops = {
	.enum_mbus_code = imx681_enum_mbus_code,
	.enum_frame_size = imx681_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx681_set_fmt,
	.get_selection = imx681_get_selection,
	.get_mbus_config = imx681_get_mbus_config,
	.enable_streams = imx681_enable_streams,
	.disable_streams = imx681_disable_streams,
};

static const struct v4l2_subdev_ops imx681_subdev_ops = {
	.video = &imx681_video_ops,
	.pad = &imx681_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx681_internal_ops = {
	.init_state = imx681_init_state,
};

static const struct media_entity_operations imx681_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* Check the endpoint: C-PHY, at most three trios, the link frequencies */
static int imx681_check_endpoint(struct imx681 *imx681, u64 **link_freqs,
				 unsigned int *num_link_freqs)
{
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_CPHY,
	};
	struct fwnode_handle *fwnode;
	int ret;

	fwnode = fwnode_graph_get_next_endpoint(dev_fwnode(imx681->dev), NULL);
	if (!fwnode)
		return dev_err_probe(imx681->dev, -EINVAL, "missing endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(fwnode, &ep);
	fwnode_handle_put(fwnode);
	if (ret)
		return ret;

	if (!ep.bus.mipi_csi2.num_data_lanes ||
	    ep.bus.mipi_csi2.num_data_lanes > 3) {
		ret = dev_err_probe(imx681->dev, -EINVAL,
				    "unsupported number of trios %u\n",
				    ep.bus.mipi_csi2.num_data_lanes);
		goto out;
	}
	imx681->lanes = ep.bus.mipi_csi2.num_data_lanes;

	*link_freqs = devm_kmemdup(imx681->dev, ep.link_frequencies,
				   array_size(ep.nr_of_link_frequencies,
					      sizeof(u64)), GFP_KERNEL);
	if (!*link_freqs)
		ret = -ENOMEM;
	*num_link_freqs = ep.nr_of_link_frequencies;
out:
	v4l2_fwnode_endpoint_free(&ep);

	return ret;
}

/*
 * Build the mode table from the blob. A blob is accepted only if all modes
 * use the same media bus code, a Bayer RAW10 code, a trio count the endpoint
 * offers, a link frequency the endpoint lists, and differ in size, so that a
 * frame size selects one mode.
 */
static int imx681_setup_modes(struct imx681 *imx681, const u64 *dt_freqs,
			      unsigned int num_dt_freqs)
{
	struct device *dev = imx681->dev;
	const struct srb *b = &imx681->srb;
	unsigned int i, j, k;

	if (!b->n_modes)
		return dev_err_probe(dev, -EINVAL, "register blob has no mode\n");

	imx681->modes = devm_kcalloc(dev, b->n_modes, sizeof(*imx681->modes),
				     GFP_KERNEL);
	imx681->link_freqs = devm_kcalloc(dev, b->n_modes,
					  sizeof(*imx681->link_freqs),
					  GFP_KERNEL);
	if (!imx681->modes || !imx681->link_freqs)
		return -ENOMEM;

	for (i = 0; i < b->n_modes; i++) {
		struct imx681_mode *mode = &imx681->modes[i];
		struct srb_mode m;

		srb_get_mode(b, i, &m);

		switch (m.mbus_code) {
		case MEDIA_BUS_FMT_SBGGR10_1X10:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
		case MEDIA_BUS_FMT_SGRBG10_1X10:
		case MEDIA_BUS_FMT_SRGGB10_1X10:
			break;
		default:
			return dev_err_probe(dev, -EINVAL,
					     "mode %u: media bus code 0x%x is not a RAW10 Bayer code\n",
					     i, m.mbus_code);
		}
		if (i && m.mbus_code != imx681->mbus_code)
			return dev_err_probe(dev, -EINVAL,
					     "mode %u: the modes use different bus codes\n",
					     i);
		imx681->mbus_code = m.mbus_code;

		if (m.lanes != imx681->lanes)
			return dev_err_probe(dev, -EINVAL,
					     "mode %u needs %u trios, the endpoint has %u\n",
					     i, m.lanes, imx681->lanes);

		for (j = 0; j < i; j++)
			if (imx681->modes[j].width == m.width &&
			    imx681->modes[j].height == m.height)
				return dev_err_probe(dev, -EINVAL,
						     "modes %u and %u have the same size\n",
						     j, i);

		for (j = 0; j < num_dt_freqs; j++)
			if (dt_freqs[j] == m.link_freq)
				break;
		if (j == num_dt_freqs)
			return dev_err_probe(dev, -EINVAL,
					     "mode %u: link frequency %u is not in the endpoint\n",
					     i, m.link_freq);

		for (k = 0; k < imx681->num_link_freqs; k++)
			if (imx681->link_freqs[k] == m.link_freq)
				break;
		if (k == imx681->num_link_freqs)
			imx681->link_freqs[imx681->num_link_freqs++] = m.link_freq;

		mode->width = m.width;
		mode->height = m.height;
		mode->hts = m.hts;
		mode->vts = m.vts;
		mode->vts_min = m.vts_min;
		mode->line_clock = div_u64((u64)m.pixel_rate *
					   IMX681_PIXCLK_RATIO_DEN,
					   IMX681_PIXCLK_RATIO_NUM);
		if (!m.pixel_rate || (u64)m.pixel_rate * IMX681_PIXCLK_RATIO_DEN >
		    (u64)U32_MAX * IMX681_PIXCLK_RATIO_NUM)
			return dev_err_probe(dev, -EINVAL,
					     "mode %u: pixel rate %u out of range\n",
					     i, m.pixel_rate);
		mode->link_freq_index = k;
		mode->ops = m.ops;
		imx681->num_modes++;
	}

	for (i = 0; i < b->n_ctrls; i++) {
		struct srb_ctrl c;

		srb_get_ctrl(b, i, &c);
		imx681->ctrl_info[c.kind] = c;
		imx681->has_ctrl[c.kind] = true;
	}

	return 0;
}

static int imx681_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx681 *imx681;
	unsigned int num_dt_freqs;
	u64 *dt_freqs;
	const char *name;
	unsigned long freq;
	unsigned int i;
	int ret;

	imx681 = devm_kzalloc(dev, sizeof(*imx681), GFP_KERNEL);
	if (!imx681)
		return -ENOMEM;
	imx681->dev = dev;

	ret = device_property_read_string(dev, "firmware-name", &name);
	if (ret)
		return dev_err_probe(dev, ret, "missing firmware-name\n");

	/* Nothing touches the hardware before the register blob is valid */
	ret = request_firmware(&imx681->fw, name, dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "cannot load the register blob %s\n", name);

	ret = srb_parse(dev, imx681->fw, &imx681->srb);
	if (ret)
		goto err_release_fw;

	ret = imx681_check_endpoint(imx681, &dt_freqs, &num_dt_freqs);
	if (ret)
		goto err_release_fw;

	ret = imx681_setup_modes(imx681, dt_freqs, num_dt_freqs);
	if (ret)
		goto err_release_fw;

	imx681->clk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(imx681->clk)) {
		ret = dev_err_probe(dev, PTR_ERR(imx681->clk),
				    "failed to get the clock\n");
		goto err_release_fw;
	}

	freq = clk_get_rate(imx681->clk);
	if (freq != IMX681_XCLK_FREQ) {
		ret = dev_err_probe(dev, -EINVAL,
				    "external clock %lu is not supported\n", freq);
		goto err_release_fw;
	}

	for (i = 0; i < IMX681_NUM_SUPPLIES; i++)
		imx681->supplies[i].supply = imx681_supply_names[i];
	ret = devm_regulator_bulk_get(dev, IMX681_NUM_SUPPLIES, imx681->supplies);
	if (ret) {
		dev_err_probe(dev, ret, "failed to get the supplies\n");
		goto err_release_fw;
	}

	imx681->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(imx681->reset_gpio)) {
		ret = dev_err_probe(dev, PTR_ERR(imx681->reset_gpio),
				    "failed to get the reset GPIO\n");
		goto err_release_fw;
	}

	imx681->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx681->regmap)) {
		ret = PTR_ERR(imx681->regmap);
		goto err_release_fw;
	}

	v4l2_i2c_subdev_init(&imx681->sd, client, &imx681_subdev_ops);
	imx681->cur_mode = imx681->modes;

	ret = imx681_power_on(dev);
	if (ret) {
		dev_err_probe(dev, ret, "failed to power on\n");
		goto err_release_fw;
	}

	ret = imx681_identify(imx681);
	if (ret)
		goto err_power_off;

	ret = imx681_init_controls(imx681);
	if (ret)
		goto err_power_off;

	imx681->sd.internal_ops = &imx681_internal_ops;
	imx681->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx681->sd.entity.ops = &imx681_subdev_entity_ops;
	imx681->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx681->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&imx681->sd.entity, 1, &imx681->pad);
	if (ret)
		goto err_free_ctrls;

	imx681->sd.state_lock = imx681->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&imx681->sd);
	if (ret)
		goto err_entity_cleanup;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&imx681->sd);
	if (ret)
		goto err_subdev_cleanup;

	pm_runtime_idle(dev);

	return 0;

err_subdev_cleanup:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&imx681->sd);
err_entity_cleanup:
	media_entity_cleanup(&imx681->sd.entity);
err_free_ctrls:
	v4l2_ctrl_handler_free(&imx681->ctrl_handler);
err_power_off:
	imx681_power_off(dev);
err_release_fw:
	release_firmware(imx681->fw);

	return ret;
}

static void imx681_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx681 *imx681 = to_imx681(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(imx681->dev);
	if (!pm_runtime_status_suspended(imx681->dev))
		imx681_power_off(imx681->dev);
	pm_runtime_set_suspended(imx681->dev);

	release_firmware(imx681->fw);
}

static const struct dev_pm_ops imx681_pm_ops = {
	SET_RUNTIME_PM_OPS(imx681_power_off, imx681_power_on, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

static const struct of_device_id imx681_of_match[] = {
	{ .compatible = "sony,imx681" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx681_of_match);

static struct i2c_driver imx681_i2c_driver = {
	.driver = {
		.name = "imx681",
		.of_match_table = imx681_of_match,
		.pm = &imx681_pm_ops,
	},
	.probe = imx681_probe,
	.remove = imx681_remove,
};
module_i2c_driver(imx681_i2c_driver);

MODULE_DESCRIPTION("Sony IMX681 image sensor driver");
MODULE_LICENSE("GPL");
