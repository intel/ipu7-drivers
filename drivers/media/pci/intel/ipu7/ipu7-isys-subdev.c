// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013 - 2024 Intel Corporation
 */

#include <linux/bug.h>
#include <linux/device.h>
#include <linux/minmax.h>
#include <linux/types.h>
#include <linux/version.h>

#include <media/media-entity.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0)
#include <media/mipi-csi2.h>
#endif
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#include <uapi/linux/media-bus-format.h>

#include "ipu7-bus.h"
#include "ipu7-isys.h"
#include "ipu7-isys-subdev.h"

unsigned int ipu7_isys_mbus_code_to_bpp(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_RGB888_1X24:
		return 24;
	case MEDIA_BUS_FMT_YUYV10_1X20:
		return 20;
	case MEDIA_BUS_FMT_Y10_1X10:
	case MEDIA_BUS_FMT_RGB565_1X16:
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_YUYV8_1X16:
		return 16;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		return 12;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return 10;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		return 8;
	default:
		WARN_ON(1);
		return -EINVAL;
	}
}

unsigned int ipu7_isys_mbus_code_to_data_type(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_RGB565_1X16:
		return MIPI_CSI2_TYPE_RGB565;
	case MEDIA_BUS_FMT_RGB888_1X24:
		return MIPI_CSI2_TYPE_RGB888;
	case MEDIA_BUS_FMT_YUYV10_1X20:
		return MIPI_CSI2_TYPE_YUV422_10;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_YUYV8_1X16:
		return MIPI_CSI2_TYPE_YUV422_8;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		return MIPI_CSI2_TYPE_RAW12;
	case MEDIA_BUS_FMT_Y10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return MIPI_CSI2_TYPE_RAW10;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		return MIPI_CSI2_TYPE_RAW8;
	default:
		WARN_ON(1);
		return -EINVAL;
	}
}

bool ipu7_isys_is_bayer_format(u32 code)
{
	switch (ipu7_isys_mbus_code_to_data_type(code)) {
	case MIPI_CSI2_TYPE_RAW8:
	case MIPI_CSI2_TYPE_RAW10:
	case MIPI_CSI2_TYPE_RAW12:
		return true;
	}
	return false;
}

u32 ipu7_isys_convert_bayer_order(u32 code, int x, int y)
{
	static const u32 code_map[] = {
		MEDIA_BUS_FMT_SRGGB8_1X8,
		MEDIA_BUS_FMT_SGRBG8_1X8,
		MEDIA_BUS_FMT_SGBRG8_1X8,
		MEDIA_BUS_FMT_SBGGR8_1X8,
		MEDIA_BUS_FMT_SRGGB10_1X10,
		MEDIA_BUS_FMT_SGRBG10_1X10,
		MEDIA_BUS_FMT_SGBRG10_1X10,
		MEDIA_BUS_FMT_SBGGR10_1X10,
		MEDIA_BUS_FMT_SRGGB12_1X12,
		MEDIA_BUS_FMT_SGRBG12_1X12,
		MEDIA_BUS_FMT_SGBRG12_1X12,
		MEDIA_BUS_FMT_SBGGR12_1X12,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(code_map); i++)
		if (code_map[i] == code)
			break;

	if (i == ARRAY_SIZE(code_map)) {
		WARN_ON(1);
		return code;
	}

	return code_map[i ^ (((y & 1) << 1) | (x & 1))];
}

struct v4l2_mbus_framefmt *ipu7_isys_get_ffmt(struct v4l2_subdev *sd,
					      struct v4l2_subdev_state *state,
					      unsigned int pad,
					      unsigned int which)
{
	struct ipu7_isys_subdev *asd = to_ipu7_isys_subdev(sd);

	if (which == V4L2_SUBDEV_FORMAT_ACTIVE)
		return &asd->ffmt[pad];
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	return v4l2_subdev_state_get_format(state, pad);
#else
	return v4l2_subdev_get_try_format(sd, state, pad);
#endif
}

struct v4l2_rect *ipu7_isys_get_crop(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     unsigned int pad,
				     unsigned int which)
{
	struct ipu7_isys_subdev *asd = to_ipu7_isys_subdev(sd);

	if (which == V4L2_SUBDEV_FORMAT_ACTIVE)
		return &asd->crop;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	return v4l2_subdev_state_get_crop(state, pad);
#else
	return v4l2_subdev_get_try_crop(sd, state, pad);
#endif
}

int ipu7_isys_subdev_set_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *format)
{
	struct ipu7_isys_subdev *asd = to_ipu7_isys_subdev(sd);
	struct v4l2_mbus_framefmt *fmt =
		ipu7_isys_get_ffmt(sd, state, format->pad, format->which);
	struct v4l2_mbus_framefmt *source_fmt;
	u32 code = asd->supported_codes[0];
	unsigned int i;

	dev_dbg(&asd->isys->adev->auxdev.dev, "set format for %s pad %d.\n",
		sd->name, format->pad);
	mutex_lock(&asd->mutex);
	/* No transcoding, source and sink formats must match. */
	if ((sd->entity.pads[format->pad].flags & MEDIA_PAD_FL_SOURCE) &&
	    sd->entity.num_pads > 1) {
		format->format = *fmt;
		mutex_unlock(&asd->mutex);
		return 0;
	}

	format->format.width = clamp(format->format.width, IPU_ISYS_MIN_WIDTH,
				     IPU_ISYS_MAX_WIDTH);
	format->format.height = clamp(format->format.height,
				      IPU_ISYS_MIN_HEIGHT, IPU_ISYS_MAX_HEIGHT);
	for (i = 0; asd->supported_codes[i]; i++) {
		if (asd->supported_codes[i] == format->format.code) {
			code = asd->supported_codes[i];
			break;
		}
	}
	format->format.code = code;
	format->format.field = V4L2_FIELD_NONE;
	*fmt = format->format;
	if (sd->entity.pads[format->pad].flags & MEDIA_PAD_FL_SINK) {
		/* propagate format to following source pad */
		struct v4l2_rect *crop =
			ipu7_isys_get_crop(sd, state, format->pad + 1,
					   format->which);

		dev_dbg(&asd->isys->adev->auxdev.dev,
			"propagate format to source pad.\n");
		source_fmt = ipu7_isys_get_ffmt(sd, state, format->pad + 1,
						format->which);
		*source_fmt = format->format;
		/* reset crop */
		crop->left = 0;
		crop->top = 0;
		crop->width = fmt->width;
		crop->height = fmt->height;
	}

	mutex_unlock(&asd->mutex);

	return 0;
}

int ipu7_isys_subdev_get_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	struct ipu7_isys_subdev *asd = to_ipu7_isys_subdev(sd);

	mutex_lock(&asd->mutex);
	fmt->format = *ipu7_isys_get_ffmt(sd, state, fmt->pad,
					  fmt->which);
	mutex_unlock(&asd->mutex);

	return 0;
}

int ipu7_isys_subdev_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	struct ipu7_isys_subdev *asd = to_ipu7_isys_subdev(sd);
	const u32 *supported_codes = asd->supported_codes;
	u32 index;

	for (index = 0; supported_codes[index]; index++) {
		if (index == code->index) {
			code->code = supported_codes[index];
			return 0;
		}
	}

	return -EINVAL;
}

/*
 * Besides validating the link, figure out the external pad and the
 * ISYS FW ABI source.
 */
int ipu7_isys_subdev_link_validate(struct v4l2_subdev *sd,
				   struct media_link *link,
				   struct v4l2_subdev_format *source_fmt,
				   struct v4l2_subdev_format *sink_fmt)
{
	struct v4l2_subdev *source_sd =
		media_entity_to_v4l2_subdev(link->source->entity);
	struct ipu7_isys_stream *stream =
		to_ipu7_isys_pipeline(media_entity_pipeline(&sd->entity));
	struct device *dev = &stream->isys->adev->auxdev.dev;

	if (!source_sd)
		return -ENODEV;
	if (strncmp(source_sd->name, IPU_ISYS_ENTITY_PREFIX,
		    strlen(IPU_ISYS_ENTITY_PREFIX)) != 0) {
		/*
		 * source_sd isn't ours --- sd must be the external
		 * sub-device.
		 */
		stream->external = link->source;
		stream->stream_source = to_ipu7_isys_subdev(sd)->source;
		dev_dbg(dev, "%s: using source %d\n",
			sd->entity.name, stream->stream_source);
	} else if (source_sd->entity.num_pads == 1) {
		/* All internal sources have a single pad. */
		stream->external = link->source;
		stream->stream_source = to_ipu7_isys_subdev(source_sd)->source;

		dev_dbg(dev, "%s: using source %d\n",
			sd->entity.name, stream->stream_source);
	}

	return v4l2_subdev_link_validate_default(sd, link, source_fmt,
						 sink_fmt);
}

int ipu7_isys_subdev_init(struct ipu7_isys_subdev *asd,
			  const struct v4l2_subdev_ops *ops,
			  unsigned int nr_ctrls,
			  unsigned int num_sink_pads,
			  unsigned int num_source_pads)
{
	unsigned int num_pads = num_sink_pads + num_source_pads;
	unsigned int i;
	int ret;

	mutex_init(&asd->mutex);

	v4l2_subdev_init(&asd->sd, ops);

	asd->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		V4L2_SUBDEV_FL_HAS_EVENTS;
	asd->sd.owner = THIS_MODULE;
	asd->sd.dev = &asd->isys->adev->auxdev.dev;
	asd->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;

	asd->pad = devm_kcalloc(&asd->isys->adev->auxdev.dev, num_pads,
				sizeof(*asd->pad), GFP_KERNEL);
	asd->ffmt = devm_kcalloc(&asd->isys->adev->auxdev.dev, num_pads,
				 sizeof(*asd->ffmt), GFP_KERNEL);
	if (!asd->pad || !asd->ffmt)
		return -ENOMEM;

	for (i = 0; i < num_sink_pads; i++)
		asd->pad[i].flags = MEDIA_PAD_FL_SINK |
			MEDIA_PAD_FL_MUST_CONNECT;

	for (i = num_sink_pads; i < num_pads; i++)
		asd->pad[i].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&asd->sd.entity, num_pads, asd->pad);
	if (ret) {
		pr_err("isys subdev init failed %d.\n", ret);
		goto out_mutex_destroy;
	}

	if (asd->ctrl_init) {
		ret = v4l2_ctrl_handler_init(&asd->ctrl_handler, nr_ctrls);
		if (ret)
			goto out_media_entity_cleanup;

		asd->ctrl_init(&asd->sd);
		if (asd->ctrl_handler.error) {
			ret = asd->ctrl_handler.error;
			goto out_v4l2_ctrl_handler_free;
		}

		asd->sd.ctrl_handler = &asd->ctrl_handler;
	}

	return 0;

out_v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(&asd->ctrl_handler);

out_media_entity_cleanup:
	media_entity_cleanup(&asd->sd.entity);

out_mutex_destroy:
	mutex_destroy(&asd->mutex);

	return ret;
}

void ipu7_isys_subdev_cleanup(struct ipu7_isys_subdev *asd)
{
	media_entity_cleanup(&asd->sd.entity);
	v4l2_ctrl_handler_free(&asd->ctrl_handler);
	mutex_destroy(&asd->mutex);
}
