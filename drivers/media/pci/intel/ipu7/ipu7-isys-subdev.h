/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2013 - 2024 Intel Corporation
 */

#ifndef IPU7_ISYS_SUBDEV_H
#define IPU7_ISYS_SUBDEV_H

#include <linux/container_of.h>
#include <linux/mutex.h>

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

struct ipu7_isys;

#define MIPI_CSI2_TYPE_NULL		0x10
#define MIPI_CSI2_TYPE_BLANKING		0x11
#define MIPI_CSI2_TYPE_EMBEDDED8	0x12
#define MIPI_CSI2_TYPE_YUV422_8		0x1e
#define MIPI_CSI2_TYPE_YUV422_10	0x1f
#define MIPI_CSI2_TYPE_RGB565		0x22
#define MIPI_CSI2_TYPE_RGB888		0x24
#define MIPI_CSI2_TYPE_RAW6		0x28
#define MIPI_CSI2_TYPE_RAW7		0x29
#define MIPI_CSI2_TYPE_RAW8		0x2a
#define MIPI_CSI2_TYPE_RAW10		0x2b
#define MIPI_CSI2_TYPE_RAW12		0x2c
#define MIPI_CSI2_TYPE_RAW14		0x2d

struct ipu7_isys_subdev {
	/* Serialise access to any other field in the struct */
	struct mutex mutex;
	struct v4l2_subdev sd;
	struct ipu7_isys *isys;
	u32 const *supported_codes;
	struct media_pad *pad;
	struct v4l2_mbus_framefmt *ffmt;
	struct v4l2_rect crop;
	struct v4l2_ctrl_handler ctrl_handler;
	void (*ctrl_init)(struct v4l2_subdev *sd);
	int source;	/* SSI stream source; -1 if unset */
#ifdef CONFIG_VIDEO_INTEL_IPU7_MGC
	bool is_tpg;
#endif
};

#define to_ipu7_isys_subdev(__sd)			\
	container_of(__sd, struct ipu7_isys_subdev, sd)

struct v4l2_mbus_framefmt *ipu7_isys_get_ffmt(struct v4l2_subdev *sd,
					      struct v4l2_subdev_state *state,
					      unsigned int pad,
					      unsigned int which);

unsigned int ipu7_isys_mbus_code_to_bpp(u32 code);
unsigned int ipu7_isys_mbus_code_to_data_type(u32 code);
bool ipu7_isys_is_bayer_format(u32 code);
u32 ipu7_isys_convert_bayer_order(u32 code, int x, int y);

int ipu7_isys_subdev_set_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt);
int ipu7_isys_subdev_get_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt);
struct v4l2_rect *ipu7_isys_get_crop(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     unsigned int pad,
				     unsigned int which);
int ipu7_isys_subdev_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_mbus_code_enum
				    *code);
int ipu7_isys_subdev_link_validate(struct v4l2_subdev *sd,
				   struct media_link *link,
				   struct v4l2_subdev_format *source_fmt,
				   struct v4l2_subdev_format *sink_fmt);

int ipu7_isys_subdev_init(struct ipu7_isys_subdev *asd,
			  const struct v4l2_subdev_ops *ops,
			  unsigned int nr_ctrls,
			  unsigned int num_sink_pads,
			  unsigned int num_source_pads);
void ipu7_isys_subdev_cleanup(struct ipu7_isys_subdev *asd);
#endif /* IPU7_ISYS_SUBDEV_H */
