/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2013 - 2024 Intel Corporation */

#ifndef IPU_ISYS_TPG_H
#define IPU_ISYS_TPG_H

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#include "ipu-isys-subdev.h"
#include "ipu-isys-video.h"
#include "ipu-isys-queue.h"

struct ipu_isys_tpg_pdata;
struct ipu_isys;

#define TPG_PAD_SOURCE			0
#define NR_OF_TPG_PADS			1
#define NR_OF_TPG_SOURCE_PADS		1
#define NR_OF_TPG_SINK_PADS		0
#define NR_OF_TPG_STREAMS		1

enum isys_tpg_mode {
	TPG_MODE_RAMP = 0,
	TPG_MODE_CHECKERBOARD = 1,
	TPG_MODE_MONO = 2,
	TPG_MODE_COLOR_PALETTE = 3,
};

/*
 * struct ipu_isys_tpg
 *
 * @nlanes: number of lanes in the receiver
 */
struct ipu_isys_tpg {
	struct ipu_isys_tpg_pdata *pdata;
	struct ipu_isys *isys;
	struct ipu_isys_subdev asd;

	/* MG base not MGC */
	void __iomem *base;
	void __iomem *sel;
	unsigned int index;

	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *pixel_rate;
};

#define to_ipu_isys_tpg(sd)		\
	container_of(to_ipu_isys_subdev(sd), \
	struct ipu_isys_tpg, asd)

void ipu_isys_tpg_sof_event(struct ipu_isys_tpg *tpg);
void ipu_isys_tpg_eof_event(struct ipu_isys_tpg *tpg);
int ipu_isys_tpg_init(struct ipu_isys_tpg *tpg,
		      struct ipu_isys *isys,
		      void __iomem *base, void __iomem *sel,
		      unsigned int index);
void ipu_isys_tpg_cleanup(struct ipu_isys_tpg *tpg);
int tpg_set_stream(struct v4l2_subdev *sd, int enable);

#endif /* IPU_ISYS_TPG_H */
