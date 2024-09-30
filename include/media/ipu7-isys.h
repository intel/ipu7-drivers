/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2014 - 2022 Intel Corporation */

#ifndef MEDIA_IPU_ISYS_H
#define MEDIA_IPU_ISYS_H

#include <linux/i2c.h>
#include <linux/clkdev.h>
#include <media/v4l2-async.h>

struct ipu7_isys_csi2_config {
	unsigned int nlanes;
	unsigned int port;
};

struct ipu7_isys_subdev_i2c_info {
	struct i2c_board_info board_info;
	int i2c_adapter_id;
	char i2c_adapter_bdf[32];
};

struct ipu7_isys_subdev_info {
	struct ipu7_isys_csi2_config *csi2;
	struct ipu7_isys_subdev_i2c_info i2c;
};

struct ipu7_isys_clk_mapping {
	struct clk_lookup clkdev_data;
	char *platform_clock_name;
};

struct ipu7_isys_subdev_pdata {
	struct ipu7_isys_subdev_info **subdevs;
	struct ipu7_isys_clk_mapping *clk_map;
};

struct sensor_async_sd {
	struct v4l2_async_connection asc;
	struct ipu7_isys_csi2_config csi2;
};

#endif /* MEDIA_IPU7_ISYS_H */
