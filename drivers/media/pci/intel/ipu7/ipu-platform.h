/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2013 - 2024 Intel Corporation */

#ifndef IPU_PLATFORM_H
#define IPU_PLATFORM_H

#include "ipu-fw-isys.h"

#define IPU_NAME			"intel-ipu7"

#define IPU7_FIRMWARE_NAME		"intel/ipu7_fw.bin"
#define IPU7P5_FIRMWARE_NAME		"intel/ipu7ptl_fw.bin"

/*
 * The following definitions are encoded to the media_device's model field so
 * that the software components which uses IPU driver can get the hw stepping
 * information.
 */
#define IPU_MEDIA_DEV_MODEL_NAME		"ipu7"

#define IPU7_ISYS_NUM_STREAMS            IPU7_NONSECURE_STREAM_ID_MAX

extern struct ipu_isys_internal_pdata ipu7_isys_ipdata;
extern struct ipu_psys_internal_pdata ipu7_psys_ipdata;
extern struct ipu_isys_internal_pdata ipu7p5_isys_ipdata;
extern struct ipu_psys_internal_pdata ipu7p5_psys_ipdata;
extern const struct ipu_buttress_ctrl isys_buttress_ctrl;
extern const struct ipu_buttress_ctrl psys_buttress_ctrl;

#endif
