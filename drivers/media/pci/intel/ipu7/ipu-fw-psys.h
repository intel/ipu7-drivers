/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2016 - 2024 Intel Corporation */

#ifndef IPU_FW_PSYS_H
#define IPU_FW_PSYS_H

#include "ipu-syscom.h"
#include "ia_gofo_common_abi.h"
#include "ipu_msg_task.h"
#include "ipu_msg_graph.h"
#include "ipu_msg_state.h"
#include "ipu_msg_header.h"
#include "ipu_msg_device.h"
#include "fwps_msg_q_defs.h"

#include <uapi/linux/ipu-psys.h>

// TODO: use FW configuration replace it
#define IPU_PSYS_MAX_GRAPH_NUMS		(8U)

struct ipu_msg_to_str {
	const enum ipu_msg_type type;
	const char *msg;
};

struct ipu_psys;
struct ipu_psys_stream;
struct ipu_psys_task_queue;

int ipu_fw_psys_init(struct ipu_psys *psys);
void ipu_fw_psys_release(struct ipu_psys *psys);
int ipu_fw_psys_open(struct ipu_psys *psys);
void ipu_fw_psys_close(struct ipu_psys *psys);
int ipu_fw_psys_graph_open(const struct ipu_psys_graph_info *graph,
			   struct ipu_psys *psys, struct ipu_psys_stream *ip);
int ipu_fw_psys_graph_close(u8 graph_id, struct ipu_psys *psys);
int ipu_fw_psys_task_request(const struct ipu_psys_task_request *task,
			     struct ipu_psys_stream *ip,
			     struct ipu_psys_task_queue *tq,
			     struct ipu_psys *psys);
int ipu_fw_psys_event_handle(struct ipu_psys *psys, u8 *buf_ptr);
int ipu_fw_psys_get_log(struct ipu_psys *psys);
#endif /* IPU_FW_PSYS_H */
