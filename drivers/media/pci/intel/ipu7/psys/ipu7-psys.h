/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2013 - 2024 Intel Corporation */

#ifndef IPU7_PSYS_H
#define IPU7_PSYS_H

#include <linux/cdev.h>
#include <linux/workqueue.h>
#include <linux/completion.h>

#include "ipu7.h"
#include "ipu7-fw-psys.h"

#define IPU_PSYS_WORK_QUEUE		system_power_efficient_wq

#define IPU_PSYS_CMD_QUEUE_SIZE      0x20
#define IPU_PSYS_TASK_QUEUE_SIZE     0x20
#define IPU_PSYS_ACK_QUEUE_SIZE      0x40
#define IPU_PSYS_LOG_QUEUE_SIZE      256
#define IPU_PSYS_OUT_MSG_SIZE        256

/**
 * Each event from FW will be first queued into a
 * event queue, define the queue depth here
 */
#define MAX_TASK_EVENT_QUEUE_SIZE		3
/**
 * Each task queue from user will be first queued into
 * a task queue, define the queue depth here
 */
#define MAX_TASK_REQUEST_QUEUE_SIZE		8

// TODO: use FW configuration to replace it
#define IPU_PSYS_NUM_STREAMS			8

#define INVALID_STREAM_ID			0xFF
/**
 * Task request queues per stream
 *
 * Each task will first assigned a task queue buffer here,
 * all the nodes will share the same task queue, maximum
 * queue will be full there.
 */
struct ipu_psys_task_queue {
	struct ipu_psys_term_buffers task_buffers[MAX_GRAPH_TERMINALS];
	dma_addr_t ipu7_addr[MAX_GRAPH_TERMINALS];
	u32 index; /* index of the task request buffer */
	u8 available; /* if the task queue can be used or not */
	struct ipu7_msg_task *msg_task;
	dma_addr_t task_dma_addr;
	/* task state of each task input, represent ipu7_msg_task_state */
	enum ipu7_msg_task_state task_state;
};

struct psys_fw_log {
	struct mutex mutex; /* protect whole struct */
	void *head;
	void *addr;
	u32 count; /* running counter of log */
	u32 size; /* actual size of log content, in bits */
};

/**
 * Task quest event context
 *
 * Each task request should get its event ack from FW and save
 * to this structure and for user dequeue purpose.
 */
struct ipu_psys_task_ack {
	u8 graph_id; /* graph id of the task request */
	u8 node_ctx_id; /* logical node id */
	u8 frame_id; /* frame id of the original task request */
	u8 available; /* if the task ack slot can be used */

	u32 err_code; /* error indication to user */
};

/**
 * stream here is equal to pipe, each stream has
 * its dedicated graph_id, and task request queue.
 *
 * For multiple stream supported design.
 */
struct ipu7_psys_stream {
	struct ipu7_psys_fh *fh;

	u8 graph_id; /* graph_id on this stream */

	/* Serialize task done queue */
	struct mutex event_mutex;
	/* current event queue write index, incremental */
	u32 event_write_index;
	/* current event queue read index, incremental */
	u32 event_read_index;
	struct ipu_psys_task_ack event_queue[MAX_TASK_EVENT_QUEUE_SIZE];

	/* Serialize task queue */
	struct mutex task_mutex;
	struct ipu_psys_task_queue task_queue[MAX_TASK_REQUEST_QUEUE_SIZE];

	u8 num_nodes; /* Number of enabled nodes */
	struct graph_node nodes[MAX_GRAPH_NODES];
	u8 q_id[MAX_GRAPH_NODES]; /* syscom input queue id assigned by fw */

	struct completion graph_open;
	struct completion graph_close;

	/* Graph state, represent enum ipu7_msg_graph_state */
	enum ipu7_msg_graph_state graph_state;
};

struct task_struct;
struct ipu7_psys {
	struct ipu_psys_capability caps;
	struct cdev cdev;
	struct device dev;

	struct mutex mutex;	/* Psys various */
	int ready; /* psys fw status */
	bool icache_prefetch_sp;
	bool icache_prefetch_isp;
	spinlock_t ready_lock;	/* protect psys firmware state */

	struct list_head fhs;

	struct ipu7_psys_pdata *pdata;
	struct ipu7_bus_device *adev;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfsdir;
#endif

	unsigned long timeout;

	struct psys_fw_log *fw_log;

	int power_gating;

	/* available graph_id range is 0 ~ IPU_PSYS_NUM_STREAMS - 1 */
	u8 graph_id[IPU_PSYS_NUM_STREAMS];

	/* Device state, represent enum ipu7_msg_dev_state */
	enum ipu7_msg_dev_state dev_state;

	struct ipu7_psys_config *subsys_config;
	dma_addr_t subsys_config_dma_addr;
};

struct ipu7_psys_fh {
	struct ipu7_psys *psys;
	struct mutex mutex;	/* Protects bufmap & kcmds fields */
	struct list_head list;
	struct list_head bufmap;
	wait_queue_head_t wait;

	struct ipu7_psys_stream *ip;
};

struct ipu7_dma_buf_attach {
	struct device *dev;
	u64 len;
	void *userptr;
	struct sg_table *sgt;
	bool vma_is_io;
	struct page **pages;
	size_t npages;
};

struct ipu7_psys_kbuffer {
	u64 len;
	void *userptr;
	u32 flags;
	int fd;
	void *kaddr;
	struct list_head list;
	dma_addr_t dma_addr;
	struct sg_table *sgt;
	struct dma_buf_attachment *db_attach;
	struct dma_buf *dbuf;
	bool valid;	/* True when buffer is usable */
};

#define inode_to_ipu_psys(inode)				\
	container_of((inode)->i_cdev, struct ipu7_psys, cdev)

void ipu7_psys_setup_hw(struct ipu7_psys *psys);
void ipu7_psys_subdomains_power(struct ipu7_psys *psys, bool on);
void ipu7_psys_handle_events(struct ipu7_psys *psys);

long ipu7_ioctl_dqevent(struct ipu_psys_event *event,
			struct ipu7_psys_fh *fh, unsigned int f_flags);

#endif /* IPU7_PSYS_H */
