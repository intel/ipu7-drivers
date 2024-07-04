/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2013 - 2024 Intel Corporation
 */

#ifndef IPU7_BUTTRESS_H
#define IPU7_BUTTRESS_H

#include <linux/completion.h>
#include <linux/irqreturn.h>
#include <linux/list.h>
#include <linux/mutex.h>

struct device;
struct ipu7_device;

struct ipu7_buttress_ctrl {
	u32 subsys_id;
	u32 freq_ctl, pwr_sts_shift, pwr_sts_mask, pwr_sts_on, pwr_sts_off;
	u32 ratio;
	u32 ratio_shift;
	u32 cdyn;
	u32 cdyn_shift;
	bool started;
	u32 ovrd_clk;
	u32 own_clk_ack;
};

struct ipu7_buttress_fused_freqs {
	u32 min_freq;
	u32 max_freq;
	u32 efficient_freq;
};

struct ipu7_buttress_ipc {
	struct completion send_complete;
	struct completion recv_complete;
	u32 nack;
	u32 nack_mask;
	u32 recv_data;
	u32 csr_out;
	u32 csr_in;
	u32 db0_in;
	u32 db0_out;
	u32 data0_out;
	u32 data0_in;
};

struct ipu7_buttress {
	struct mutex power_mutex, auth_mutex, cons_mutex, ipc_mutex;
	struct ipu7_buttress_ipc cse;
	struct ipu7_buttress_ipc ish;
	struct list_head constraints;
	struct ipu7_buttress_fused_freqs psys_fused_freqs;
	u32 psys_min_freq;
	u32 wdt_cached_value;
	u8 psys_force_ratio;
	bool force_suspend;
	u32 ref_clk;
};

enum ipu7_buttress_ipc_domain {
	IPU_BUTTRESS_IPC_CSE,
	IPU_BUTTRESS_IPC_ISH,
};

struct ipu7_buttress_constraint {
	struct list_head list;
	u32 min_freq;
};

struct ipu7_ipc_buttress_bulk_msg {
	u32 cmd;
	u32 expected_resp;
	bool require_resp;
	u8 cmd_size;
};

int ipu7_buttress_ipc_reset(struct ipu7_device *isp,
			    struct ipu7_buttress_ipc *ipc);
int ipu7_buttress_power(struct device *dev,
			struct ipu7_buttress_ctrl *ctrl, bool on);
bool ipu7_buttress_get_secure_mode(struct ipu7_device *isp);
int ipu7_buttress_authenticate(struct ipu7_device *isp);
int ipu7_buttress_reset_authentication(struct ipu7_device *isp);
bool ipu7_buttress_auth_done(struct ipu7_device *isp);
int ipu7_buttress_get_isys_freq(struct ipu7_device *isp, u32 *freq);
int ipu7_buttress_get_psys_freq(struct ipu7_device *isp, u32 *freq);
int ipu7_buttress_start_tsc_sync(struct ipu7_device *isp);
void ipu7_buttress_tsc_read(struct ipu7_device *isp, u64 *val);
u64 ipu7_buttress_tsc_ticks_to_ns(u64 ticks, const struct ipu7_device *isp);

irqreturn_t ipu7_buttress_isr(int irq, void *isp_ptr);
irqreturn_t ipu7_buttress_isr_threaded(int irq, void *isp_ptr);
int ipu7_buttress_debugfs_init(struct ipu7_device *isp);
int ipu7_buttress_init(struct ipu7_device *isp);
void ipu7_buttress_exit(struct ipu7_device *isp);
void ipu7_buttress_csi_port_config(struct ipu7_device *isp,
				   u32 legacy, u32 combo);
void ipu7_buttress_restore(struct ipu7_device *isp);
int ipu7_buttress_psys_freq_get(void *data, u64 *val);
void ipu7_buttress_wakeup_is_uc(const struct ipu7_device *isp);
void ipu7_buttress_wakeup_ps_uc(const struct ipu7_device *isp);
u32 ipu7_buttress_get_ref_clk(const struct ipu7_device *isp);
#endif /* IPU7_BUTTRESS_H */
