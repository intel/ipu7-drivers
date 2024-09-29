// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013 - 2024 Intel Corporation
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/completion.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>
#include <linux/types.h>

#include "ipu7.h"
#include "ipu7-bus.h"
#include "ipu7-buttress.h"
#include "ipu7-buttress-regs.h"

#define BOOTLOADER_STATUS_OFFSET	BUTTRESS_REG_FW_BOOT_PARAMS7

#define BOOTLOADER_MAGIC_KEY		0xb00710ad

#define ENTRY	BUTTRESS_IU2CSECSR_IPC_PEER_COMP_ACTIONS_RST_PHASE1
#define EXIT	BUTTRESS_IU2CSECSR_IPC_PEER_COMP_ACTIONS_RST_PHASE2
#define QUERY	BUTTRESS_IU2CSECSR_IPC_PEER_QUERIED_IP_COMP_ACTIONS_RST_PHASE

#define BUTTRESS_TSC_SYNC_RESET_TRIAL_MAX	10

#define BUTTRESS_POWER_TIMEOUT_US		(200 * USEC_PER_MSEC)

#define BUTTRESS_CSE_BOOTLOAD_TIMEOUT_US	(5 * USEC_PER_SEC)
#define BUTTRESS_CSE_AUTHENTICATE_TIMEOUT_US	(10 * USEC_PER_SEC)
#define BUTTRESS_CSE_FWRESET_TIMEOUT_US		(100 * USEC_PER_MSEC)

#define BUTTRESS_IPC_TX_TIMEOUT_MS		MSEC_PER_SEC
#define BUTTRESS_IPC_RX_TIMEOUT_MS		MSEC_PER_SEC
#define BUTTRESS_IPC_VALIDITY_TIMEOUT_US	(1 * USEC_PER_SEC)
#define BUTTRESS_TSC_SYNC_TIMEOUT_US		(5 * USEC_PER_MSEC)

#define BUTTRESS_IPC_RESET_RETRY		2000
#define BUTTRESS_CSE_IPC_RESET_RETRY		4
#define BUTTRESS_IPC_CMD_SEND_RETRY		1

static const u32 ipu7_adev_irq_mask[2] = {
	BUTTRESS_IRQ_IS_IRQ,
	BUTTRESS_IRQ_PS_IRQ
};

int ipu7_buttress_ipc_reset(struct ipu7_device *isp,
			    struct ipu7_buttress_ipc *ipc)
{
	struct device *dev = &isp->pdev->dev;

	unsigned int retries = BUTTRESS_IPC_RESET_RETRY;
	struct ipu7_buttress *b = &isp->buttress;
	u32 val = 0, csr_in_clr;

	if (!isp->secure_mode) {
		dev_dbg(dev, "Skip IPC reset for non-secure mode\n");
		return 0;
	}

	mutex_lock(&b->ipc_mutex);

	/* Clear-by-1 CSR (all bits), corresponding internal states. */
	val = readl(isp->base + ipc->csr_in);
	writel(val, isp->base + ipc->csr_in);

	/* Set peer CSR bit IPC_PEER_COMP_ACTIONS_RST_PHASE1 */
	writel(ENTRY, isp->base + ipc->csr_out);
	/*
	 * Clear-by-1 all CSR bits EXCEPT following
	 * bits:
	 * A. IPC_PEER_COMP_ACTIONS_RST_PHASE1.
	 * B. IPC_PEER_COMP_ACTIONS_RST_PHASE2.
	 * C. Possibly custom bits, depending on
	 * their role.
	 */
	csr_in_clr = BUTTRESS_IU2CSECSR_IPC_PEER_DEASSERTED_REG_VALID_REQ |
		BUTTRESS_IU2CSECSR_IPC_PEER_ACKED_REG_VALID |
		BUTTRESS_IU2CSECSR_IPC_PEER_ASSERTED_REG_VALID_REQ | QUERY;

	do {
		usleep_range(400, 500);
		val = readl(isp->base + ipc->csr_in);
		switch (val) {
		case ENTRY | EXIT:
		case ENTRY | EXIT | QUERY:
			/*
			 * 1) Clear-by-1 CSR bits
			 * (IPC_PEER_COMP_ACTIONS_RST_PHASE1,
			 * IPC_PEER_COMP_ACTIONS_RST_PHASE2).
			 * 2) Set peer CSR bit
			 * IPC_PEER_QUERIED_IP_COMP_ACTIONS_RST_PHASE.
			 */
			writel(ENTRY | EXIT, isp->base + ipc->csr_in);
			writel(QUERY, isp->base + ipc->csr_out);
			break;
		case ENTRY:
		case ENTRY | QUERY:
			/*
			 * 1) Clear-by-1 CSR bits
			 * (IPC_PEER_COMP_ACTIONS_RST_PHASE1,
			 * IPC_PEER_QUERIED_IP_COMP_ACTIONS_RST_PHASE).
			 * 2) Set peer CSR bit
			 * IPC_PEER_COMP_ACTIONS_RST_PHASE1.
			 */
			writel(ENTRY | QUERY, isp->base + ipc->csr_in);
			writel(ENTRY, isp->base + ipc->csr_out);
			break;
		case EXIT:
		case EXIT | QUERY:
			/*
			 * Clear-by-1 CSR bit
			 * IPC_PEER_COMP_ACTIONS_RST_PHASE2.
			 * 1) Clear incoming doorbell.
			 * 2) Clear-by-1 all CSR bits EXCEPT following
			 * bits:
			 * A. IPC_PEER_COMP_ACTIONS_RST_PHASE1.
			 * B. IPC_PEER_COMP_ACTIONS_RST_PHASE2.
			 * C. Possibly custom bits, depending on
			 * their role.
			 * 3) Set peer CSR bit
			 * IPC_PEER_COMP_ACTIONS_RST_PHASE2.
			 */
			writel(EXIT, isp->base + ipc->csr_in);
			writel(0, isp->base + ipc->db0_in);
			writel(csr_in_clr, isp->base + ipc->csr_in);
			writel(EXIT, isp->base + ipc->csr_out);

			/*
			 * Read csr_in again to make sure if RST_PHASE2 is done.
			 * If csr_in is QUERY, it should be handled again.
			 */
			usleep_range(200, 300);
			val = readl(isp->base + ipc->csr_in);
			if (val & QUERY) {
				dev_dbg(dev,
					"RST_PHASE2 retry csr_in = %x\n", val);
				break;
			}
			mutex_unlock(&b->ipc_mutex);
			return 0;
		case QUERY:
			/*
			 * 1) Clear-by-1 CSR bit
			 * IPC_PEER_QUERIED_IP_COMP_ACTIONS_RST_PHASE.
			 * 2) Set peer CSR bit
			 * IPC_PEER_COMP_ACTIONS_RST_PHASE1
			 */
			writel(QUERY, isp->base + ipc->csr_in);
			writel(ENTRY, isp->base + ipc->csr_out);
			break;
		default:
			dev_dbg_ratelimited(dev, "Unexpected CSR 0x%x\n", val);
			break;
		}
	} while (retries--);

	mutex_unlock(&b->ipc_mutex);
	dev_err(dev, "Timed out while waiting for CSE\n");

	return -ETIMEDOUT;
}

static void ipu7_buttress_ipc_validity_close(struct ipu7_device *isp,
					     struct ipu7_buttress_ipc *ipc)
{
	writel(BUTTRESS_IU2CSECSR_IPC_PEER_DEASSERTED_REG_VALID_REQ,
	       isp->base + ipc->csr_out);
}

static int
ipu7_buttress_ipc_validity_open(struct ipu7_device *isp,
				struct ipu7_buttress_ipc *ipc)
{
	unsigned int mask = BUTTRESS_IU2CSECSR_IPC_PEER_ACKED_REG_VALID;
	void __iomem *addr;
	int ret;
	u32 val;

	writel(BUTTRESS_IU2CSECSR_IPC_PEER_ASSERTED_REG_VALID_REQ,
	       isp->base + ipc->csr_out);

	addr = isp->base + ipc->csr_in;
	ret = readl_poll_timeout(addr, val, val & mask, 200,
				 BUTTRESS_IPC_VALIDITY_TIMEOUT_US);
	if (ret) {
		dev_err(&isp->pdev->dev, "CSE validity timeout 0x%x\n", val);
		ipu7_buttress_ipc_validity_close(isp, ipc);
	}

	return ret;
}

static void ipu7_buttress_ipc_recv(struct ipu7_device *isp,
				   struct ipu7_buttress_ipc *ipc, u32 *ipc_msg)
{
	if (ipc_msg)
		*ipc_msg = readl(isp->base + ipc->data0_in);
	writel(0, isp->base + ipc->db0_in);
}

static int ipu7_buttress_ipc_send_bulk(struct ipu7_device *isp,
				       struct ipu7_ipc_buttress_bulk_msg *msgs,
				       u32 size)
{
	unsigned long tx_timeout_jiffies, rx_timeout_jiffies;
	unsigned int i, retry = BUTTRESS_IPC_CMD_SEND_RETRY;
	struct ipu7_buttress *b = &isp->buttress;
	struct device *dev = &isp->pdev->dev;
	struct ipu7_buttress_ipc *ipc;
	u32 val;
	int ret;
	int tout;

	ipc = &b->cse;

	mutex_lock(&b->ipc_mutex);

	ret = ipu7_buttress_ipc_validity_open(isp, ipc);
	if (ret) {
		dev_err(dev, "IPC validity open failed\n");
		goto out;
	}

	tx_timeout_jiffies = msecs_to_jiffies(BUTTRESS_IPC_TX_TIMEOUT_MS);
	rx_timeout_jiffies = msecs_to_jiffies(BUTTRESS_IPC_RX_TIMEOUT_MS);

	for (i = 0; i < size; i++) {
		reinit_completion(&ipc->send_complete);
		if (msgs[i].require_resp)
			reinit_completion(&ipc->recv_complete);

		dev_dbg(dev, "bulk IPC command: 0x%x\n",
			msgs[i].cmd);
		writel(msgs[i].cmd, isp->base + ipc->data0_out);
		val = BUTTRESS_IU2CSEDB0_BUSY | msgs[i].cmd_size;
		writel(val, isp->base + ipc->db0_out);

		tout = wait_for_completion_timeout(&ipc->send_complete,
						   tx_timeout_jiffies);
		if (!tout) {
			dev_err(dev, "send IPC response timeout\n");
			if (!retry--) {
				ret = -ETIMEDOUT;
				goto out;
			}

			/* Try again if CSE is not responding on first try */
			writel(0, isp->base + ipc->db0_out);
			i--;
			continue;
		}

		retry = BUTTRESS_IPC_CMD_SEND_RETRY;

		if (!msgs[i].require_resp)
			continue;

		tout = wait_for_completion_timeout(&ipc->recv_complete,
						   rx_timeout_jiffies);
		if (!tout) {
			dev_err(dev, "recv IPC response timeout\n");
			ret = -ETIMEDOUT;
			goto out;
		}

		if (ipc->nack_mask &&
		    (ipc->recv_data & ipc->nack_mask) == ipc->nack) {
			dev_err(dev, "IPC NACK for cmd 0x%x\n", msgs[i].cmd);
			ret = -EIO;
			goto out;
		}

		if (ipc->recv_data != msgs[i].expected_resp) {
			dev_err(dev,
				"expected resp: 0x%x, IPC response: 0x%x\n",
				msgs[i].expected_resp, ipc->recv_data);
			ret = -EIO;
			goto out;
		}
	}

	dev_dbg(dev, "bulk IPC commands done\n");

out:
	ipu7_buttress_ipc_validity_close(isp, ipc);
	mutex_unlock(&b->ipc_mutex);
	return ret;
}

static int ipu7_buttress_ipc_send(struct ipu7_device *isp,
				  u32 ipc_msg, u32 size, bool require_resp,
				  u32 expected_resp)
{
	struct ipu7_ipc_buttress_bulk_msg msg = {
		.cmd = ipc_msg,
		.cmd_size = size,
		.require_resp = require_resp,
		.expected_resp = expected_resp,
	};

	return ipu7_buttress_ipc_send_bulk(isp, &msg, 1);
}

static irqreturn_t ipu7_buttress_call_isr(struct ipu7_bus_device *adev)
{
	irqreturn_t ret = IRQ_WAKE_THREAD;

	if (!adev || !adev->auxdrv || !adev->auxdrv_data)
		return IRQ_NONE;

	if (adev->auxdrv_data->isr)
		ret = adev->auxdrv_data->isr(adev);

	if (ret == IRQ_WAKE_THREAD && !adev->auxdrv_data->isr_threaded)
		ret = IRQ_NONE;

	return ret;
}

irqreturn_t ipu7_buttress_isr(int irq, void *isp_ptr)
{
	struct ipu7_device *isp = isp_ptr;
	struct ipu7_bus_device *adev[] = { isp->isys, isp->psys };
	struct ipu7_buttress *b = &isp->buttress;
	struct device *dev = &isp->pdev->dev;
	irqreturn_t ret = IRQ_NONE;
	u32 disable_irqs = 0;
	u32 irq_status;
	u32 pb_irq, pb_local_irq;
	unsigned int i;

	pm_runtime_get_noresume(dev);

	pb_irq = readl(isp->pb_base + INTERRUPT_STATUS);
	writel(pb_irq, isp->pb_base + INTERRUPT_STATUS);

	pb_local_irq = readl(isp->pb_base + BTRS_LOCAL_INTERRUPT_MASK);
	if (pb_local_irq) {
		dev_warn(dev, "PB interrupt status 0x%x local 0x%x\n", pb_irq,
			 pb_local_irq);
		dev_warn(dev, "Details: %x %x %x %x %x %x %x %x\n",
			 readl(isp->pb_base + ATS_ERROR_LOG1),
			 readl(isp->pb_base + ATS_ERROR_LOG2),
			 readl(isp->pb_base + CFI_0_ERROR_LOG),
			 readl(isp->pb_base + CFI_1_ERROR_LOGGING),
			 readl(isp->pb_base + IMR_ERROR_LOGGING_LOW),
			 readl(isp->pb_base + IMR_ERROR_LOGGING_HIGH),
			 readl(isp->pb_base + IMR_ERROR_LOGGING_CFI_1_LOW),
			 readl(isp->pb_base + IMR_ERROR_LOGGING_CFI_1_HIGH));
	}
	irq_status = readl(isp->base + BUTTRESS_REG_IRQ_STATUS);
	if (!irq_status) {
		pm_runtime_put_noidle(dev);
		return IRQ_NONE;
	}

	do {
		writel(irq_status, isp->base + BUTTRESS_REG_IRQ_CLEAR);

		for (i = 0; i < ARRAY_SIZE(ipu7_adev_irq_mask); i++) {
			irqreturn_t r = ipu7_buttress_call_isr(adev[i]);

			if (!(irq_status & ipu7_adev_irq_mask[i]))
				continue;

			if (r == IRQ_WAKE_THREAD) {
				ret = IRQ_WAKE_THREAD;
				disable_irqs |= ipu7_adev_irq_mask[i];
			} else if (ret == IRQ_NONE && r == IRQ_HANDLED) {
				ret = IRQ_HANDLED;
			}
		}

		if (irq_status & (BUTTRESS_IRQS | BUTTRESS_IRQ_SAI_VIOLATION) &&
		    ret == IRQ_NONE)
			ret = IRQ_HANDLED;

		if (irq_status & BUTTRESS_IRQ_IPC_FROM_CSE_IS_WAITING) {
			dev_dbg(dev, "BUTTRESS_IRQ_IPC_FROM_CSE_IS_WAITING\n");
			ipu7_buttress_ipc_recv(isp, &b->cse, &b->cse.recv_data);
			complete(&b->cse.recv_complete);
		}

		if (irq_status & BUTTRESS_IRQ_CSE_CSR_SET)
			dev_dbg(dev, "BUTTRESS_IRQ_CSE_CSR_SET\n");

		if (irq_status & BUTTRESS_IRQ_IPC_EXEC_DONE_BY_CSE) {
			dev_dbg(dev, "BUTTRESS_IRQ_IPC_EXEC_DONE_BY_CSE\n");
			complete(&b->cse.send_complete);
		}

		if (irq_status & BUTTRESS_IRQ_PUNIT_2_IUNIT_IRQ)
			dev_dbg(dev, "BUTTRESS_IRQ_PUNIT_2_IUNIT_IRQ\n");

		if (irq_status & BUTTRESS_IRQ_SAI_VIOLATION &&
		    ipu7_buttress_get_secure_mode(isp))
			dev_err(dev, "BUTTRESS_IRQ_SAI_VIOLATION\n");

		irq_status = readl(isp->base + BUTTRESS_REG_IRQ_STATUS);
	} while (irq_status);

	if (disable_irqs)
		writel(BUTTRESS_IRQS & ~disable_irqs,
		       isp->base + BUTTRESS_REG_IRQ_ENABLE);

	pm_runtime_put(dev);

	return ret;
}

irqreturn_t ipu7_buttress_isr_threaded(int irq, void *isp_ptr)
{
	struct ipu7_device *isp = isp_ptr;
	struct ipu7_bus_device *adev[] = { isp->isys, isp->psys };
	const struct ipu7_auxdrv_data *drv_data = NULL;
	irqreturn_t ret = IRQ_NONE;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ipu7_adev_irq_mask) && adev[i]; i++) {
		drv_data = adev[i]->auxdrv_data;
		if (!drv_data)
			continue;

		if (drv_data->wake_isr_thread &&
		    drv_data->isr_threaded(adev[i]) == IRQ_HANDLED)
			ret = IRQ_HANDLED;
	}

	writel(BUTTRESS_IRQS, isp->base + BUTTRESS_REG_IRQ_ENABLE);

	return ret;
}

static int isys_d2d_power(struct device *dev, bool on)
{
	struct ipu7_device *isp = to_ipu7_bus_device(dev)->isp;
	int ret = 0;
	u32 val;

	dev_dbg(dev, "power %s isys d2d.\n", on ? "UP" : "DOWN");
	val = readl(isp->base + BUTTRESS_REG_D2D_CTL);
	if (!(val & BUTTRESS_D2D_PWR_ACK) ^ on) {
		dev_info(dev, "d2d already in %s state.\n",
			 on ? "UP" : "DOWN");
		return 0;
	}

	val = on ? val | BUTTRESS_D2D_PWR_EN : val & (~BUTTRESS_D2D_PWR_EN);
	writel(val, isp->base + BUTTRESS_REG_D2D_CTL);
	ret = readl_poll_timeout(isp->base + BUTTRESS_REG_D2D_CTL,
				 val, (!(val & BUTTRESS_D2D_PWR_ACK) ^ on),
				 100, BUTTRESS_POWER_TIMEOUT_US);
	if (ret)
		dev_err(dev, "power %s d2d timeout. status: 0x%x\n",
			on ? "UP" : "DOWN", val);

	return ret;
}

static void isys_nde_control(struct device *dev, bool on)
{
	u32 val, value, scale, valid, resvec;
	struct ipu7_device *isp = to_ipu7_bus_device(dev)->isp;

	if (on) {
		value = BUTTRESS_NDE_VAL_ACTIVE;
		scale = BUTTRESS_NDE_SCALE_ACTIVE;
		valid = BUTTRESS_NDE_VALID_ACTIVE;
	} else {
		value = BUTTRESS_NDE_VAL_DEFAULT;
		scale = BUTTRESS_NDE_SCALE_DEFAULT;
		valid = BUTTRESS_NDE_VALID_DEFAULT;
	}

	resvec = BUTTRESS_NDE_RESVEC;
	val = FIELD_PREP(NDE_VAL_MASK, value) |
		FIELD_PREP(NDE_SCALE_MASK, scale) |
		FIELD_PREP(NDE_VALID_MASK, valid) |
		FIELD_PREP(NDE_RESVEC_MASK, resvec);

	dev_dbg(dev, "Set NED control value to 0x%x\n", val);
	writel(val, isp->base + BUTTRESS_REG_NDE_CONTROL);
}

int ipu7_buttress_powerup(struct device *dev, struct ipu7_buttress_ctrl *ctrl)
{
	struct ipu7_device *isp = to_ipu7_bus_device(dev)->isp;
	u32 val, exp_sts;
	int ret = 0;

	if (!ctrl)
		return 0;

	mutex_lock(&isp->buttress.power_mutex);

	exp_sts = ctrl->pwr_sts_on << ctrl->pwr_sts_shift;
	if (ctrl->subsys_id == IPU_IS) {
		ret = isys_d2d_power(dev, true);
		if (ret)
			goto out_power;
		isys_nde_control(dev, true);
	}

	/* request clock resource ownership */
	val = readl(isp->base + BUTTRESS_REG_SLEEP_LEVEL_CFG);
	val |= ctrl->ovrd_clk;
	writel(val, isp->base + BUTTRESS_REG_SLEEP_LEVEL_CFG);
	ret = readl_poll_timeout(isp->base + BUTTRESS_REG_SLEEP_LEVEL_STS,
				 val, (val & ctrl->own_clk_ack),
				 100, BUTTRESS_POWER_TIMEOUT_US);
	if (ret)
		dev_warn(dev, "request clk ownership timeout. status 0x%x\n",
			 val);

	val = ctrl->ratio << ctrl->ratio_shift | ctrl->cdyn << ctrl->cdyn_shift;

	dev_dbg(dev, "set 0x%x to %s_WORKPOINT_REQ.\n", val,
		ctrl->subsys_id == IPU_IS ? "IS" : "PS");
	writel(val, isp->base + ctrl->freq_ctl);

	ret = readl_poll_timeout(isp->base + BUTTRESS_REG_PWR_STATUS,
				 val, ((val & ctrl->pwr_sts_mask) == exp_sts),
				 100, BUTTRESS_POWER_TIMEOUT_US);
	if (ret) {
		dev_err(dev, "%s power up timeout with status: 0x%x\n",
			ctrl->subsys_id == IPU_IS ? "IS" : "PS", val);
		goto out_power;
	}

	dev_dbg(dev, "%s power up successfully. status: 0x%x\n",
		ctrl->subsys_id == IPU_IS ? "IS" : "PS", val);

	/* release clock resource ownership */
	val = readl(isp->base + BUTTRESS_REG_SLEEP_LEVEL_CFG);
	val &= ~ctrl->ovrd_clk;
	writel(val, isp->base + BUTTRESS_REG_SLEEP_LEVEL_CFG);

out_power:
	ctrl->started = !ret;
	mutex_unlock(&isp->buttress.power_mutex);

	return ret;
}

int ipu7_buttress_powerdown(struct device *dev,
			    struct ipu7_buttress_ctrl *ctrl)
{
	struct ipu7_device *isp = to_ipu7_bus_device(dev)->isp;
	u32 val, exp_sts;
	int ret = 0;

	if (!ctrl)
		return 0;

	mutex_lock(&isp->buttress.power_mutex);

	exp_sts = ctrl->pwr_sts_off << ctrl->pwr_sts_shift;
	val = 0x8 << ctrl->ratio_shift;

	dev_dbg(dev, "set 0x%x to %s_WORKPOINT_REQ.\n", val,
		ctrl->subsys_id == IPU_IS ? "IS" : "PS");
	writel(val, isp->base + ctrl->freq_ctl);
	ret = readl_poll_timeout(isp->base + BUTTRESS_REG_PWR_STATUS,
				 val, ((val & ctrl->pwr_sts_mask) == exp_sts),
				 100, BUTTRESS_POWER_TIMEOUT_US);
	if (ret) {
		dev_err(dev, "%s power down timeout with status: 0x%x\n",
			ctrl->subsys_id == IPU_IS ? "IS" : "PS", val);
		goto out_power;
	}

	dev_dbg(dev, "%s power down successfully. status: 0x%x\n",
		ctrl->subsys_id == IPU_IS ? "IS" : "PS", val);
out_power:
	if (ctrl->subsys_id == IPU_IS && !ret) {
		isys_d2d_power(dev, false);
		isys_nde_control(dev, false);
	}

	ctrl->started = false;
	mutex_unlock(&isp->buttress.power_mutex);

	return ret;
}

bool ipu7_buttress_get_secure_mode(struct ipu7_device *isp)
{
	u32 val;

	val = readl(isp->base + BUTTRESS_REG_SECURITY_CTL);

	return val & BUTTRESS_SECURITY_CTL_FW_SECURE_MODE;
}

bool ipu7_buttress_auth_done(struct ipu7_device *isp)
{
	u32 val;

	if (!isp->secure_mode)
		return true;

	val = readl(isp->base + BUTTRESS_REG_SECURITY_CTL);
	val = FIELD_GET(BUTTRESS_SECURITY_CTL_FW_SETUP_MASK, val);

	return val == BUTTRESS_SECURITY_CTL_AUTH_DONE;
}
EXPORT_SYMBOL_NS_GPL(ipu7_buttress_auth_done, INTEL_IPU7);

int ipu7_buttress_get_isys_freq(struct ipu7_device *isp, u32 *freq)
{
	u32 reg_val;
	int ret;

	ret = pm_runtime_get_sync(&isp->isys->auxdev.dev);
	if (ret < 0) {
		pm_runtime_put(&isp->isys->auxdev.dev);
		dev_err(&isp->pdev->dev, "Runtime PM failed (%d)\n", ret);
		return ret;
	}

	reg_val = readl(isp->base + BUTTRESS_REG_IS_WORKPOINT_REQ);

	pm_runtime_put(&isp->isys->auxdev.dev);

	*freq = (reg_val & BUTTRESS_IS_FREQ_CTL_RATIO_MASK) * 50 / 3;

	return 0;
}
EXPORT_SYMBOL_NS_GPL(ipu7_buttress_get_isys_freq, INTEL_IPU7);

int ipu7_buttress_get_psys_freq(struct ipu7_device *isp, u32 *freq)
{
	u32 reg_val;
	int ret;

	ret = pm_runtime_get_sync(&isp->psys->auxdev.dev);
	if (ret < 0) {
		pm_runtime_put(&isp->psys->auxdev.dev);
		dev_err(&isp->pdev->dev, "Runtime PM failed (%d)\n", ret);
		return ret;
	}

	reg_val = readl(isp->base + BUTTRESS_REG_PS_WORKPOINT_REQ);

	pm_runtime_put(&isp->psys->auxdev.dev);

	reg_val &= BUTTRESS_PS_FREQ_CTL_RATIO_MASK;
	*freq = BUTTRESS_PS_FREQ_RATIO_STEP * reg_val;

	return 0;
}
EXPORT_SYMBOL_NS_GPL(ipu7_buttress_get_psys_freq, INTEL_IPU7);

int ipu7_buttress_reset_authentication(struct ipu7_device *isp)
{
	struct device *dev = &isp->pdev->dev;
	int ret;
	u32 val;

	if (!isp->secure_mode) {
		dev_dbg(dev, "Skip auth for non-secure mode\n");
		return 0;
	}

	writel(BUTTRESS_FW_RESET_CTL_START, isp->base +
	       BUTTRESS_REG_FW_RESET_CTL);

	ret = readl_poll_timeout(isp->base + BUTTRESS_REG_FW_RESET_CTL, val,
				 val & BUTTRESS_FW_RESET_CTL_DONE, 500,
				 BUTTRESS_CSE_FWRESET_TIMEOUT_US);
	if (ret) {
		dev_err(dev, "Time out while resetting authentication state\n");
		return ret;
	}

	dev_dbg(dev, "FW reset for authentication done\n");
	writel(0, isp->base + BUTTRESS_REG_FW_RESET_CTL);
	/* leave some time for HW restore */
	usleep_range(800, 1000);

	return 0;
}

int ipu7_buttress_authenticate(struct ipu7_device *isp)
{
	struct ipu7_buttress *b = &isp->buttress;
	struct device *dev = &isp->pdev->dev;
	u32 data, mask, done, fail;
	int ret;

	if (!isp->secure_mode) {
		dev_dbg(dev, "Skip auth for non-secure mode\n");
		return 0;
	}

	mutex_lock(&b->auth_mutex);

	if (ipu7_buttress_auth_done(isp)) {
		ret = 0;
		goto out_unlock;
	}

	/*
	 * BUTTRESS_REG_FW_SOURCE_BASE needs to be set with FW CPD
	 * package address for secure mode.
	 */

	writel(isp->cpd_fw->size, isp->base + BUTTRESS_REG_FW_SOURCE_SIZE);
	writel(sg_dma_address(isp->psys->fw_sgt.sgl),
	       isp->base + BUTTRESS_REG_FW_SOURCE_BASE);

	/*
	 * Write boot_load into IU2CSEDATA0
	 * Write sizeof(boot_load) | 0x2 << CLIENT_ID to
	 * IU2CSEDB.IU2CSECMD and set IU2CSEDB.IU2CSEBUSY as
	 */
	dev_info(dev, "Sending BOOT_LOAD to CSE\n");
	ret = ipu7_buttress_ipc_send(isp, BUTTRESS_IU2CSEDATA0_IPC_BOOT_LOAD,
				     1, true,
				     BUTTRESS_CSE2IUDATA0_IPC_BOOT_LOAD_DONE);
	if (ret) {
		dev_err(dev, "CSE boot_load failed\n");
		goto out_unlock;
	}

	mask = BUTTRESS_SECURITY_CTL_FW_SETUP_MASK;
	done = BUTTRESS_SECURITY_CTL_FW_SETUP_DONE;
	fail = BUTTRESS_SECURITY_CTL_AUTH_FAILED;
	ret = readl_poll_timeout(isp->base + BUTTRESS_REG_SECURITY_CTL, data,
				 ((data & mask) == done ||
				  (data & mask) == fail), 500,
				 BUTTRESS_CSE_BOOTLOAD_TIMEOUT_US);
	if (ret) {
		dev_err(dev, "CSE boot_load timeout\n");
		goto out_unlock;
	}

	if ((data & mask) == fail) {
		dev_err(dev, "CSE auth failed\n");
		ret = -EINVAL;
		goto out_unlock;
	}

	ret = readl_poll_timeout(isp->base + BOOTLOADER_STATUS_OFFSET,
				 data, data == BOOTLOADER_MAGIC_KEY, 500,
				 BUTTRESS_CSE_BOOTLOAD_TIMEOUT_US);
	if (ret) {
		dev_err(dev, "Unexpected magic number 0x%x\n",
			data);
		goto out_unlock;
	}

	/*
	 * Write authenticate_run into IU2CSEDATA0
	 * Write sizeof(boot_load) | 0x2 << CLIENT_ID to
	 * IU2CSEDB.IU2CSECMD and set IU2CSEDB.IU2CSEBUSY as
	 */
	dev_info(dev, "Sending AUTHENTICATE_RUN to CSE\n");
	ret = ipu7_buttress_ipc_send(isp, BUTTRESS_IU2CSEDATA0_IPC_AUTH_RUN,
				     1, true,
				     BUTTRESS_CSE2IUDATA0_IPC_AUTH_RUN_DONE);
	if (ret) {
		dev_err(dev, "CSE authenticate_run failed\n");
		goto out_unlock;
	}

	done = BUTTRESS_SECURITY_CTL_AUTH_DONE;
	ret = readl_poll_timeout(isp->base + BUTTRESS_REG_SECURITY_CTL, data,
				 ((data & mask) == done ||
				  (data & mask) == fail), 500,
				 BUTTRESS_CSE_AUTHENTICATE_TIMEOUT_US);
	if (ret) {
		dev_err(dev, "CSE authenticate timeout\n");
		goto out_unlock;
	}

	if ((data & mask) == fail) {
		dev_err(dev, "CSE boot_load failed\n");
		ret = -EINVAL;
		goto out_unlock;
	}

	dev_info(dev, "CSE authenticate_run done\n");

out_unlock:
	mutex_unlock(&b->auth_mutex);

	return ret;
}

static int ipu7_buttress_send_tsc_request(struct ipu7_device *isp)
{
	u32 val, mask, done;
	int ret;

	mask = BUTTRESS_PWR_STATUS_HH_STATUS_MASK;

	writel(BUTTRESS_TSC_CMD_START_TSC_SYNC,
	       isp->base + BUTTRESS_REG_TSC_CMD);

	val = readl(isp->base + BUTTRESS_REG_PWR_STATUS);
	val = FIELD_GET(mask, val);
	if (val == BUTTRESS_PWR_STATUS_HH_STATE_ERR) {
		dev_err(&isp->pdev->dev, "Start tsc sync failed\n");
		return -EINVAL;
	}

	done = BUTTRESS_PWR_STATUS_HH_STATE_DONE;
	ret = readl_poll_timeout(isp->base + BUTTRESS_REG_PWR_STATUS, val,
				 FIELD_GET(mask, val) == done, 500,
				 BUTTRESS_TSC_SYNC_TIMEOUT_US);
	if (ret)
		dev_err(&isp->pdev->dev, "Start tsc sync timeout\n");

	return ret;
}

int ipu7_buttress_start_tsc_sync(struct ipu7_device *isp)
{
	void __iomem *base = isp->base;
	unsigned int i;
	u32 val;

	if (is_ipu7p5(isp->hw_ver)) {
		val = readl(base + BUTTRESS_REG_TSC_CTL);
		val |= BUTTRESS_SEL_PB_TIMESTAMP;
		writel(val, base + BUTTRESS_REG_TSC_CTL);

		for (i = 0; i < BUTTRESS_TSC_SYNC_RESET_TRIAL_MAX; i++) {
			val = readl(base + BUTTRESS_REG_PB_TIMESTAMP_VALID);
			if (val == 1)
				return 0;
			usleep_range(40, 50);
		}

		dev_err(&isp->pdev->dev, "PB HH sync failed (valid %u)\n", val);

		return -ETIMEDOUT;
	}

	for (i = 0; i < BUTTRESS_TSC_SYNC_RESET_TRIAL_MAX; i++) {
		int ret;

		ret = ipu7_buttress_send_tsc_request(isp);
		if (ret != -ETIMEDOUT)
			return ret;

		val = readl(base + BUTTRESS_REG_TSC_CTL);
		val = val | BUTTRESS_TSW_WA_SOFT_RESET;
		writel(val, base + BUTTRESS_REG_TSC_CTL);
		val = val & (~BUTTRESS_TSW_WA_SOFT_RESET);
		writel(val, base + BUTTRESS_REG_TSC_CTL);
	}

	dev_err(&isp->pdev->dev, "TSC sync failed (timeout)\n");

	return -ETIMEDOUT;
}
EXPORT_SYMBOL_NS_GPL(ipu7_buttress_start_tsc_sync, INTEL_IPU7);

void ipu7_buttress_tsc_read(struct ipu7_device *isp, u64 *val)
{
	u32 tsc_hi, tsc_lo;
	unsigned long flags;

	local_irq_save(flags);
	if (is_ipu7p5(isp->hw_ver)) {
		tsc_lo = readl(isp->base + BUTTRESS_REG_PB_TIMESTAMP_LO);
		tsc_hi = readl(isp->base + BUTTRESS_REG_PB_TIMESTAMP_HI);
	} else {
		tsc_lo = readl(isp->base + BUTTRESS_REG_TSC_LO);
		tsc_hi = readl(isp->base + BUTTRESS_REG_TSC_HI);
	}
	*val = (u64)tsc_hi << 32 | tsc_lo;
	local_irq_restore(flags);
}
EXPORT_SYMBOL_NS_GPL(ipu7_buttress_tsc_read, INTEL_IPU7);

#ifdef CONFIG_DEBUG_FS
static int ipu7_buttress_isys_freq_get(void *data, u64 *val)
{
	u32 freq;
	int ret;

	ret = ipu7_buttress_get_isys_freq(data, &freq);
	if (ret < 0)
		return ret;

	*val = freq;

	return 0;
}

static void ipu7_buttress_set_isys_ratio(struct ipu7_device *isp,
					 u32 isys_ratio)
{
	struct ipu7_buttress_ctrl *ctrl = isp->isys->ctrl;

	mutex_lock(&isp->buttress.power_mutex);

	if (ctrl->ratio == isys_ratio)
		goto out_mutex_unlock;

	ctrl->ratio = isys_ratio;
	if (!ctrl->started)
		return;

	writel(ctrl->ratio << ctrl->ratio_shift |
	       ctrl->cdyn << ctrl->cdyn_shift,
	       isp->base + ctrl->freq_ctl);

out_mutex_unlock:
	mutex_unlock(&isp->buttress.power_mutex);
}

static int ipu7_buttress_set_isys_freq(struct ipu7_device *isp, u64 freq)
{
	u32 ratio = freq * 50 / 3;
	int ret;

	if (freq < BUTTRESS_MIN_FORCE_IS_RATIO ||
	    freq > BUTTRESS_MAX_FORCE_IS_RATIO)
		return -EINVAL;

	ret = pm_runtime_get_sync(&isp->isys->auxdev.dev);
	if (ret < 0) {
		pm_runtime_put(&isp->isys->auxdev.dev);
		dev_err(&isp->pdev->dev, "Runtime PM failed (%d)\n", ret);
		return ret;
	}

	if (freq)
		ipu7_buttress_set_isys_ratio(isp, ratio);

	pm_runtime_put(&isp->isys->auxdev.dev);

	return 0;
}

static int ipu7_buttress_isys_freq_set(void *data, u64 val)
{
	return ipu7_buttress_set_isys_freq(data, val);
}

DEFINE_SIMPLE_ATTRIBUTE(ipu7_buttress_psys_freq_fops,
			ipu7_buttress_psys_freq_get, NULL, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(ipu7_buttress_isys_freq_fops,
			ipu7_buttress_isys_freq_get,
			ipu7_buttress_isys_freq_set, "%llu\n");

int ipu7_buttress_debugfs_init(struct ipu7_device *isp)
{
	struct dentry *dir, *file;

	dir = debugfs_create_dir("buttress", isp->ipu7_dir);
	if (!dir)
		return -ENOMEM;

	file = debugfs_create_file("psys_freq", 0400, dir, isp,
				   &ipu7_buttress_psys_freq_fops);
	if (!file)
		goto err;

	file = debugfs_create_file("isys_freq", 0700, dir, isp,
				   &ipu7_buttress_isys_freq_fops);
	if (!file)
		goto err;

	return 0;
err:
	debugfs_remove_recursive(dir);
	return -ENOMEM;
}

static ssize_t psys_fused_min_freq_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct ipu7_device *isp = pci_get_drvdata(to_pci_dev(dev));

	return snprintf(buf, PAGE_SIZE, "%u\n",
			isp->buttress.psys_fused_freqs.min_freq);
}

static DEVICE_ATTR_RO(psys_fused_min_freq);

static ssize_t psys_fused_max_freq_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct ipu7_device *isp = pci_get_drvdata(to_pci_dev(dev));

	return snprintf(buf, PAGE_SIZE, "%u\n",
			isp->buttress.psys_fused_freqs.max_freq);
}

static DEVICE_ATTR_RO(psys_fused_max_freq);

static ssize_t psys_fused_efficient_freq_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct ipu7_device *isp = pci_get_drvdata(to_pci_dev(dev));

	return snprintf(buf, PAGE_SIZE, "%u\n",
			isp->buttress.psys_fused_freqs.efficient_freq);
}

static DEVICE_ATTR_RO(psys_fused_efficient_freq);
#endif /* CONFIG_DEBUG_FS */

u64 ipu7_buttress_tsc_ticks_to_ns(u64 ticks, const struct ipu7_device *isp)
{
	u64 ns = ticks * 10000;

	/*
	 * converting TSC tick count to ns is calculated by:
	 * Example (TSC clock frequency is 19.2MHz):
	 * ns = ticks * 1000 000 000 / 19.2Mhz
	 *    = ticks * 1000 000 000 / 19200000Hz
	 *    = ticks * 10000 / 192 ns
	 */
	return div_u64(ns, isp->buttress.ref_clk);
}
EXPORT_SYMBOL_NS_GPL(ipu7_buttress_tsc_ticks_to_ns, INTEL_IPU7);

/* trigger uc control to wakeup fw */
void ipu7_buttress_wakeup_is_uc(const struct ipu7_device *isp)
{
	u32 val;

	val = readl(isp->base + BUTTRESS_REG_DRV_IS_UCX_CONTROL_STATUS);
	val |= UCX_CTL_WAKEUP;
	writel(val, isp->base + BUTTRESS_REG_DRV_IS_UCX_CONTROL_STATUS);
}
EXPORT_SYMBOL_NS_GPL(ipu7_buttress_wakeup_is_uc, INTEL_IPU7);

void ipu7_buttress_wakeup_ps_uc(const struct ipu7_device *isp)
{
	u32 val;

	val = readl(isp->base + BUTTRESS_REG_DRV_PS_UCX_CONTROL_STATUS);
	val |= UCX_CTL_WAKEUP;
	writel(val, isp->base + BUTTRESS_REG_DRV_PS_UCX_CONTROL_STATUS);
}
EXPORT_SYMBOL_NS_GPL(ipu7_buttress_wakeup_ps_uc, INTEL_IPU7);

static void ipu7_buttress_setup(struct ipu7_device *isp)
{
	struct device *dev = &isp->pdev->dev;

	/* program PB BAR */
	writel(0, isp->pb_base + GLOBAL_INTERRUPT_MASK);
	writel(0x100, isp->pb_base + BAR2_MISC_CONFIG);

	if (is_ipu7p5(isp->hw_ver)) {
		writel(BIT(14), isp->pb_base + TLBID_HASH_ENABLE_63_32);
		writel(BIT(9), isp->pb_base + TLBID_HASH_ENABLE_95_64);
		dev_dbg(dev, "PTL TLBID_HASH %x %x\n",
			readl(isp->pb_base + TLBID_HASH_ENABLE_63_32),
			readl(isp->pb_base + TLBID_HASH_ENABLE_95_64));
	} else {
		writel(BIT(22), isp->pb_base + TLBID_HASH_ENABLE_63_32);
		writel(BIT(1), isp->pb_base + TLBID_HASH_ENABLE_127_96);
		dev_dbg(dev, "TLBID_HASH %x %x\n",
			readl(isp->pb_base + TLBID_HASH_ENABLE_63_32),
			readl(isp->pb_base + TLBID_HASH_ENABLE_127_96));
	}

	writel(BUTTRESS_IRQS, isp->base + BUTTRESS_REG_IRQ_CLEAR);
	writel(BUTTRESS_IRQS, isp->base + BUTTRESS_REG_IRQ_MASK);
	writel(BUTTRESS_IRQS, isp->base + BUTTRESS_REG_IRQ_ENABLE);
	/* LNL SW workaround for PS PD hang when PS sub-domain during PD */
	writel(PS_FSM_CG, isp->base + BUTTRESS_REG_CG_CTRL_BITS);
}

void ipu7_buttress_restore(struct ipu7_device *isp)
{
	struct ipu7_buttress *b = &isp->buttress;

	ipu7_buttress_setup(isp);

	writel(b->wdt_cached_value, isp->base + BUTTRESS_REG_IDLE_WDT);
}

int ipu7_buttress_init(struct ipu7_device *isp)
{
	int ret, ipc_reset_retry = BUTTRESS_CSE_IPC_RESET_RETRY;
	struct ipu7_buttress *b = &isp->buttress;
	struct device *dev = &isp->pdev->dev;
	u32 val;

	mutex_init(&b->power_mutex);
	mutex_init(&b->auth_mutex);
	mutex_init(&b->cons_mutex);
	mutex_init(&b->ipc_mutex);
	init_completion(&b->cse.send_complete);
	init_completion(&b->cse.recv_complete);

	b->cse.nack = BUTTRESS_CSE2IUDATA0_IPC_NACK;
	b->cse.nack_mask = BUTTRESS_CSE2IUDATA0_IPC_NACK_MASK;
	b->cse.csr_in = BUTTRESS_REG_CSE2IUCSR;
	b->cse.csr_out = BUTTRESS_REG_IU2CSECSR;
	b->cse.db0_in = BUTTRESS_REG_CSE2IUDB0;
	b->cse.db0_out = BUTTRESS_REG_IU2CSEDB0;
	b->cse.data0_in = BUTTRESS_REG_CSE2IUDATA0;
	b->cse.data0_out = BUTTRESS_REG_IU2CSEDATA0;

	INIT_LIST_HEAD(&b->constraints);

	isp->secure_mode = ipu7_buttress_get_secure_mode(isp);
	val = readl(isp->base + BUTTRESS_REG_IPU_SKU);
	dev_info(dev, "IPU%u SKU %u in %s mode mask 0x%x\n", val & 0xf,
		 (val >> 4) & 0x7, isp->secure_mode ? "secure" : "non-secure",
		 readl(isp->base + BUTTRESS_REG_CAMERA_MASK));
	b->wdt_cached_value = readl(isp->base + BUTTRESS_REG_IDLE_WDT);
	b->ref_clk = 384;

	ipu7_buttress_setup(isp);
#ifdef CONFIG_DEBUG_FS

	ret = device_create_file(dev, &dev_attr_psys_fused_min_freq);
	if (ret) {
		dev_err(dev, "Create min freq file failed\n");
		goto err_mutex_destroy;
	}

	ret = device_create_file(dev, &dev_attr_psys_fused_max_freq);
	if (ret) {
		dev_err(dev, "Create max freq file failed\n");
		goto err_remove_min_freq_file;
	}

	ret = device_create_file(dev, &dev_attr_psys_fused_efficient_freq);
	if (ret) {
		dev_err(dev, "Create efficient freq file failed\n");
		goto err_remove_max_freq_file;
	}
#endif

	/* Retry couple of times in case of CSE initialization is delayed */
	do {
		ret = ipu7_buttress_ipc_reset(isp, &b->cse);
		if (ret) {
			dev_warn(dev, "IPC reset protocol failed, retrying\n");
		} else {
			dev_dbg(dev, "IPC reset done\n");
			return 0;
		}
	} while (ipc_reset_retry--);

	dev_err(dev, "IPC reset protocol failed\n");

#ifdef CONFIG_DEBUG_FS
err_remove_max_freq_file:
	device_remove_file(dev, &dev_attr_psys_fused_max_freq);
err_remove_min_freq_file:
	device_remove_file(dev, &dev_attr_psys_fused_min_freq);
err_mutex_destroy:
#endif
	mutex_destroy(&b->power_mutex);
	mutex_destroy(&b->auth_mutex);
	mutex_destroy(&b->cons_mutex);
	mutex_destroy(&b->ipc_mutex);

	return ret;
}

void ipu7_buttress_exit(struct ipu7_device *isp)
{
	struct ipu7_buttress *b = &isp->buttress;

	writel(0, isp->base + BUTTRESS_REG_IRQ_ENABLE);

#ifdef CONFIG_DEBUG_FS
	device_remove_file(&isp->pdev->dev,
			   &dev_attr_psys_fused_efficient_freq);
	device_remove_file(&isp->pdev->dev, &dev_attr_psys_fused_max_freq);
	device_remove_file(&isp->pdev->dev, &dev_attr_psys_fused_min_freq);
#endif
	mutex_destroy(&b->power_mutex);
	mutex_destroy(&b->auth_mutex);
	mutex_destroy(&b->cons_mutex);
	mutex_destroy(&b->ipc_mutex);
}
