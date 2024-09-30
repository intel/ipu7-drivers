// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013 - 2024 Intel Corporation
 */

#include <linux/bitmap.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/types.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-device.h>

#include "ipu7.h"
#include "ipu7-bus.h"
#include "ipu7-buttress.h"
#include "ipu7-isys.h"
#include "ipu7-isys-csi2.h"
#include "ipu7-isys-csi2-regs.h"
#include "ipu7-platform-regs.h"
#include "ipu7-isys-csi-phy.h"

#define PORT_A	0
#define PORT_B	1
#define PORT_C	2
#define PORT_D	3

struct ddlcal_counter_ref_s {
	u16 min_mbps;
	u16 max_mbps;

	u16 ddlcal_counter_ref;
};

struct ddlcal_params {
	u16 min_mbps;
	u16 max_mbps;
	u16 oa_lanex_hsrx_cdphy_sel_fast;
	u16 ddlcal_max_phase;
	u16 phase_bound;
	u16 ddlcal_dll_fbk;
	u16 ddlcal_ddl_coarse_bank;
	u16 fjump_deskew;
	u16 min_eye_opening_deskew;
};

struct i_thssettle_params {
	u16 min_mbps;
	u16 max_mbps;
	u16 i_thssettle;
};

struct i_coarse_target_params {
	u16 min_mbps;
	u16 max_mbps;
	u16 i_coarse_target;
};

struct post_thres_params {
	u16 min_mbps;
	u16 max_mbps;

	u16 post_received_reset_thresh;
	u16 post_det_delay_thresh;
};

 /* lane2 for 4l3t, lane1 for 2l2t */
struct oa_lane_clk_div_params {
	u16 min_mbps;
	u16 max_mbps;
	u16 oa_lane_hsrx_hs_clk_div;
};

struct des_delay_params {
	u16 min_mbps;
	u16 max_mbps;
	u16 deserializer_en_delay_deass_thresh;
};

static const struct ddlcal_counter_ref_s table0[] = {
	{1500, 1999, 118},
	{2000, 2499, 157},
	{2500, 3499, 196},
	{3500, 4499, 274},
	{4500, 4500, 352},
	{}
};

static const struct ddlcal_params table1[] = {
	{1500, 1587, 0, 143, 167, 17, 3, 4, 29},
	{1588, 1687, 0, 135, 167, 15, 3, 4, 27},
	{1688, 1799, 0, 127, 135, 15, 2, 4, 26},
	{1800, 1928, 0, 119, 135, 13, 2, 3, 24},
	{1929, 2076, 0, 111, 135, 13, 2, 3, 23},
	{2077, 2249, 0, 103, 135, 11, 2, 3, 21},
	{2250, 2454, 0, 95, 103, 11, 1, 3, 19},
	{2455, 2699, 0, 87, 103, 9, 1, 3, 18},
	{2700, 2999, 0, 79, 103, 9, 1, 2, 16},
	{3000, 3229, 0, 71, 71, 7, 1, 2, 15},
	{3230, 3599, 1, 87, 103, 9, 1, 3, 18},
	{3600, 3999, 1, 79, 103, 9, 1, 2, 16},
	{4000, 4499, 1, 71, 103, 7, 1, 2, 15},
	{4500, 4500, 1, 63, 71, 7, 0, 2, 13},
	{}
};

static const struct i_thssettle_params table2[] = {
	{80, 124, 24},
	{125, 249, 20},
	{250, 499, 16},
	{500, 749, 14},
	{750, 1499, 13},
	{1500, 4500, 12},
	{}
};

static const struct oa_lane_clk_div_params table6[] = {
	{80, 159, 0x1},
	{160, 319, 0x2},
	{320, 639, 0x3},
	{640, 1279, 0x4},
	{1280, 2560, 0x5},
	{2561, 4500, 0x6},
	{}
};

enum ppi_datawidth {
	PPI_DATAWIDTH_8BIT = 0,
	PPI_DATAWIDTH_16BIT = 1,
};

static void dwc_phy_write(struct ipu7_isys *isys, u32 id, u32 addr, u16 data)
{
	void __iomem *isys_base = isys->pdata->base;
	void __iomem *base = isys_base + IS_IO_CDPHY_BASE(id);

	dev_dbg(&isys->adev->auxdev.dev, "phy write: reg 0x%lx = data 0x%04x",
		base + addr - isys_base, data);
	writew(data, base + addr);
}

static u16 dwc_phy_read(struct ipu7_isys *isys, u32 id, u32 addr)
{
	u16 data;
	void __iomem *isys_base = isys->pdata->base;
	void __iomem *base = isys_base + IS_IO_CDPHY_BASE(id);

	data = readw(base + addr);
	dev_dbg(&isys->adev->auxdev.dev, "phy read: reg 0x%lx = data 0x%04x",
		base + addr - isys_base, data);

	return data;
}

static void dwc_csi_write(struct ipu7_isys *isys, u32 id, u32 addr,
			  u32 data)
{
	struct device *dev = &isys->adev->auxdev.dev;
	void __iomem *isys_base = isys->pdata->base;
	void __iomem *base = isys_base + IS_IO_CSI2_HOST_BASE(id);

	dev_dbg(dev, "csi write: reg 0x%lx = data 0x%08x",
		base + addr - isys_base, data);
	writel(data, base + addr);
	dev_dbg(dev, "csi read: reg 0x%lx = data 0x%08x",
		base + addr - isys_base, readl(base + addr));
}

static void gpreg_write(struct ipu7_isys *isys, u32 id, u32 addr,
			u32 data)
{
	struct device *dev = &isys->adev->auxdev.dev;
	void __iomem *isys_base = isys->pdata->base;
	void __iomem *base = isys_base + IS_IO_CSI2_GPREGS_BASE(id);

	dev_dbg(dev, "gpreg write: reg 0x%lx = data 0x%08x",
		base + addr - isys_base, data);
	writel(data, base + addr);
	dev_dbg(dev, "gpreg read: reg 0x%lx = data 0x%08x",
		base + addr - isys_base, readl(base + addr));
}

static u32 dwc_csi_read(struct ipu7_isys *isys, u32 id, u32 addr)
{
	u32 data;
	void __iomem *isys_base = isys->pdata->base;
	void __iomem *base = isys_base + IS_IO_CSI2_HOST_BASE(id);

	data = readl(base + addr);
	dev_dbg(&isys->adev->auxdev.dev, "csi read: reg 0x%lx = data 0x%x",
		base + addr - isys_base, data);

	return data;
}

static void dwc_phy_write_mask(struct ipu7_isys *isys, u32 id, u32 addr,
			       u16 val, u8 lo, u8 hi)
{
	u32 temp, mask;

	WARN_ON(lo > hi);
	WARN_ON(hi > 15);

	mask = ((~0U - (1 << lo) + 1)) & (~0U >> (31 - hi));
	temp = dwc_phy_read(isys, id, addr);
	temp &= ~mask;
	temp |= (val << lo) & mask;
	dwc_phy_write(isys, id, addr, temp);
}

static void dwc_csi_write_mask(struct ipu7_isys *isys, u32 id, u32 addr,
			       u32 val, u8 hi, u8 lo)
{
	u32 temp, mask;

	WARN_ON(lo > hi);

	mask = ((~0U - (1 << lo) + 1)) & (~0U >> (31 - hi));
	temp = dwc_csi_read(isys, id, addr);
	temp &= ~mask;
	temp |= (val << lo) & mask;
	dwc_csi_write(isys, id, addr, temp);
}

static void ipu7_isys_csi_ctrl_cfg(struct ipu7_isys_csi2 *csi2)
{
	struct ipu7_isys *isys = csi2->isys;
	struct device *dev = &isys->adev->auxdev.dev;
	u32 id, lanes;
	u32 val;

	id = csi2->port;
	lanes = csi2->nlanes;
	dev_dbg(dev, "csi-%d controller init with %u lanes", id, lanes);

	val = dwc_csi_read(isys, id, VERSION);
	dev_dbg(dev, "csi-%d controller version = 0x%x", id, val);

	/* num of active data lanes */
	dwc_csi_write(isys, id, N_LANES, lanes - 1);
	dwc_csi_write(isys, id, CDPHY_MODE, PHY_MODE_DPHY);
	dwc_csi_write(isys, id, VC_EXTENSION, 0);

	/* only mask PHY_FATAL and PKT_FATAL interrupts */
	dwc_csi_write(isys, id, INT_MSK_PHY_FATAL, 0xff);
	dwc_csi_write(isys, id, INT_MSK_PKT_FATAL, 0x3);
	dwc_csi_write(isys, id, INT_MSK_PHY, 0x0);
	dwc_csi_write(isys, id, INT_MSK_LINE, 0x0);
	dwc_csi_write(isys, id, INT_MSK_BNDRY_FRAME_FATAL, 0x0);
	dwc_csi_write(isys, id, INT_MSK_SEQ_FRAME_FATAL, 0x0);
	dwc_csi_write(isys, id, INT_MSK_CRC_FRAME_FATAL, 0x0);
	dwc_csi_write(isys, id, INT_MSK_PLD_CRC_FATAL, 0x0);
	dwc_csi_write(isys, id, INT_MSK_DATA_ID, 0x0);
	dwc_csi_write(isys, id, INT_MSK_ECC_CORRECTED, 0x0);
}

static void ipu7_isys_csi_phy_reset(struct ipu7_isys *isys, u32 id)
{
	dwc_csi_write(isys, id, PHY_SHUTDOWNZ, 0);
	dwc_csi_write(isys, id, DPHY_RSTZ, 0);
	dwc_csi_write(isys, id, CSI2_RESETN, 0);
	gpreg_write(isys, id, PHY_RESET, 0);
	gpreg_write(isys, id, PHY_SHUTDOWN, 0);
}

#define N_DATA_IDS		8
static DECLARE_BITMAP(data_ids, N_DATA_IDS);
/* 8 Data ID monitors, each Data ID is composed by pair of VC and data type */
int ipu7_isys_csi_ctrl_dids_config(struct ipu7_isys *isys, u32 id, u8 vc, u8 dt)
{
	u32 reg, n;
	int ret;
	u8 lo, hi;

	dev_dbg(&isys->adev->auxdev.dev, "Config CSI-%u with vc:%u data-type:0x%x\n",
		id, vc, dt);
	/* enable VCX: 2-bit field for DPHY, 3-bit for CPHY */
	dwc_csi_write(isys, id, VC_EXTENSION, 0x0);
	n = find_first_zero_bit(data_ids, N_DATA_IDS);
	if (n == N_DATA_IDS)
		return -ENOSPC;

	ret = test_and_set_bit(n, data_ids);
	if (ret)
		return -EBUSY;

	reg = n < 4 ? DATA_IDS_VC_1 : DATA_IDS_VC_2;
	lo = (n % 4) * 8;
	hi = lo + 4;
	dwc_csi_write_mask(isys, id, reg, vc & GENMASK(4, 0), hi, lo);

	reg = n < 4 ? DATA_IDS_1 : DATA_IDS_2;
	lo = (n % 4) * 8;
	hi = lo + 5;
	dwc_csi_write_mask(isys, id, reg, dt & GENMASK(5, 0), hi, lo);

	return 0;
}

#define CDPHY_TIMEOUT (5000000)
static int ipu7_isys_phy_ready(struct ipu7_isys *isys, u32 id)
{
	void __iomem *base = isys->pdata->base;
	void __iomem *gpreg = base + IS_IO_CSI2_GPREGS_BASE(id);
	struct device *dev = &isys->adev->auxdev.dev;
	unsigned int i;
	u32 phy_ready;
	u32 reg, rext;
	int ret;

	dev_dbg(dev, "waiting phy ready...\n");
	ret = readl_poll_timeout(gpreg + PHY_READY, phy_ready,
				 phy_ready & BIT(0) && phy_ready != ~0U,
				 100, CDPHY_TIMEOUT);
	dev_dbg(dev, "phy %u ready = 0x%08x\n", id, readl(gpreg + PHY_READY));
	dev_dbg(dev, "csi %u PHY_RX = 0x%08x\n", id,
		dwc_csi_read(isys, id, PHY_RX));
	dev_dbg(dev, "csi %u PHY_STOPSTATE = 0x%08x\n", id,
		dwc_csi_read(isys, id, PHY_STOPSTATE));
	dev_dbg(dev, "csi %u PHY_CAL = 0x%08x\n", id,
		dwc_csi_read(isys, id, PHY_CAL));
	for (i = 0; i < 4; i++) {
		reg = CORE_DIG_DLANE_0_R_HS_RX_0 + (i * 0x400);
		dev_dbg(dev, "phy %u DLANE%u skewcal = 0x%04x\n",
			id, i, dwc_phy_read(isys, id, reg));
	}
	dev_dbg(dev, "phy %u DDLCAL = 0x%04x\n", id,
		dwc_phy_read(isys, id, PPI_CALIBCTRL_R_COMMON_CALIBCTRL_2_5));
	dev_dbg(dev, "phy %u TERMCAL = 0x%04x\n", id,
		dwc_phy_read(isys, id, PPI_R_TERMCAL_DEBUG_0));
	dev_dbg(dev, "phy %u LPDCOCAL = 0x%04x\n", id,
		dwc_phy_read(isys, id, PPI_R_LPDCOCAL_DEBUG_RB));
	dev_dbg(dev, "phy %u HSDCOCAL = 0x%04x\n", id,
		dwc_phy_read(isys, id, PPI_R_HSDCOCAL_DEBUG_RB));
	dev_dbg(dev, "phy %u LPDCOCAL_VT = 0x%04x\n", id,
		dwc_phy_read(isys, id, PPI_R_LPDCOCAL_DEBUG_VT));

	if (!ret) {
		if (id) {
			dev_dbg(dev, "ignore phy %u rext\n", id);
			return 0;
		}

		rext = dwc_phy_read(isys, id,
				    CORE_DIG_IOCTRL_R_AFE_CB_CTRL_2_15) & 0xf;
		dev_dbg(dev, "phy %u rext value = %u\n", id, rext);
		isys->phy_rext_cal = rext;

		if (!isys->phy_rext_cal)
			isys->phy_rext_cal = 5;

		return 0;
	}

	dev_err(dev, "wait phy ready timeout!\n");

	return ret;
}

static int lookup_table0(u16 mbps)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(table0); i++) {
		if (mbps >= table0[i].min_mbps && mbps <= table0[i].max_mbps)
			return i;
	}

	return -ENXIO;
}

static int lookup_table1(u16 mbps)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(table1); i++) {
		if (mbps >= table1[i].min_mbps && mbps <= table1[i].max_mbps)

			return i;
	}

	return -ENXIO;
}

static int lookup_table2(u16 mbps)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(table2); i++) {
		if (mbps >= table2[i].min_mbps && mbps <= table2[i].max_mbps)
			return i;
	}

	return -ENXIO;
}

static int lookup_table6(u16 mbps)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(table6); i++) {
		if (mbps >= table6[i].min_mbps && mbps <= table6[i].max_mbps)
			return i;
	}

	return -ENXIO;
}

static const u16 deskew_fine_mem[] = {
	0x0404, 0x040c, 0x0414, 0x041c,
	0x0423, 0x0429, 0x0430, 0x043a,
	0x0445, 0x044a, 0x0450, 0x045a,
	0x0465, 0x0469, 0x0472, 0x047a,
	0x0485, 0x0489, 0x0490, 0x049a,
	0x04a4, 0x04ac, 0x04b4, 0x04bc,
	0x04c4, 0x04cc, 0x04d4, 0x04dc,
	0x04e4, 0x04ec, 0x04f4, 0x04fc,
	0x0504, 0x050c, 0x0514, 0x051c,
	0x0523, 0x0529, 0x0530, 0x053a,
	0x0545, 0x054a, 0x0550, 0x055a,
	0x0565, 0x0569, 0x0572, 0x057a,
	0x0585, 0x0589, 0x0590, 0x059a,
	0x05a4, 0x05ac, 0x05b4, 0x05bc,
	0x05c4, 0x05cc, 0x05d4, 0x05dc,
	0x05e4, 0x05ec, 0x05f4, 0x05fc,
	0x0604, 0x060c, 0x0614, 0x061c,
	0x0623, 0x0629, 0x0632, 0x063a,
	0x0645, 0x064a, 0x0650, 0x065a,
	0x0665, 0x0669, 0x0672, 0x067a,
	0x0685, 0x0689, 0x0690, 0x069a,
	0x06a4, 0x06ac, 0x06b4, 0x06bc,
	0x06c4, 0x06cc, 0x06d4, 0x06dc,
	0x06e4, 0x06ec, 0x06f4, 0x06fc,
	0x0704, 0x070c, 0x0714, 0x071c,
	0x0723, 0x072a, 0x0730, 0x073a,
	0x0745, 0x074a, 0x0750, 0x075a,
	0x0765, 0x0769, 0x0772, 0x077a,
	0x0785, 0x0789, 0x0790, 0x079a,
	0x07a4, 0x07ac, 0x07b4, 0x07bc,
	0x07c4, 0x07cc, 0x07d4, 0x07dc,
	0x07e4, 0x07ec, 0x07f4, 0x07fc,
};

static void ipu7_isys_phy_config(struct ipu7_isys *isys, u8 id, u8 lanes,
				 bool aggregation)
{
	struct device *dev = &isys->adev->auxdev.dev;
	u16 hsrxval0, hsrxval1, hsrxval2;
	s64 link_freq;
	int index;
	s64 mbps;
	u16 reg;
	u16 val;
	u32 i;

	if (aggregation)
		link_freq = ipu7_isys_csi2_get_link_freq(&isys->csi2[0]);
	else
		link_freq = ipu7_isys_csi2_get_link_freq(&isys->csi2[id]);

	if (link_freq < 0) {
		dev_warn(dev, "get link freq failed, use default mbps\n");
		link_freq = 560000000;
	}

	mbps = div_u64(link_freq, 500000);
	dev_dbg(dev, "config phy %u with lanes %u aggregation %d mbps %lld\n",
		id, lanes, aggregation, mbps);

	dwc_phy_write_mask(isys, id, PPI_STARTUP_RW_COMMON_DPHY_10, 48, 0, 7);
	dwc_phy_write_mask(isys, id, CORE_DIG_ANACTRL_RW_COMMON_ANACTRL_2,
			   1, 12, 13);
	dwc_phy_write_mask(isys, id, CORE_DIG_ANACTRL_RW_COMMON_ANACTRL_0,
			   63, 2, 7);
	dwc_phy_write_mask(isys, id, PPI_STARTUP_RW_COMMON_STARTUP_1_1,
			   563, 0, 11);
	dwc_phy_write_mask(isys, id, PPI_STARTUP_RW_COMMON_DPHY_2, 5, 0, 7);
	/* bypass the RCAL state (bit6) */
	if (aggregation && id != PORT_A)
		dwc_phy_write_mask(isys, id, PPI_STARTUP_RW_COMMON_DPHY_2, 0x45,
				   0, 7);

	dwc_phy_write_mask(isys, id, PPI_STARTUP_RW_COMMON_DPHY_6, 39, 0, 7);
	dwc_phy_write_mask(isys, id, PPI_CALIBCTRL_RW_COMMON_BG_0, 500, 0, 8);
	dwc_phy_write_mask(isys, id, PPI_RW_TERMCAL_CFG_0, 38, 0, 6);
	dwc_phy_write_mask(isys, id, PPI_RW_OFFSETCAL_CFG_0, 7, 0, 4);
	dwc_phy_write_mask(isys, id, PPI_RW_LPDCOCAL_TIMEBASE, 153, 0, 9);
	dwc_phy_write_mask(isys, id, PPI_RW_LPDCOCAL_NREF, 800, 0, 10);
	dwc_phy_write_mask(isys, id, PPI_RW_LPDCOCAL_NREF_RANGE, 27, 0, 4);
	dwc_phy_write_mask(isys, id, PPI_RW_LPDCOCAL_TWAIT_CONFIG, 47, 0, 8);
	dwc_phy_write_mask(isys, id, PPI_RW_LPDCOCAL_TWAIT_CONFIG, 127, 9, 15);
	dwc_phy_write_mask(isys, id, PPI_RW_LPDCOCAL_VT_CONFIG, 47, 7, 15);
	dwc_phy_write_mask(isys, id, PPI_RW_LPDCOCAL_VT_CONFIG, 27, 2, 6);
	dwc_phy_write_mask(isys, id, PPI_RW_LPDCOCAL_VT_CONFIG, 3, 0, 1);
	dwc_phy_write_mask(isys, id, PPI_RW_LPDCOCAL_COARSE_CFG, 1, 0, 1);
	dwc_phy_write_mask(isys, id, PPI_RW_COMMON_CFG, 3, 0, 1);
	dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_0,
			   0, 10, 10);
	dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_1,
			   1, 10, 10);
	dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_1,
			   0, 15, 15);
	dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_3,
			   3, 8, 9);
	dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_0,
			   0, 15, 15);
	dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_6,
			   7, 12, 14);
	dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_7,
			   0, 8, 10);
	dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_5,
			   0, 8, 8);

	/* DPHY specific */
	dwc_phy_write_mask(isys, id, CORE_DIG_RW_COMMON_7, 0, 0, 9);
	if (mbps > 1500)
		dwc_phy_write_mask(isys, id, PPI_STARTUP_RW_COMMON_DPHY_7,
				   40, 0, 7);
	else
		dwc_phy_write_mask(isys, id, PPI_STARTUP_RW_COMMON_DPHY_7,
				   104, 0, 7);

	dwc_phy_write_mask(isys, id, PPI_STARTUP_RW_COMMON_DPHY_8, 80, 0, 7);
	dwc_phy_write_mask(isys, id, PPI_RW_DDLCAL_CFG_0, 191, 0, 9);
	dwc_phy_write_mask(isys, id, PPI_RW_DDLCAL_CFG_7, 34, 7, 12);
	dwc_phy_write_mask(isys, id, PPI_RW_DDLCAL_CFG_1, 38, 8, 15);
	dwc_phy_write_mask(isys, id, PPI_RW_DDLCAL_CFG_2, 4, 12, 15);
	dwc_phy_write_mask(isys, id, PPI_RW_DDLCAL_CFG_2, 2, 10, 11);
	dwc_phy_write_mask(isys, id, PPI_RW_DDLCAL_CFG_2, 1, 8, 8);
	dwc_phy_write_mask(isys, id, PPI_RW_DDLCAL_CFG_2, 38, 0, 7);
	dwc_phy_write_mask(isys, id, PPI_RW_DDLCAL_CFG_2, 1, 9, 9);
	dwc_phy_write_mask(isys, id, PPI_RW_DDLCAL_CFG_4, 10, 0, 9);
	dwc_phy_write_mask(isys, id, PPI_RW_DDLCAL_CFG_6, 20, 0, 9);
	dwc_phy_write_mask(isys, id, PPI_RW_DDLCAL_CFG_7, 19, 0, 6);

	index = lookup_table0(mbps);
	if (index >= 0)
		dwc_phy_write_mask(isys, id, PPI_RW_DDLCAL_CFG_3,
				   table0[index].ddlcal_counter_ref, 0, 9);

	index = lookup_table1(mbps);
	if (index >= 0) {
		dwc_phy_write_mask(isys, id, PPI_RW_DDLCAL_CFG_1,
				   table1[index].phase_bound, 0, 7);
		dwc_phy_write_mask(isys, id, PPI_RW_DDLCAL_CFG_5,
				   table1[index].ddlcal_dll_fbk, 4, 9);
		dwc_phy_write_mask(isys, id, PPI_RW_DDLCAL_CFG_5,
				   table1[index].ddlcal_ddl_coarse_bank, 0, 3);

		reg = CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_8;
		val = table1[index].oa_lanex_hsrx_cdphy_sel_fast;
		for (i = 0; i < lanes + 1; i++)
			dwc_phy_write_mask(isys, id, reg + (i * 0x400), val,
					   12, 12);
	}

	reg = CORE_DIG_DLANE_0_RW_LP_0;
	for (i = 0; i < lanes; i++)
		dwc_phy_write_mask(isys, id, reg + (i * 0x400), 6, 8, 11);

	dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_2,
			   0, 0, 0);
	if (is_ipu7p5(isys->adev->isp->hw_ver) ||
	    id == PORT_B || id == PORT_C) {
		dwc_phy_write_mask(isys, id,
				   CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_2,
				   1, 0, 0);
		dwc_phy_write_mask(isys, id,
				   CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_2,
				   0, 0, 0);
	} else {
		dwc_phy_write_mask(isys, id,
				   CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_2,
				   0, 0, 0);
		dwc_phy_write_mask(isys, id,
				   CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_2,
				   1, 0, 0);
	}

	if (lanes == 4 && !is_ipu7p5(isys->adev->isp->hw_ver)) {
		dwc_phy_write_mask(isys, id,
				   CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_2,
				   0, 0, 0);
		dwc_phy_write_mask(isys, id,
				   CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_2,
				   0, 0, 0);
	}

	dwc_phy_write_mask(isys, id, CORE_DIG_RW_COMMON_6, 1, 0, 2);
	dwc_phy_write_mask(isys, id, CORE_DIG_RW_COMMON_6, 1, 3, 5);

	reg = CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_12;
	val = (mbps > 1500) ? 0 : 1;
	for (i = 0; i < lanes + 1; i++) {
		dwc_phy_write_mask(isys, id, reg + (i * 0x400), val, 1, 1);
		dwc_phy_write_mask(isys, id, reg + (i * 0x400), !val, 3, 3);
	}

	reg = CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_13;
	val = (mbps > 1500) ? 0 : 1;
	for (i = 0; i < lanes + 1; i++) {
		dwc_phy_write_mask(isys, id, reg + (i * 0x400), val, 1, 1);
		dwc_phy_write_mask(isys, id, reg + (i * 0x400), val, 3, 3);
	}

	index = lookup_table6(mbps);
	if (index >= 0) {
		val = table6[index].oa_lane_hsrx_hs_clk_div;
		if (is_ipu7p5(isys->adev->isp->hw_ver) ||
		    id == PORT_B || id == PORT_C)
			reg = CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_9;
		else
			reg = CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9;

		dwc_phy_write_mask(isys, id, reg, val, 5, 7);
	}

	if (aggregation) {
		dwc_phy_write_mask(isys, id, CORE_DIG_RW_COMMON_0, 1,
				   1, 1);

		reg = CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_15;
		dwc_phy_write_mask(isys, id, reg, 3, 3, 4);

		val = (id == PORT_A) ? 3 : 0;
		reg = CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_15;
		dwc_phy_write_mask(isys, id, reg, val, 3, 4);

		reg = CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_15;
		dwc_phy_write_mask(isys, id, reg, 3, 3, 4);
	}

	dwc_phy_write_mask(isys, id, CORE_DIG_DLANE_CLK_RW_HS_RX_0, 28, 0, 7);
	dwc_phy_write_mask(isys, id, CORE_DIG_DLANE_CLK_RW_HS_RX_7, 6, 0, 7);

	index = lookup_table2(mbps);
	if (index >= 0) {
		val = table2[index].i_thssettle;
		reg = CORE_DIG_DLANE_0_RW_HS_RX_0;
		for (i = 0; i < lanes; i++)
			dwc_phy_write_mask(isys, id, reg + (i * 0x400), val,
					   8, 15);
	}

	/* deskew */
	for (i = 0; i < lanes; i++) {
		reg = CORE_DIG_DLANE_0_RW_CFG_1;
		dwc_phy_write_mask(isys, id, reg + (i * 0x400),
				   ((mbps > 1500) ? 0x1 : 0x2), 2, 3);

		reg = CORE_DIG_DLANE_0_RW_HS_RX_2;
		dwc_phy_write_mask(isys, id, reg + (i * 0x400),
				   ((mbps > 2500) ? 0 : 1), 15, 15);
		dwc_phy_write_mask(isys, id, reg + (i * 0x400), 1, 13, 13);
		dwc_phy_write_mask(isys, id, reg + (i * 0x400), 7, 9, 12);

		reg = CORE_DIG_DLANE_0_RW_LP_0;
		dwc_phy_write_mask(isys, id, reg + (i * 0x400), 1, 12, 15);

		reg = CORE_DIG_DLANE_0_RW_LP_2;
		dwc_phy_write_mask(isys, id, reg + (i * 0x400), 0, 0, 0);

		reg = CORE_DIG_DLANE_0_RW_HS_RX_1;
		dwc_phy_write_mask(isys, id, reg + (i * 0x400), 16, 0, 7);

		reg = CORE_DIG_DLANE_0_RW_HS_RX_3;
		dwc_phy_write_mask(isys, id, reg + (i * 0x400), 2, 0, 2);
		index = lookup_table1(mbps);
		if (index >= 0) {
			val = table1[index].fjump_deskew;
			dwc_phy_write_mask(isys, id, reg + (i * 0x400), val,
					   3, 8);
		}

		reg = CORE_DIG_DLANE_0_RW_HS_RX_4;
		dwc_phy_write_mask(isys, id, reg + (i * 0x400), 150, 0, 15);

		reg = CORE_DIG_DLANE_0_RW_HS_RX_5;
		dwc_phy_write_mask(isys, id, reg + (i * 0x400), 0, 0, 7);
		dwc_phy_write_mask(isys, id, reg + (i * 0x400), 1, 8, 15);

		reg = CORE_DIG_DLANE_0_RW_HS_RX_6;
		dwc_phy_write_mask(isys, id, reg + (i * 0x400), 2, 0, 7);
		index = lookup_table1(mbps);
		if (index >= 0) {
			val = table1[index].min_eye_opening_deskew;
			dwc_phy_write_mask(isys, id, reg + (i * 0x400), val,
					   8, 15);
		}
		reg = CORE_DIG_DLANE_0_RW_HS_RX_7;
		dwc_phy_write_mask(isys, id, reg + (i * 0x400), 0, 13, 13);
		dwc_phy_write_mask(isys, id, reg + (i * 0x400), 0, 15, 15);

		reg = CORE_DIG_DLANE_0_RW_HS_RX_9;
		index = lookup_table1(mbps);
		if (index >= 0) {
			val = table1[index].ddlcal_max_phase;
			dwc_phy_write_mask(isys, id, reg + (i * 0x400),
					   val, 0, 7);
		}
	}

	dwc_phy_write_mask(isys, id, CORE_DIG_DLANE_CLK_RW_LP_0, 1, 12, 15);
	dwc_phy_write_mask(isys, id, CORE_DIG_DLANE_CLK_RW_LP_2, 0, 0, 0);

	for (i = 0; i < ARRAY_SIZE(deskew_fine_mem); i++)
		dwc_phy_write_mask(isys, id, CORE_DIG_COMMON_RW_DESKEW_FINE_MEM,
				   deskew_fine_mem[i], 0, 15);

	if (mbps <= 1500) {
		hsrxval0 = 0;
		hsrxval1 = 0;
		hsrxval2 = 0;
	}

	if (mbps > 1500) {
		hsrxval0 = 4;
		hsrxval1 = 0;
		hsrxval2 = 3;
	}

	if (mbps > 2500)
		hsrxval1 = 2;

	dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_9,
			   hsrxval0, 0, 2);
	dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_9,
			   hsrxval0, 0, 2);
	dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9,
			   hsrxval0, 0, 2);
	if (lanes == 4 && !is_ipu7p5(isys->adev->isp->hw_ver)) {
		dwc_phy_write_mask(isys, id,
				   CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_9,
				   hsrxval0, 0, 2);
		dwc_phy_write_mask(isys, id,
				   CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_9,
				   hsrxval0, 0, 2);
	}

	dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_9,
			   hsrxval1, 3, 4);
	dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_9,
			   hsrxval1, 3, 4);
	dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_9,
			   hsrxval1, 3, 4);
	if (lanes == 4 && !is_ipu7p5(isys->adev->isp->hw_ver)) {
		dwc_phy_write_mask(isys, id,
				   CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_9,
				   hsrxval1, 3, 4);
		dwc_phy_write_mask(isys, id,
				   CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_9,
				   hsrxval1, 3, 4);
	}

	dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_15,
			   hsrxval2, 0, 2);
	dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_15,
			   hsrxval2, 0, 2);
	dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_15,
			   hsrxval2, 0, 2);
	if (lanes == 4 && !is_ipu7p5(isys->adev->isp->hw_ver)) {
		dwc_phy_write_mask(isys, id,
				   CORE_DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_15,
				   hsrxval2, 0, 2);
		dwc_phy_write_mask(isys, id,
				   CORE_DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_15,
				   hsrxval2, 0, 2);
	}

	/* force and override rext */
	if (isys->phy_rext_cal && id) {
		dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_8,
				   isys->phy_rext_cal, 0, 3);
		dwc_phy_write_mask(isys, id, CORE_DIG_IOCTRL_RW_AFE_CB_CTRL_2_7,
				   1, 11, 11);
	}
}

int ipu7_isys_csi_phy_powerup(struct ipu7_isys_csi2 *csi2)
{
	struct ipu7_isys *isys = csi2->isys;
	u32 lanes = csi2->nlanes;
	bool aggregation = false;
	u32 id = csi2->port;
	int ret;

	/* lanes remapping for aggregation (port AB) mode */
	if (is_ipu7p5(isys->adev->isp->hw_ver) && lanes > 2 && id == PORT_A) {
		aggregation = true;
		lanes = 2;
	}

	ipu7_isys_csi_phy_reset(isys, id);
	gpreg_write(isys, id, PHY_CLK_LANE_CONTROL, 0x1);
	gpreg_write(isys, id, PHY_CLK_LANE_FORCE_CONTROL, 0x2);
	gpreg_write(isys, id, PHY_LANE_CONTROL_EN, (1 << lanes) - 1);
	gpreg_write(isys, id, PHY_LANE_FORCE_CONTROL, 0xf);
	gpreg_write(isys, id, PHY_MODE, PHY_MODE_DPHY);

	/* config PORT_B if aggregation mode */
	if (aggregation) {
		ipu7_isys_csi_phy_reset(isys, PORT_B);
		gpreg_write(isys, PORT_B, PHY_CLK_LANE_CONTROL, 0x0);
		gpreg_write(isys, PORT_B, PHY_LANE_CONTROL_EN, 0x3);
		gpreg_write(isys, PORT_B, PHY_CLK_LANE_FORCE_CONTROL, 0x2);
		gpreg_write(isys, PORT_B, PHY_LANE_FORCE_CONTROL, 0xf);
		gpreg_write(isys, PORT_B, PHY_MODE, PHY_MODE_DPHY);
	}

	ipu7_isys_csi_ctrl_cfg(csi2);
	/* TODO: get the DT and VC from the frame descriptor */
	ipu7_isys_csi_ctrl_dids_config(isys, id, 0, MIPI_CSI2_DT_RAW10);

	ipu7_isys_phy_config(isys, id, lanes, aggregation);
	gpreg_write(isys, id, PHY_RESET, 1);
	gpreg_write(isys, id, PHY_SHUTDOWN, 1);
	dwc_csi_write(isys, id, DPHY_RSTZ, 1);
	dwc_csi_write(isys, id, PHY_SHUTDOWNZ, 1);
	dwc_csi_write(isys, id, CSI2_RESETN, 1);

	ret = ipu7_isys_phy_ready(isys, id);
	if (ret < 0)
		return ret;

	gpreg_write(isys, id, PHY_LANE_FORCE_CONTROL, 0);
	gpreg_write(isys, id, PHY_CLK_LANE_FORCE_CONTROL, 0);

	/* config PORT_B if aggregation mode */
	if (aggregation) {
		ipu7_isys_phy_config(isys, PORT_B, 2, aggregation);
		gpreg_write(isys, PORT_B, PHY_RESET, 1);
		gpreg_write(isys, PORT_B, PHY_SHUTDOWN, 1);
		dwc_csi_write(isys, PORT_B, DPHY_RSTZ, 1);
		dwc_csi_write(isys, PORT_B, PHY_SHUTDOWNZ, 1);
		dwc_csi_write(isys, PORT_B, CSI2_RESETN, 1);
		ret = ipu7_isys_phy_ready(isys, PORT_B);
		if (ret < 0)
			return ret;

		gpreg_write(isys, PORT_B, PHY_LANE_FORCE_CONTROL, 0);
		gpreg_write(isys, PORT_B, PHY_CLK_LANE_FORCE_CONTROL, 0);
	}

	return 0;
}

void ipu7_isys_csi_phy_powerdown(struct ipu7_isys_csi2 *csi2)
{
	struct ipu7_isys *isys = csi2->isys;

	ipu7_isys_csi_phy_reset(isys, csi2->port);
	if (is_ipu7p5(isys->adev->isp->hw_ver) &&
	    csi2->nlanes > 2 && csi2->port == PORT_A)
		ipu7_isys_csi_phy_reset(isys, PORT_B);
}
