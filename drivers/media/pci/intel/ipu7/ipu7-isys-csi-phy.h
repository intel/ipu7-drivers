/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2013 - 2024 Intel Corporation
 */

#ifndef IPU7_ISYS_CSI_PHY_H
#define IPU7_ISYS_CSI_PHY_H

#define PHY_MODE_DPHY		0
#define PHY_MODE_CPHY		1

void ipu7_isys_csi_ctrl_init(struct ipu_isys *isys,
			     struct ipu_isys_csi2_config *cfg);
void ipu7_isys_csi_ctrl_reset(struct ipu_isys *isys,
			      struct ipu_isys_csi2_config *cfg);
int ipu7_isys_csi_ctrl_dids_config(struct ipu_isys *isys, u32 id, u8 vc, u8 dt);
int ipu7_isys_csi_phy_powerup(struct ipu_isys *isys,
			      struct ipu_isys_csi2_config *cfg);
void ipu7_isys_csi_phy_powerdown(struct ipu_isys *isys,
				 struct ipu_isys_csi2_config *cfg);
#endif
