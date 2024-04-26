/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2018 - 2024 Intel Corporation */

#ifndef IPU_PLATFORM_REGS_H
#define IPU_PLATFORM_REGS_H

#define IS_BASE					0x230000
#define IS_UC_CTRL_BASE				(IS_BASE + 0x0)

#define PS_BASE					0x130000
#define PS_UC_CTRL_BASE				(PS_BASE + 0x0)

/*
 * bit 0: IRQ from FW,
 * bit 1, 2 and 3: IRQ from HW
 */
#define TO_SW_IRQ_MASK				0xf
#define TO_SW_IRQ_FW				BIT(0)

#define FW_CODE_BASE				0x0
#define FW_DATA_BASE				0x4
#define CPU_AXI_CNTL				0x8
#define CPU_QOS_CNTL				0xc
#define IDMA_AXI_CNTL				0x10
#define IDMA_QOS_CNTL				0x14
#define MEF_SPLIT_SIZE				0x18
#define FW_MSG_CONTROL				0x1c
#define FW_MSG_CREDITS_STATUS			0x20
#define FW_MSG_CREDIT_TAKEN			0x24
#define FW_MSG_CREDIT_RETURNED			0x28
#define TRIG_IDMA_IN				0x2c
#define IDMA_DONE				0x30
#define IDMA_DONE_CLEAR				0x34
#define DMEM_CAPACITY				0x38
#define NON_SECURE_CODE_OFFSET			0x3c
#define UC_CG_CTRL_BITS				0x40
#define ALT_RESET_VEC				0x44
#define WDT_NMI_DURATION			0x104
#define WDT_RST_REQ_DURATION			0x108
#define WDT_CNTL				0x10c
#define WDT_NMI_CURRENT_COUNT			0x110
#define WDT_RST_CURRENT_COUNT			0x114
#define WDT_HALT				0x118
#define WDT_STATUS				0x11c
#define SPARE_REG_RW				0x120
#define SPARE_REG_RO				0x124
#define FW_TO_FW_IRQ_CNTL_EDGE			0x200
#define FW_TO_FW_IRQ_CNTL_MASK_N		0x204
#define FW_TO_FW_IRQ_CNTL_STATUS		0x208
#define FW_TO_FW_IRQ_CNTL_CLEAR			0x20c
#define FW_TO_FW_IRQ_CNTL_ENABLE		0x210
#define FW_TO_FW_IRQ_CNTL_LEVEL_NOT_PULSE	0x214
#define CLK_GATE_DIS				0x218
#define DEBUG_STATUS				0x1000
#define DEBUG_EXCPETION				0x1004
#define TIE_GENERAL_INPUT			0x1008
#define ERR_STATUS				0x100c
#define UC_ERR_INFO				0x1010
#define SPARE_CNTL				0x1014
#define MEF_TRC_CNTL				0x1100
#define DBG_MEF_LAST_PUSH			0x1104
#define DBG_MEF_LAST_POP			0x1108
#define DBG_MEF_COUNT_CNTL			0x110c
#define DBG_MEF_COUNT1				0x1110
#define DBG_MEF_COUNT2				0x1114
#define DBG_MEF_ACC_OCCUPANCY			0x1118
#define DBG_MEF_MAX_IRQ_TO_POP			0x111c
#define DBG_IRQ_CNTL				0x1120
#define DBG_IRQ_COUNT				0x1124
#define DBG_CYC_COUNT				0x1128
#define DBG_CNTL				0x1130
#define DBG_RST_REG				0x1134
#define DBG_MEF_STATUS0				0x1138
#define DBG_MEF_STATUS1				0x113c
#define PDEBUG_CTL				0x1140
#define PDEBUG_DATA				0x1144
#define PDEBUG_INST				0x1148
#define PDEBUG_LS0ADDR				0x114c
#define PDEBUG_LS0DATA				0x1150
#define PDEBUG_LS0STAT				0x1154
#define PDEBUG_PC				0x1158
#define PDEBUG_MISC				0x115c
#define PDEBUG_PREF_STS				0x1160
#define MEF0_ADDR				0x2000
#define MEF1_ADDR				0x2020
#define PRINTF_EN_THROUGH_TRACE			0x3004
#define PRINTF_EN_DIRECTLY_TO_DDR		0x3008
#define PRINTF_DDR_BASE_ADDR			0x300c
#define PRINTF_DDR_SIZE				0x3010
#define PRINTF_DDR_NEXT_ADDR			0x3014
#define PRINTF_STATUS				0x3018
#define PRINTF_AXI_CNTL				0x301c
#define PRINTF_MSG_LENGTH			0x3020
#define TO_SW_IRQ_CNTL_EDGE			0x4000
#define TO_SW_IRQ_CNTL_MASK_N			0x4004
#define TO_SW_IRQ_CNTL_STATUS			0x4008
#define TO_SW_IRQ_CNTL_CLEAR			0x400c
#define TO_SW_IRQ_CNTL_ENABLE			0x4010
#define TO_SW_IRQ_CNTL_LEVEL_NOT_PULSE		0x4014
#define ERR_IRQ_CNTL_EDGE			0x4018
#define ERR_IRQ_CNTL_MASK_N			0x401c
#define ERR_IRQ_CNTL_STATUS			0x4020
#define ERR_IRQ_CNTL_CLEAR			0x4024
#define ERR_IRQ_CNTL_ENABLE			0x4028
#define ERR_IRQ_CNTL_LEVEL_NOT_PULSE		0x402c
#define LOCAL_DMEM_BASE_ADDR			0x1300000

/*
 * IS_UC_TO_SW irqs
 * bit 0: IRQ from local FW
 * bit 1~3: IRQ from HW
 */
#define IS_UC_TO_SW_IRQ_MASK			0xf

/*
 * IPU6 uses uniform address within IPU, therefore all subsystem registers
 * locates in one signle space starts from 0 but in different sctions with
 * different addresses, the subsystem offsets are defined to 0 as the
 * register definition will have the address offset to 0.
 */
#define IPU_ISYS_SPC_OFFSET		0x210000

#define IPU7_PSYS_SPC_OFFSET		0x118000

#define IPU_ISYS_DMEM_OFFSET		0x200000
#define IPU_PSYS_DMEM_OFFSET		0x100000

#define IPU_REG_ISYS_CSI_TOP_CTRL0_IRQ_EDGE		0x238200
#define IPU_REG_ISYS_CSI_TOP_CTRL0_IRQ_MASK		0x238204
#define IPU_REG_ISYS_CSI_TOP_CTRL0_IRQ_STATUS		0x238208
#define IPU_REG_ISYS_CSI_TOP_CTRL0_IRQ_CLEAR		0x23820c
#define IPU_REG_ISYS_CSI_TOP_CTRL0_IRQ_ENABLE		0x238210
#define IPU_REG_ISYS_CSI_TOP_CTRL0_IRQ_LEVEL_NOT_PULSE	0x238214

#define IPU_REG_ISYS_CSI_TOP_CTRL1_IRQ_EDGE		0x238220
#define IPU_REG_ISYS_CSI_TOP_CTRL1_IRQ_MASK		0x238224
#define IPU_REG_ISYS_CSI_TOP_CTRL1_IRQ_STATUS		0x238228
#define IPU_REG_ISYS_CSI_TOP_CTRL1_IRQ_CLEAR		0x23822c
#define IPU_REG_ISYS_CSI_TOP_CTRL1_IRQ_ENABLE		0x238230
#define IPU_REG_ISYS_CSI_TOP_CTRL1_IRQ_LEVEL_NOT_PULSE	0x238234

#define IPU_REG_ISYS_ISL_TOP_IRQ_EDGE			0x2b0200
#define IPU_REG_ISYS_ISL_TOP_IRQ_MASK			0x2b0204
#define IPU_REG_ISYS_ISL_TOP_IRQ_STATUS			0x2b0208
#define IPU_REG_ISYS_ISL_TOP_IRQ_CLEAR			0x2b020c
#define IPU_REG_ISYS_ISL_TOP_IRQ_ENABLE			0x2b0210
#define IPU_REG_ISYS_ISL_TOP_IRQ_LEVEL_NOT_PULSE	0x2b0214

#define IPU_REG_ISYS_CMPR_TOP_IRQ_EDGE			0x2d2100
#define IPU_REG_ISYS_CMPR_TOP_IRQ_MASK			0x2d2104
#define IPU_REG_ISYS_CMPR_TOP_IRQ_STATUS		0x2d2108
#define IPU_REG_ISYS_CMPR_TOP_IRQ_CLEAR			0x2d210c
#define IPU_REG_ISYS_CMPR_TOP_IRQ_ENABLE		0x2d2110
#define IPU_REG_ISYS_CMPR_TOP_IRQ_LEVEL_NOT_PULSE	0x2d2114

#define IPU_ISYS_CSI_PHY_NUM				2
#define IPU_CSI_IRQ_NUM_PER_PIPE			4
#define IPU7_ISYS_CSI_PORT_NUM				4
/* Maximum 4 virtual channels supported */
#define IPU_MAX_VC_NUMS					4

#define IPU_ISYS_CSI_PORT_IRQ(irq_num)		(1 << (irq_num))

#define IPU_ISYS_REG_SPC_STATUS_CTRL		0x0

#define IPU_ISYS_SPC_STATUS_START			BIT(1)
#define IPU_ISYS_SPC_STATUS_RUN				BIT(3)
#define IPU_ISYS_SPC_STATUS_READY			BIT(5)
#define IPU_ISYS_SPC_STATUS_CTRL_ICACHE_INVALIDATE	BIT(12)
#define IPU_ISYS_SPC_STATUS_ICACHE_PREFETCH		BIT(13)

#define IPU_PSYS_REG_SPC_STATUS_CTRL		0x0
#define IPU_PSYS_REG_SPC_START_PC		0x4
#define IPU_PSYS_REG_SPC_ICACHE_BASE		0x10
#define IPU_REG_PSYS_INFO_SEG_0_CONFIG_ICACHE_MASTER	0x14

#define IPU_PSYS_SPC_STATUS_START			BIT(1)
#define IPU_PSYS_SPC_STATUS_RUN				BIT(3)
#define IPU_PSYS_SPC_STATUS_READY			BIT(5)
#define IPU_PSYS_SPC_STATUS_CTRL_ICACHE_INVALIDATE	BIT(12)
#define IPU_PSYS_SPC_STATUS_ICACHE_PREFETCH		BIT(13)

#define IPU_PSYS_REG_SPP0_STATUS_CTRL			0x20000

#define IPU_INFO_ENABLE_SNOOP			BIT(0)
#define IPU_INFO_DEC_FORCE_FLUSH		BIT(1)
#define IPU_INFO_DEC_PASS_THROUGH		BIT(2)
#define IPU_INFO_ZLW                            BIT(3)
#define IPU_INFO_STREAM_ID_SET(id)		(((id) & 0x1F) << 4)
#define IPU_INFO_REQUEST_DESTINATION_IOSF	BIT(9)
#define IPU_INFO_IMR_BASE			BIT(10)
#define IPU_INFO_IMR_DESTINED			BIT(11)

#define IPU_INFO_REQUEST_DESTINATION_PRIMARY IPU_INFO_REQUEST_DESTINATION_IOSF

/* Trace unit related register definitions */
#define TRACE_REG_MAX_ISYS_OFFSET	0xfffff
#define TRACE_REG_MAX_PSYS_OFFSET	0xfffff
#define IPU_ISYS_OFFSET			IPU_ISYS_DMEM_OFFSET
#define IPU_PSYS_OFFSET			IPU_PSYS_DMEM_OFFSET
/* ISYS trace unit registers */
/* Trace unit base offset */
#define IPU_TRACE_REG_IS_TRACE_UNIT_BASE		0x27d000
/* Trace monitors */
#define IPU_TRACE_REG_IS_SP_EVQ_BASE		0x211000
/* GPC blocks */
#define IPU_TRACE_REG_IS_SP_GPC_BASE		0x210800
#define IPU_TRACE_REG_IS_ISL_GPC_BASE		0x2b0a00
#define IPU_TRACE_REG_IS_MMU_GPC_BASE		0x2e0f00
/* each CSI2 port has a dedicated trace monitor, index 0..7 */
#define IPU_TRACE_REG_CSI2_TM_BASE(port)	(0x220400 + 0x1000 * (port))

/* Trace timers */
#define IPU_TRACE_REG_IS_GPREG_TRACE_TIMER_RST_N		0x27c410
#define TRACE_REG_GPREG_TRACE_TIMER_RST_OFF		BIT(0)

/* SIG2CIO */
#define IPU_TRACE_REG_CSI2_PORT_SIG2SIO_GR_BASE(port)		\
			(0x220e00 + (port) * 0x1000)

/* PSYS trace unit registers */
/* Trace unit base offset */
#define IPU_TRACE_REG_PS_TRACE_UNIT_BASE		0x1b4000
/* Trace monitors */
#define IPU_TRACE_REG_PS_SPC_EVQ_BASE			0x119000
#define IPU_TRACE_REG_PS_SPP0_EVQ_BASE			0x139000

/* GPC blocks */
#define IPU_TRACE_REG_PS_SPC_GPC_BASE			0x118800
#define IPU_TRACE_REG_PS_SPP0_GPC_BASE			0x138800
#define IPU_TRACE_REG_PS_MMU_GPC_BASE			0x1b1b00

/* Trace timers */
#define IPU_TRACE_REG_PS_GPREG_TRACE_TIMER_RST_N	0x1aa714

/*
 * s2m_pixel_soc_pixel_remapping is dedicated for the enableing of the
 * pixel s2m remp ability.Remap here  means that s2m rearange the order
 * of the pixels in each 4 pixels group.
 * For examle, mirroring remping means that if input's 4 first pixels
 * are 1 2 3 4 then in output we should see 4 3 2 1 in this 4 first pixels.
 * 0xE4 is from s2m MAS document. It means no remaping.
 */
#define S2M_PIXEL_SOC_PIXEL_REMAPPING_FLAG_NO_REMAPPING 0xE4
/*
 * csi_be_soc_pixel_remapping is for the enabling of the csi\mipi be pixel
 * remapping feature. This remapping is exactly like the stream2mmio remapping.
 */
#define CSI_BE_SOC_PIXEL_REMAPPING_FLAG_NO_REMAPPING    0xE4

/* IRQ-related registers in PSYS */
#define IPU_REG_PSYS_TO_SW_IRQ_CNTL_EDGE		0x134000
#define IPU_REG_PSYS_TO_SW_IRQ_CNTL_MASK		0x134004
#define IPU_REG_PSYS_TO_SW_IRQ_CNTL_STATUS		0x134008
#define IPU_REG_PSYS_TO_SW_IRQ_CNTL_CLEAR		0x13400c
#define IPU_REG_PSYS_TO_SW_IRQ_CNTL_ENABLE		0x134010
#define IPU_REG_PSYS_TO_SW_IRQ_CNTL_LEVEL_NOT_PULSE	0x134014
#define IRQ_FROM_LOCAL_FW				BIT(0)

/*
 * psys subdomains power request regs
 */
enum ipu_device_buttress_psys_domain_pos {
	IPU_PSYS_SUBDOMAIN_LB		= 0,
	IPU_PSYS_SUBDOMAIN_BB		= 1,
};

#define IPU_PSYS_DOMAIN_POWER_MASK  (BIT(IPU_PSYS_SUBDOMAIN_LB) | \
				     BIT(IPU_PSYS_SUBDOMAIN_BB))
#define IPU_PSYS_DOMAIN_POWER_IN_PROGRESS	BIT(31)

#endif /* IPU_PLATFORM_REGS_H */
