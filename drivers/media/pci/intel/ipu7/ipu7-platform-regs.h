/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018 - 2024 Intel Corporation
 */

#ifndef IPU7_PLATFORM_REGS_H
#define IPU7_PLATFORM_REGS_H

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

#define IPU7_ISYS_CSI_PORT_NUM		4

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
enum ipu7_device_buttress_psys_domain_pos {
	IPU_PSYS_SUBDOMAIN_LB		= 0,
	IPU_PSYS_SUBDOMAIN_BB		= 1,
};

#define IPU_PSYS_DOMAIN_POWER_MASK  (BIT(IPU_PSYS_SUBDOMAIN_LB) | \
				     BIT(IPU_PSYS_SUBDOMAIN_BB))
#define IPU_PSYS_DOMAIN_POWER_IN_PROGRESS	BIT(31)

#endif /* IPU7_PLATFORM_REGS_H */
