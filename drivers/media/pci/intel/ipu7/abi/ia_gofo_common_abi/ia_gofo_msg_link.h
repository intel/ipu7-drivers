// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IA_GOFO_MSG_LINK_H_INCLUDED__
#define IA_GOFO_MSG_LINK_H_INCLUDED__

/**
 * @addtogroup ia_gofo_msg_abi
 * @{
 */

#pragma pack(push, 1)

/** Data movement mechanisms between producer and consumer */
enum ia_gofo_msg_link_streaming_mode {
	/**
	 * Static Offline.
	 * This value is required to be set when FW is responsible to manage the data flow
	 * accros a link without streaming hardware support.
	 *
	 * If SW manages the timing of this data flow by holding back task requests, it
	 * should not define as a link on this interface.
	 */
	IA_GOFO_MSG_LINK_STREAMING_MODE_SOFF = 0,
	/** Dynamic Offline */
	IA_GOFO_MSG_LINK_STREAMING_MODE_DOFF = 1,
	/** Buffer chasing large memory */
	IA_GOFO_MSG_LINK_STREAMING_MODE_BCLM = 2,
	/** Buffer chasing small memory with fix starting point */
	IA_GOFO_MSG_LINK_STREAMING_MODE_BCSM_FIX = 3,
	/** Number of items in this enumeration */
	IA_GOFO_MSG_LINK_STREAMING_MODE_N
};

/**
 * SOC PBKs.
 * Enumeration of all HW PBKs instances which are external to specifics IPs.
 * Those PBK used for cross IPs connection.
 */
enum ia_gofo_soc_pbk_instance_id {
	IA_GOFO_SOC_PBK_ID0 = 0,
	IA_GOFO_SOC_PBK_ID1 = 1,
	IA_GOFO_SOC_PBK_ID_N
};

/** Maxinum number of PBK slots that may be allocated per link  */
#define IA_GOFO_MSG_LINK_PBK_MAX_SLOTS (2U)

#pragma pack(pop)

/**  @} */

#endif
