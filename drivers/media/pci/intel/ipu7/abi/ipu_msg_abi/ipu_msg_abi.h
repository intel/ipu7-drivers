// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IPU_MSG_ABI_H_INCLUDED__
#define IPU_MSG_ABI_H_INCLUDED__

/**
 * @defgroup ipu_msg_abi IPU Processing System (PS) Messaging Application Binary Interface
 * See @ref ipu_sw_fw_states "IPU SW FW ABI State Machine"
 * and
 * @ref ipu_sw_fw_msgs "IPU SW FW ABI Messaging Format and FGraph Model"
 * @{
 */

/**
 * @file ipu_msg_abi.h
 * Convenience header to include all sub-headers describing the
 * messaging protocol between IPU software and firmware
 *
 * @ref ipu_sw_fw_states "IPU SW FW ABI State Machine"
 */

/**
 * Tool not yet available
 * @mscfile ipu_sw_fw.signalling "SW-FW Interface Sequence
 * (Place holder - MSCGEN tool integration not yet available"
 */

#include "ia_gofo_msg_indirect.h"
#include "ipu_msg_device.h"
#include "ipu_msg_graph.h"

/** @} */

#endif

