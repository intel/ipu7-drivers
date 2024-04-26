// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IPU_MSG_NODE_H_INCLUDED__
#define IPU_MSG_NODE_H_INCLUDED__

/**
 * @addtogroup ipu_msg_abi
 * @{
 */

#include "ipu_msg_header.h"
#include "ia_gofo_common_abi.h"

#pragma pack(push, 1)

/** Maximum terminals per node */
#define IPU_MSG_MAX_NODE_TERMS (64U)

/** Maximum image fragments */
#define IPU_MSG_MAX_FRAGS (7U)

/** Enumeration of different kinds of nodes. */
enum ipu_msg_node_type {
	/** Type zero is always padding */
	IPU_MSG_NODE_TYPE_PAD = 0,
	/** Basic generic node.  May be extended with options. */
	IPU_MSG_NODE_TYPE_BASE,
	IPU_MSG_NODE_TYPE_N
};

/**
 * Maximum number of devices possible in a CB
 * @todo This number copied from CB HAS "MAX_NUM_OF_DEVICE".  Should get from geneated headers.
 * @todo The actual number can be much smaller, so INTERNAL data structures might benefit from
 * lower memory use.  Consider differentiating ABI definition from internal usage.
 */
#define IPU_MSG_NODE_MAX_DEVICES (128U)

/** Number of 32 bit integers in a DEB */
#define DEB_NUM_UINT32 (IPU_MSG_NODE_MAX_DEVICES/(sizeof(uint32_t) * 8U))

/** Type for Terminal Enable Bitmaps (TEB) */
typedef uint32_t ipu_msg_teb_t[2];

/** Type for Device Enable Bitmap (DEB) bitmaps */
typedef uint32_t ipu_msg_deb_t[DEB_NUM_UINT32];

/**
 * Maximum number of routing bits within a CB
 * @todo This number copied from CB HAS "MAX_RBM_SIZE".  Should get from geneated headers.
 */
#define IPU_MSG_NODE_MAX_ROUTE_ENABLES (128U)

/** Number of 32 bit integers in a REB */
#define RBM_NUM_UINT32 (IPU_MSG_NODE_MAX_ROUTE_ENABLES/(sizeof(uint32_t) * 8U))

/** Type for Routing BitMap (RBM) */
typedef uint32_t ipu_msg_rbm_t[RBM_NUM_UINT32];

/** Type for Route Enable Bitmap (REB). Tied to RBM definition. */
typedef ipu_msg_rbm_t ipu_msg_reb_t;

/** TLV types for node profiles.  All types must extend IPU_MSG_NODE_PROFILE_TYPE_BASE */
enum ipu_msg_node_profile_type {
	/** Type zero is always padding */
	IPU_MSG_NODE_PROFILE_TYPE_PAD = 0,
	/** Profile for a basic generic node - see ipu_msg_node_profile */
	IPU_MSG_NODE_PROFILE_TYPE_BASE,
	/** Profile for a CB node.- see ipu_msg_cb_profile */
	IPU_MSG_NODE_PROFILE_TYPE_CB,
	IPU_MSG_NODE_PROFILE_TYPE_N
};

/**
 * Node profile for generic nodes.  Represents one of possibly several ways to configure the
 * terminals of a node within the same node context.
 */
struct ipu_msg_node_profile {
	/** Type here is one of ipu_msg_node_profile_type */
	struct ia_gofo_tlv_header tlv_header;
	/** Terminal Enable Bitmap - must be a subset of that declared during graph open. */
	ipu_msg_teb_t teb;
};

/** Node profile for CB nodes.  Includes additional bitmaps that CB nodes require */
struct ipu_msg_cb_profile {
	/** Generic node profile.  CB extensions follow. */
	struct ipu_msg_node_profile profile_base;
	/** Internal CB Device Enable Bitmask */
	ipu_msg_deb_t deb;
	/** Internal CB Routing BitMask */
	ipu_msg_rbm_t rbm;
	/** Internal CB Routing element Enable Bitmask */
	ipu_msg_reb_t reb;
};

/** Maximum number of profiles allowed in ipu_msg_node::profile_list */
#define IPU_MSG_NODE_MAX_PROFILES (2U)

/** Default profile index.  Must always exist as there must always be at least one profile. */
#define IPU_MSG_NODE_DEF_PROFILE_IDX (0U)

/**
 *  Special node resource ID to identify a generic external node.  Required when there is a link
 *  to/from IPU and that node.
 *
 *  Usage of this ID assumes that all external IP's have the same abstracted streaming link
 *  interface with IPU, such that IPU need not know or care who they really are.  Thus we need not
 *  define specific ID's for each external node and maintain it separately per project.
 *
 *  If an external node has a interface requiring specific behavior from IPU, then a specific
 *  resource ID must be defined higher in the stack, just like IPU nodes.
 *
 *  Generic EXT_IP node instances (see ipu_msg_node) should use a single node profile (see
 *  ipu_msg_node::profiles_list) with TLV type IPU_MSG_NODE_PROFILE_TYPE_BASE and structure
 *  ipu_msg_node_profile.  ipu_msg_node_profile::teb should be set to all-one's, as if all terminals
 *  are enabled.
 *
 *  Links to/from EXT_IP nodes should specify the node node_ctx_id of the EXT_IP instance as for all
 *  links in their endpoints.  The endpoint terminal ID should be set to
 *  IPU_MSG_LINK_TERM_ID_DONT_CARE.
 */
#define IPU_MSG_NODE_RSRC_ID_EXT_IP (0xFFU)

/**
 * External IP Nodes have all their TEB bits set.
 * @{
 */
#define IPU_MSG_NODE_DONT_CARE_TEB_HI (0xFFFFFFFFU)
#define IPU_MSG_NODE_DONT_CARE_TEB_LO (0xFFFFFFFFU)
/** @} */

/**
 * Node resource ID of INSYS, required when there is a link from INSYS to PSYS.
 */
#define IPU_MSG_NODE_RSRC_ID_IS (0xFEU)

/** Base structure for all node objects */
struct ipu_msg_node {
	/** Type here is one of ipu_msg_node_type */
	struct ia_gofo_tlv_header tlv_header;
	/**
	 *  Identifier of node resource ID in the manifest graph model.  See also
	 *  IPU_MSG_NODE_RSRC_ID_EXT_IP, IPU_MSG_NODE_RSRC_ID_IS
	 */
	uint8_t node_rsrc_id;
	/**
	 * Zero-based index of this node in the fgraph.  Must be numbered in the
	 * same order as the node appears in the fgraph's node-list.
	 */
	uint8_t node_ctx_id;
	/** Number of fragments for all future frames in this node context */
	uint8_t num_frags;
	/** Alignment.  Set to zero. */
	uint8_t reserved[1];
	/**
	 * List of configuration alternatives for this node.  At least one is required.
	 * All TLV profile types must derive from ipu_msg_node_profile.  Profile order is
	 * significant as task generation will be defined by the the order of the profiles
	 * in the list.  See struct ipu_msg_node_profile.
	 */
	struct ia_gofo_tlv_list profiles_list;
	/**
	 * List of terminal objects of type @see ipu_msg_term or derivations.
	 * Only enabled terminals are allowed - and even then they are optional.  Generally,
	 * terminals need not be in the list if they have no special options as the superteb
	 * defines which are enabled.
	 */
	struct ia_gofo_tlv_list terms_list;
	/** Extension options per node */
	struct ia_gofo_tlv_list node_options;
};

/** Enumeration of node option types */
enum ipu_msg_node_option_types {
	/** All TLV headers must define value zero as padding */
	IPU_MSG_NODE_OPTION_TYPES_PADDING = 0,
	/** Number of items in this enumeration */
	IPU_MSG_NODE_OPTION_TYPES_N
};

#pragma pack(pop)

/** @} */

/** Compile time checks only - this function is not to be called! */
static inline void ipu_msg_node_test_func(void)
{
	CHECK_ALIGN32(struct ipu_msg_node);
	CHECK_ALIGN32(struct ipu_msg_node_profile);
	CHECK_ALIGN32(struct ipu_msg_cb_profile);
}

#endif

