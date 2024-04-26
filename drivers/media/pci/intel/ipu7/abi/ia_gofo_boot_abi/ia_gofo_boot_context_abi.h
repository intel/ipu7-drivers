// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IA_GOFO_BOOT_CONTEXT_ABI_H_INCLUDED__
#define IA_GOFO_BOOT_CONTEXT_ABI_H_INCLUDED__

/**
 * Syscom primary context index
 *
 * In Intel SoC (CCG) it is used as Unsecured boot single syscom context OR unsecure (untrusted) context for secure boot
 * In Stand-alone SoC (HKR) it is used as the Feature context
 */
#define IA_GOFO_BOOT_CONTEXT_PRIMARY (0U)

/**
 * Syscom secondary context index
 *
 * In Intel SoC (CCG) it is used as Secure (trusted) context for secure boot
 * In Stand-alone SoC (HKR) it is used as the Safety context
 */
#define IA_GOFO_BOOT_CONTEXT_SECONDARY (1U)

/** Maximum number of syscom contexts supported  */
#define IA_GOFO_BOOT_MAX_CONTEXTS (((uint32_t)IA_GOFO_BOOT_CONTEXT_SECONDARY) + ((uint32_t)1U))

#endif /* IA_GOFO_BOOT_CONTEXT_ABI_H_INCLUDED__ */
