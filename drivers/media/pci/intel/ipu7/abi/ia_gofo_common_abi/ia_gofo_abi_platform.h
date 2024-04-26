// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IA_GOFO_ABI_PLATFORM_H_INCLUDED__
#define IA_GOFO_ABI_PLATFORM_H_INCLUDED__

#ifndef NO_IA_GOFO_ABI_PLATFORM_DEPS
#include "ia_gofo_abi_platform_deps.h"
#endif

#ifndef IA_GOFO_MODULO
/**
 *  Simple modulo.  Required due to way some assert macro's work where the expression
 *  is string-ified and then the percent sign could be erroneously interpreted as part of a
 *  format string.
 */
#define IA_GOFO_MODULO(dividend, divisor) ((dividend) % (divisor))
#endif

#ifndef IA_GOFO_ASSERT
/** Empty definition to allow headers to compile if NO_IA_GOFO_ABI_PLATFORM_DEPS is defined */
#define IA_GOFO_ASSERT(expr)
#endif

#endif /* __IA_GOFO_ABI_PLATFORM_H_INCLUDED__ */
