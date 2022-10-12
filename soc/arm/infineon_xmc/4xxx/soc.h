/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2020 Linumiz
 * Author: Parthiban Nallathambi <parthiban@linumiz.com>
 *
 */

#ifdef CONFIG_SOC_XMC4500
#include <system_XMC4500.h>
#include <XMC4500.h>
#define PMU_FLASH_WS		(0x3U)
#endif

#ifdef CONFIG_SOC_XMC4800
#include <system_XMC4800.h>
#include <XMC4800.h>
#define PMU_FLASH_WS		(0x4U)
#endif
