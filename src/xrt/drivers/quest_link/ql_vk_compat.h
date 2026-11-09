// Copyright 2026
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief The Vulkan helper names this driver (and the WiVRn encoders it uses) were written against
 * (Monado 2023), mapped onto current upstream's: vk_result_string -> vk_str_result, and the
 * vk_has_error / vk_check_error checks, which upstream replaced with the VK_CHK_* macros.
 */
#pragma once

#include "vk/vk_helpers.h"
#include "vk/vk_pretty_print.h"
#include "util/u_logging.h"

#include <stdbool.h>

static inline const char *
vk_result_string(VkResult result)
{
	return vk_str_result(result);
}

//! Logs and returns true if @p res is an error.
static inline bool
vk_has_error(VkResult res, const char *fun, const char *file, int line)
{
	if (res == VK_SUCCESS) {
		return false;
	}
	U_LOG_E("%s failed with %s in %s:%d", fun, vk_str_result(res), file, line);
	return true;
}

#define vk_check_error(fun, res, ret)                                                                                  \
	do {                                                                                                           \
		if (vk_has_error(res, fun, __FILE__, __LINE__))                                                        \
			return ret;                                                                                    \
	} while (0)
