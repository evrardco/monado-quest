// Copyright 2019-2024, Collabora, Ltd.
// Copyright 2025-2026, NVIDIA CORPORATION.
// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pretty printing various Vulkan things.
 *
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Christoph Haag <christoph.haag@collabora.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup aux_vk
 */

#pragma once

#include "xrt/xrt_compiler.h"
#include "xrt/xrt_vulkan_includes.h"


#ifdef __cplusplus
extern "C" {
#endif


/*
 *
 * String helper functions.
 *
 */

/*!
 * @name Enum string functions
 *
 * Returns a string of the enum value, or NULL if unknown. The versions without
 * the `_or_null` suffix return 'UNKNOWN <TYPE>' instead of NULL.
 *
 * @ingroup aux_vk
 * @{
 */

XRT_CHECK_RESULT const char *
vk_str_result_or_null(VkResult code);

XRT_CHECK_RESULT const char *
vk_str_object_type_or_null(VkObjectType type);

XRT_CHECK_RESULT const char *
vk_str_physical_device_type_or_null(VkPhysicalDeviceType device_type);

XRT_CHECK_RESULT const char *
vk_str_format_or_null(VkFormat code);

XRT_CHECK_RESULT const char *
vk_str_sharing_mode_or_null(VkSharingMode code);

XRT_CHECK_RESULT const char *
vk_str_present_mode_or_null(VkPresentModeKHR code);

XRT_CHECK_RESULT const char *
vk_str_color_space_or_null(VkColorSpaceKHR code);

XRT_CHECK_RESULT const char *
vk_str_display_power_state_or_null(VkDisplayPowerStateEXT code);

XRT_CHECK_RESULT const char *
vk_str_queue_global_priority_or_null(VkQueueGlobalPriorityEXT priority);

/*!
 * @}
 */

#define VK_STR_NO_NULL(NAME, TYPE, UNKNOWN_STR)                                                                        \
	static inline const char *NAME(TYPE enumerate)                                                                 \
	{                                                                                                              \
		const char *str = NAME##_or_null(enumerate);                                                           \
		return str != NULL ? str : UNKNOWN_STR;                                                                \
	}

VK_STR_NO_NULL(vk_str_result, VkResult, "UNKNOWN RESULT")
VK_STR_NO_NULL(vk_str_object_type, VkObjectType, "UNKNOWN OBJECT TYPE")
VK_STR_NO_NULL(vk_str_physical_device_type, VkPhysicalDeviceType, "UNKNOWN PHYSICAL DEVICE TYPE")
VK_STR_NO_NULL(vk_str_format, VkFormat, "UNKNOWN FORMAT")
VK_STR_NO_NULL(vk_str_sharing_mode, VkSharingMode, "UNKNOWN SHARING MODE")
VK_STR_NO_NULL(vk_str_present_mode, VkPresentModeKHR, "UNKNOWN PRESENT MODE")
VK_STR_NO_NULL(vk_str_color_space, VkColorSpaceKHR, "UNKNOWN COLOR SPACE")
VK_STR_NO_NULL(vk_str_display_power_state, VkDisplayPowerStateEXT, "UNKNOWN DISPLAY POWER STATE")
VK_STR_NO_NULL(vk_str_queue_global_priority, VkQueueGlobalPriorityEXT, "UNKNOWN QUEUE GLOBAL PRIORITY")


/*
 *
 * Flag bits string functions.
 *
 */

/*!
 * Returns the format feature flag if one valid bit is set,
 * if multiple bits are set, will return 'MULTIPLE BIT SET'.
 */
XRT_CHECK_RESULT const char *
vk_format_feature_flag_string(VkFormatFeatureFlagBits bits, bool null_on_unknown);

/*!
 * Returns the swapchain create flag if one valid bit is set,
 * if multiple bits are set, will return 'MULTIPLE BIT SET'.
 */
XRT_CHECK_RESULT const char *
vk_swapchain_create_flag_string(VkSwapchainCreateFlagsKHR bits, bool null_on_unknown);

/*!
 * Returns the image usage flag if one valid bit is set,
 * if multiple bits are set, will return 'MULTIPLE BIT SET'.
 */
XRT_CHECK_RESULT const char *
vk_image_usage_flag_string(VkImageUsageFlagBits bits, bool null_on_unknown);

/*!
 * Returns the composite alpha flag if one valid bit is set,
 * if multiple bits are set, will return 'MULTIPLE BIT SET'.
 */
XRT_CHECK_RESULT const char *
vk_composite_alpha_flag_string(VkCompositeAlphaFlagBitsKHR bits, bool null_on_unknown);

/*!
 * Returns the surface transform flag if one valid bit is set,
 * if multiple bits are set, will return 'MULTIPLE BIT SET'.
 */
XRT_CHECK_RESULT const char *
vk_surface_transform_flag_string(VkSurfaceTransformFlagBitsKHR bits, bool null_on_unknown);

#ifdef VK_KHR_display
/*!
 * Returns the display plane alpha flag if one valid bit is set,
 * if multiple bits are set, will return 'MULTIPLE BIT SET'.
 */
XRT_CHECK_RESULT const char *
vk_display_plane_alpha_flag_string(VkDisplayPlaneAlphaFlagBitsKHR bits, bool null_on_unknown);
#endif // VK_KHR_display


#undef VK_STR_NO_NULL

#ifdef __cplusplus
}
#endif
