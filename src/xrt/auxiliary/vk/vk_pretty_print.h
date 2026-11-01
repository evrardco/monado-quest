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

XRT_CHECK_RESULT const char *
vk_result_string(VkResult code);

XRT_CHECK_RESULT const char *
vk_object_type_string(VkObjectType type);

XRT_CHECK_RESULT const char *
vk_physical_device_type_string(VkPhysicalDeviceType device_type);

XRT_CHECK_RESULT const char *
vk_format_string(VkFormat code);

XRT_CHECK_RESULT const char *
vk_sharing_mode_string(VkSharingMode code);

XRT_CHECK_RESULT const char *
vk_present_mode_string(VkPresentModeKHR code);

XRT_CHECK_RESULT const char *
vk_color_space_string(VkColorSpaceKHR code);

XRT_CHECK_RESULT const char *
vk_power_state_string(VkDisplayPowerStateEXT code);

XRT_CHECK_RESULT const char *
vk_queue_priority_string(VkQueueGlobalPriorityEXT priority);


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


#ifdef __cplusplus
}
#endif
