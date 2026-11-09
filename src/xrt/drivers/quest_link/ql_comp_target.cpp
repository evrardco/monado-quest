/*
 * WiVRn VR streaming
 * Copyright (C) 2022  Guillaume Meunier <guillaume.meunier@centraliens.net>
 * Copyright (C) 2022  Patrick Nicolas <patricknicolas@laposte.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "render/render_interface.h" // render_calc_uv_to_tangent_lengths_rect
#include <cmath>
#include "ql_comp_target.h"
#include "main/comp_compositor.h"
#include "math/m_space.h"
#include "util/u_pacing.h"
#include "video_encoder.h"
#include "xrt/xrt_config_have.h"
#include "xrt_cast.h"
#include <atomic>
#include <condition_variable>
#include <list>
#include <map>
#include <vector>
#include <vulkan/vulkan_core.h>

#include "ql_types.h"
#include "ql_xrsp.h"
#include "ql_system.h"

namespace xrt::drivers::quest_link
{

static const uint8_t image_free = 0;
static const uint8_t image_acquired = 1;

struct pseudo_swapchain
{
	struct pseudo_swapchain_memory
	{
		VkFence fence;
		bool submitted; //!< `fence` belongs to a submit (it's created unsignaled)
		//! Records the layout change and the encoders' commands each present; reused (see comp_ql_present).
		VkCommandBuffer cmd;
		VkDeviceMemory memory;
		uint8_t status; // bitmask of consumer status, index 0 for acquired, the rest for each encoder
		uint64_t frame_index;
		xrt::drivers::wivrn::to_headset::video_stream_data_shard::view_info_t view_info{};
	} * images{};
	std::mutex mutex;
	std::condition_variable cv;
};

struct ql_comp_target : public comp_target
{
	ql_xrsp_host* host;

	//! Compositor frame pacing helper
	struct u_pacing_compositor * upc{};

	float fps;

	int64_t current_frame_id;

	int64_t last_avg_tx;
	int64_t last_avg_enc;
	int frames_since_encode_adjust;

	// Monotonic counter, for video stream
	uint64_t frame_index = 0;

	struct pseudo_swapchain psc;

	VkColorSpaceKHR color_space;

	struct vk_cmd_pool comp_pool;
	VkCommandPool cmd_pool;

	struct encoder_thread
	{
		int index;
		os_thread_helper thread;

		encoder_thread()
		{
			os_thread_helper_init(&thread);
		}
		~encoder_thread()
		{
			os_thread_helper_stop_and_wait(&thread);
			os_thread_helper_destroy(&thread);
		}
	};
	std::list<encoder_thread> encoder_threads;
	std::vector<std::shared_ptr<xrt::drivers::wivrn::VideoEncoder>> encoders;
};

static void target_init_semaphores(struct ql_comp_target * cn);

static void target_fini_semaphores(struct ql_comp_target * cn);

static inline struct vk_bundle * get_vk(struct ql_comp_target * cn)
{
	return &cn->c->base.vk;
}

static void destroy_images(struct ql_comp_target * cn)
{
	if (cn->images == NULL)
	{
		return;
	}

	cn->encoder_threads.clear();
	cn->encoders.clear();

	struct vk_bundle * vk = get_vk(cn);

	for (uint32_t i = 0; i < cn->image_count; i++)
	{
		if (cn->images[i].view == VK_NULL_HANDLE)
		{
			continue;
		}

		auto & img = cn->psc.images[i];
		if (img.submitted)
		{
			// The command buffer and the image may still be in use by the GPU.
			(void)vk->vkWaitForFences(vk->device, 1, &img.fence, VK_TRUE, UINT64_MAX);
		}
		if (img.cmd != VK_NULL_HANDLE)
			vk->vkFreeCommandBuffers(vk->device, cn->cmd_pool, 1, &img.cmd);
		if (img.fence != VK_NULL_HANDLE)
			vk->vkDestroyFence(vk->device, img.fence, NULL);

		vk->vkDestroyImageView(vk->device, cn->images[i].view, NULL);
		vk->vkDestroyImage(vk->device, cn->images[i].handle, NULL);
		vk->vkFreeMemory(vk->device, cn->psc.images[i].memory, NULL);
	}

	free(cn->images);
	free(cn->psc.images);
	cn->images = NULL;
	cn->psc.images = NULL;

	target_fini_semaphores(cn);
}

struct encoder_thread_param
{
	ql_comp_target * cn;
	ql_comp_target::encoder_thread * thread;
	std::vector<std::shared_ptr<xrt::drivers::wivrn::VideoEncoder>> encoders;
	std::map<uint8_t, uint32_t> idr_generation_seen; //!< per slice, see ql_xrsp_host::idr_generation
};

static void * comp_ql_present_thread(void * void_param);

static void create_encoders(ql_comp_target * cn, std::vector<xrt::drivers::wivrn::encoder_settings> & _settings)
{
	auto vk = get_vk(cn);
	assert(cn->encoders.empty());
	assert(cn->encoder_threads.empty());

	xrt::drivers::wivrn::to_headset::video_stream_description desc{};
	desc.width = cn->width;
	desc.height = cn->height;
	desc.fps = cn->fps;

	cn->last_avg_tx = 0;
	cn->last_avg_enc = 0;
	cn->frames_since_encode_adjust = 0;

	std::map<int, encoder_thread_param> thread_params;

	for (auto & settings: _settings)
	{
		uint8_t stream_index = cn->encoders.size();

		int slice_w = desc.width;
		int slice_h = desc.height / QL_NUM_SLICES;
		for (int slice_num = 0; slice_num < QL_NUM_SLICES; slice_num++)
		{
			auto & encoder = cn->encoders.emplace_back(
			        xrt::drivers::wivrn::VideoEncoder::Create(vk, settings, stream_index, slice_num, QL_NUM_SLICES, slice_w, slice_h, desc.fps));
			encoder->SetXrspHost(cn->host);

			std::vector<VkImage> images(cn->image_count);
			std::vector<VkDeviceMemory> memory(cn->image_count);
			std::vector<VkImageView> views(cn->image_count);

			for (size_t j = 0; j < cn->image_count; j++)
			{
				images[j] = cn->images[j].handle;
				memory[j] = cn->psc.images[j].memory;
				views[j] = cn->images[j].view;
			}
			encoder->SetImages(cn->width, cn->height, cn->format, cn->image_count, images.data(), views.data(), memory.data());
			thread_params[settings.group].encoders.emplace_back(encoder);
		}

		desc.items.push_back(settings);
	}

	for (auto & [group, params]: thread_params)
	{
		auto params_ptr = new encoder_thread_param(params);
		auto & thread = cn->encoder_threads.emplace_back();
		thread.index = cn->encoder_threads.size() - 1;
		params_ptr->thread = &thread;
		params_ptr->cn = cn;
		os_thread_helper_start(&thread.thread, comp_ql_present_thread, params_ptr);
	}
	//cn->cnx->send_control(desc);
}

class drm_image_modifier_helper
{
	VkImageDrmFormatModifierListCreateInfoEXT drm_info{};
	std::vector<uint64_t> modifiers;

public:
	void * pNext = nullptr;
	drm_image_modifier_helper(vk_bundle * vk, VkFormat format, VkImageTiling tiling)
	{
		if (tiling != VK_IMAGE_TILING_DRM_FORMAT_MODIFIER_EXT)
			return;

		VkDrmFormatModifierPropertiesListEXT drm_list{};
		drm_list.sType = VK_STRUCTURE_TYPE_DRM_FORMAT_MODIFIER_PROPERTIES_LIST_EXT;

		VkFormatProperties2 format_prop{};
		format_prop.sType = VK_STRUCTURE_TYPE_FORMAT_PROPERTIES_2;
		format_prop.pNext = &drm_list;

		vk->vkGetPhysicalDeviceFormatProperties2(vk->physical_device, format, &format_prop);

		std::vector<VkDrmFormatModifierPropertiesEXT> properties(drm_list.drmFormatModifierCount);
		drm_list.pDrmFormatModifierProperties = properties.data();

		vk->vkGetPhysicalDeviceFormatProperties2(vk->physical_device, format, &format_prop);

		VkFormatFeatureFlags required_features = VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT |
		                                         VK_FORMAT_FEATURE_COLOR_ATTACHMENT_BIT |
		                                         VK_FORMAT_FEATURE_TRANSFER_SRC_BIT;
		for (const auto & mod: properties)
		{
			// VideoEncoderVA hands VAAPI a single-object, single-plane DRM frame, so skip
			// multi-plane modifiers (AMD DCC compression adds metadata planes). Newer Mesa picks
			// those when allowed and the VAAPI import then fails with "resource allocation failed".
			if (mod.drmFormatModifierPlaneCount != 1)
				continue;
			if ((mod.drmFormatModifierTilingFeatures & required_features) == required_features)
				modifiers.push_back(mod.drmFormatModifier);
		}

		assert(not modifiers.empty());

		drm_info.sType = VK_STRUCTURE_TYPE_IMAGE_DRM_FORMAT_MODIFIER_LIST_CREATE_INFO_EXT;
		drm_info.drmFormatModifierCount = modifiers.size();
		drm_info.pDrmFormatModifiers = modifiers.data();

		pNext = &drm_info;
	}
};

static VkResult create_images(struct ql_comp_target * cn, VkImageUsageFlags flags, VkImageTiling tiling)
{
	struct vk_bundle * vk = get_vk(cn);

	assert(cn->image_count > 0);
	COMP_DEBUG(cn->c, "Creating %d images.", cn->image_count);

	destroy_images(cn);

	cn->images = U_TYPED_ARRAY_CALLOC(struct comp_target_image, cn->image_count);
	cn->psc.images = U_TYPED_ARRAY_CALLOC(pseudo_swapchain::pseudo_swapchain_memory, cn->image_count);

	VkImageSubresourceRange subresource_range{};
	subresource_range.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	subresource_range.baseMipLevel = 0;
	subresource_range.levelCount = 1;
	subresource_range.baseArrayLayer = 0;
	subresource_range.layerCount = 1;

	flags |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

	drm_image_modifier_helper drm_list(vk, cn->format, tiling);

	for (uint32_t i = 0; i < cn->image_count; i++)
	{
		auto & image = cn->images[i].handle;
		auto & image_view = cn->images[i].view;
		auto & memory = cn->psc.images[i].memory;

		VkExtent3D extent{cn->width, cn->height, 1};

		// The encoder (VAAPI) imports these images as DMA-BUFs, so they must be created and allocated
		// as exportable. Older RADV exported non-exportable memory anyway; Mesa 26 doesn't.
		VkExternalMemoryImageCreateInfo external_info{};
		external_info.sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO;
		external_info.pNext = drm_list.pNext;
		external_info.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT;

		VkImageCreateInfo image_info{};
		image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
		image_info.pNext = &external_info;
		image_info.imageType = VK_IMAGE_TYPE_2D;
		image_info.format = cn->format;
		image_info.extent = extent;
		image_info.mipLevels = 1;
		image_info.arrayLayers = 1;
		image_info.samples = VK_SAMPLE_COUNT_1_BIT;
		image_info.tiling = tiling;
		image_info.usage = flags;
		// The compositor renders into these on its main queue, and we submit the layout change (and
		// the x264 converter's commands) on the graphics queue. With XRT_COMPOSITOR_COMPUTE=1 upstream
		// Monado's main queue is a compute queue of another family: share the images between the two
		// rather than leave them owned by one family.
		const uint32_t families[2] = {vk->main_queue->family_index, vk->graphics_queue->family_index};
		const bool shared = families[0] != families[1];
		image_info.sharingMode = shared ? VK_SHARING_MODE_CONCURRENT : VK_SHARING_MODE_EXCLUSIVE;
		image_info.queueFamilyIndexCount = shared ? 2 : 0;
		image_info.pQueueFamilyIndices = shared ? families : NULL;
		image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

		VkResult res = vk->vkCreateImage(vk->device, &image_info, NULL, &image);
		vk_check_error("vkCreateImage", res, res);

		VkMemoryDedicatedAllocateInfoKHR dedicated_allocate_info{};
		dedicated_allocate_info.sType = VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO;
		dedicated_allocate_info.image = image;

		VkExportMemoryAllocateInfo export_info{};
		export_info.sType = VK_STRUCTURE_TYPE_EXPORT_MEMORY_ALLOCATE_INFO;
		export_info.pNext = &dedicated_allocate_info;
		export_info.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT;

		VkMemoryRequirements requirements;
		vk->vkGetImageMemoryRequirements(vk->device, image, &requirements);
		res = vk_alloc_and_bind_image_memory(vk, image, &requirements, &export_info, "ql_comp_target", &memory);
		vk_check_error("vk_alloc_and_bind_image_memory", res, res);

		res = vk_create_view(          //
		        vk,                    // vk_bundle
		        image,                 // image
		        VK_IMAGE_VIEW_TYPE_2D, // type
		        cn->format,            // format
		        subresource_range,     // subresource_range
		        &image_view);          // out_view
		vk_check_error("vk_create_view", res, res);

		VkFenceCreateInfo createinfo{};
		createinfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
		createinfo.pNext = NULL;
		createinfo.flags = 0;

		res = vk->vkCreateFence(vk->device, &createinfo, NULL, &cn->psc.images[i].fence);

		vk_check_error("vkCreateFence", res, res);
	}

	return VK_SUCCESS;
}

static bool comp_ql_init_pre_vulkan(struct comp_target * ct)
{
	return true;
}

static bool comp_ql_init_post_vulkan(struct comp_target * ct, uint32_t preferred_width, uint32_t preferred_height)
{
	struct ql_comp_target * cn = (struct ql_comp_target *)ct;
	struct vk_bundle * vk = get_vk(cn);

	/*
	 * Command buffer pool, needs to go first.
	 */
	//TODO error check
	vk_cmd_pool_init(vk, &cn->comp_pool, VK_COMMAND_POOL_CREATE_TRANSIENT_BIT);

	// RESET_COMMAND_BUFFER: comp_ql_present re-records one command buffer per image.
	VkCommandPoolCreateInfo command_pool_info = {
	    .sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
	    .flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT | VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT,
	    // Graphics work goes here (layout changes, the x264 converter's render passes): a compute-only
	    // queue got graphics commands once and hung the GPU (2026-10-28, illegal opcode).
	    .queueFamilyIndex = vk->graphics_queue->family_index,
	};

	vk->vkCreateCommandPool(vk->device, &command_pool_info, NULL, &cn->cmd_pool);

	return true;
}

static bool comp_ql_check_ready(struct comp_target * ct)
{
	return true;
}

static void target_fini_semaphores(struct ql_comp_target * cn)
{
	struct vk_bundle * vk = get_vk(cn);

	if (cn->semaphores.present_complete != VK_NULL_HANDLE)
	{
		vk->vkDestroySemaphore(vk->device, cn->semaphores.present_complete, NULL);
		cn->semaphores.present_complete = VK_NULL_HANDLE;
	}

	if (cn->semaphores.render_complete != VK_NULL_HANDLE)
	{
		vk->vkDestroySemaphore(vk->device, cn->semaphores.render_complete, NULL);
		cn->semaphores.render_complete = VK_NULL_HANDLE;
	}
}

static void target_init_semaphores(struct ql_comp_target * cn)
{
	struct vk_bundle * vk = get_vk(cn);
	VkResult ret;

	target_fini_semaphores(cn);

	VkSemaphoreCreateInfo info = {
	        .sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO,
	};

	ret = vk->vkCreateSemaphore(vk->device, &info, NULL, &cn->semaphores.present_complete);
	if (ret != VK_SUCCESS)
	{
		COMP_ERROR(cn->c, "vkCreateSemaphore: %s", vk_result_string(ret));
	}

	cn->semaphores.render_complete_is_timeline = false;
	ret = vk->vkCreateSemaphore(vk->device, &info, NULL, &cn->semaphores.render_complete);
	if (ret != VK_SUCCESS)
	{
		COMP_ERROR(cn->c, "vkCreateSemaphore: %s", vk_result_string(ret));
	}
}

static void comp_ql_create_images(struct comp_target * ct,
                                     const struct comp_target_create_images_info * create_info,
                                     struct vk_bundle_queue * present_queue)
{
	struct ql_comp_target * cn = (struct ql_comp_target *)ct;
	const VkImageUsageFlags image_usage = create_info->image_usage;
	const VkColorSpaceKHR preferred_color_space = create_info->color_space;
	// The first format the compositor accepts that the encoders take (BGRA8, see ffmpeg_helper.cpp);
	// in compute mode the compositor lists UNORM first (it can't write sRGB storage images).
	VkFormat preferred_color_format = VK_FORMAT_UNDEFINED;
	for (uint32_t i = 0; i < create_info->format_count && preferred_color_format == VK_FORMAT_UNDEFINED; i++)
	{
		if (create_info->formats[i] == VK_FORMAT_B8G8R8A8_UNORM || create_info->formats[i] == VK_FORMAT_B8G8R8A8_SRGB)
			preferred_color_format = create_info->formats[i];
	}
	if (preferred_color_format == VK_FORMAT_UNDEFINED)
	{
		U_LOG_E("Quest Link: the compositor offers no BGRA8 format the video encoder can take");
		abort();
	}

	uint64_t now_ns = os_monotonic_get_ns();
	if (cn->upc == NULL)
	{
		u_pc_fake_create(ct->c->settings.nominal_frame_interval_ns, now_ns, &cn->upc);
	}

	// Free old images.
	destroy_images(cn);

	target_init_semaphores(cn);

	ct->format = preferred_color_format;
	ct->width = cn->host->sys->hmd->encode_width;//preferred_width;
	ct->height = cn->host->sys->hmd->encode_height;//preferred_height;
	ct->surface_transform = VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR;

	cn->image_count = 3;
	cn->format = preferred_color_format;
	cn->width = cn->host->sys->hmd->encode_width;//preferred_width;
	cn->height = cn->host->sys->hmd->encode_height;//preferred_height;
	cn->color_space = preferred_color_space;

	auto settings = xrt::drivers::wivrn::get_encoder_settings(get_vk(cn), cn->width, cn->height);

	VkResult res = create_images(cn, image_usage, get_required_tiling(get_vk(cn), settings));
	if (vk_has_error(res, "create_images", __FILE__, __LINE__))
	{
		// TODO
		abort();
	}

	try
	{
		create_encoders(cn, settings);
	}
	catch (const std::exception & e)
	{
		U_LOG_E("Failed to create video encoder: %s", e.what());
		abort();
	}
}

static bool comp_ql_has_images(struct comp_target * ct)
{
	return ct->images;
}

static VkResult comp_ql_acquire(struct comp_target * ct, uint32_t * out_index)
{
	struct ql_comp_target * cn = (struct ql_comp_target *)ct;
	struct vk_bundle * vk = get_vk(cn);

	VkSubmitInfo submit{};
	submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submit.pNext = NULL;
	submit.waitSemaphoreCount = 0;
	submit.pWaitSemaphores = NULL;
	submit.pWaitDstStageMask = NULL;
	submit.commandBufferCount = 0;
	submit.pCommandBuffers = NULL;
	submit.signalSemaphoreCount = 1;
	submit.pSignalSemaphores = &cn->semaphores.present_complete;

	VkResult res = vk_cmd_submit_locked(vk, vk->graphics_queue, 1, &submit, VK_NULL_HANDLE);
	vk_check_error("vk_cmd_submit_locked", res, res);

	res = VK_ERROR_OUT_OF_HOST_MEMORY;

	while (res != VK_SUCCESS)
	{
		std::this_thread::yield();

		std::lock_guard lock(cn->psc.mutex);
		for (uint32_t i = 0; i < ct->image_count; i++)
		{
			if (cn->psc.images[i].status == image_free)
			{
				cn->psc.images[i].status = image_acquired;

				*out_index = i;
				res = VK_SUCCESS;
				break;
			}
		}
		//printf("Not Acquire?\n");
	};

	return res;
}

static void * comp_ql_present_thread(void * void_param)
{
	std::unique_ptr<encoder_thread_param> param((encoder_thread_param *)void_param);
	struct ql_comp_target * cn = param->cn;
	struct vk_bundle * vk = get_vk(cn);
	COMP_DEBUG(cn->c, "Starting encoder thread %d", param->thread->index);
	os_thread_name_self(("encoder " + std::to_string(param->thread->index)).c_str());

	uint8_t status_bit = 1 << (param->thread->index + 1);

	uint64_t skipped = 0, skipped_logged_ns = 0;
	while (os_thread_helper_is_running(&param->thread->thread))
	{
		int presenting_index = -1;
		{
			std::unique_lock lock(cn->psc.mutex);

			// Encode the newest frame waiting for us. This used to take the lowest swapchain index,
			// which with two frames waiting could encode them out of order. Older waiting frames
			// are skipped: they'd only arrive later still, and freeing them keeps the compositor
			// from running out of images.
			for (uint32_t i = 0; i < cn->image_count; i++)
			{
				if ((cn->psc.images[i].status & status_bit) and
				    (presenting_index < 0 or cn->psc.images[i].frame_index > cn->psc.images[presenting_index].frame_index))
					presenting_index = i;
			}
			for (uint32_t i = 0; presenting_index >= 0 and i < cn->image_count; i++)
			{
				if ((int)i != presenting_index and (cn->psc.images[i].status & status_bit))
				{
					cn->psc.images[i].status &= ~status_bit;
					skipped++;
				}
			}

			if (presenting_index < 0)
			{
				// condition variable is not notified when we want to stop thread,
				// use a timeout, but longer than a typical frame
				cn->psc.cv.wait_for(lock, std::chrono::milliseconds(50));
				//std::this_thread::yield();
				continue;
			}
		}
		if (skipped > 0 and os_monotonic_get_ns() - skipped_logged_ns > U_TIME_1S_IN_NS)
		{
			COMP_WARN(cn->c, "Encoder behind: skipped %llu older frame(s)", (unsigned long long)skipped);
			skipped = 0;
			skipped_logged_ns = os_monotonic_get_ns();
		}

		VkResult res = vk->vkWaitForFences(vk->device, 1, &cn->psc.images[presenting_index].fence, VK_TRUE, UINT64_MAX);
		vk_check_error("vkWaitForFences", res, NULL);

		if (++cn->frames_since_encode_adjust > 120)
		{
			if (cn->last_avg_enc / 1000000.0 > 1.0) {
				for (auto & encoder: param->encoders)
				{
					encoder->ModifyBitrate(-1000000);
				}
			}
			else if (cn->last_avg_enc / 1000000.0 < 1.0) {
				for (auto & encoder: param->encoders)
				{
					encoder->ModifyBitrate(100000);
				}
			}
			cn->frames_since_encode_adjust = 0;
		}
		

		// Copied: once the image is released (input_done below) the compositor may overwrite it.
		const auto psc_image = cn->psc.images[presenting_index];
		bool released = false;
		auto release = [&] {
			std::lock_guard lock(cn->psc.mutex);
			if (not released)
				cn->psc.images[presenting_index].status &= ~status_bit;
			released = true;
		};
#ifdef XRT_HAVE_VT // TODO: nvenc etc etc
		for (int i = 0; i < QL_NUM_SLICES; i++) {
			cn->host->start_encode(cn->host, psc_image.view_info.display_time, presenting_index, i);
		}
#endif

		try
		{
			// A new stream (first connection or USB reconnect) starts with a keyframe: see
			// idr_generation in ql_types.h.
			const uint32_t idr_generation = __atomic_load_n(&cn->host->idr_generation, __ATOMIC_ACQUIRE);
			for (auto & encoder: param->encoders)
			{
				uint32_t & seen = param->idr_generation_seen[encoder->slice_idx];
				bool idr_requested = seen != idr_generation;
				seen = idr_generation;

#ifndef XRT_HAVE_VT // TODO: nvenc etc etc
				cn->host->start_encode(cn->host, psc_image.view_info.display_time, presenting_index, encoder->slice_idx);
#endif
				// The VAAPI encoder releases the image once it has converted it (input_done), the
				// others when Encode returns. Only the last encoder of the group may release it.
				std::function<void()> input_done;
				if (&encoder == &param->encoders.back())
					input_done = release;
				encoder->Encode(nullptr, psc_image.view_info, psc_image.frame_index, presenting_index, idr_requested, input_done);
			}
		}
		catch (...)
		{
			// Ignore errors
		}
#if 0
		for (int i = 0; i < QL_NUM_SLICES; i++) {
			cn->host->start_encode(cn->host, psc_image.view_info.display_time, presenting_index, i);
		}
#endif

		release();
	}

	return NULL;
}

/*!
 * Debug (QL_VIEW_DEBUG=1): every 2 s, what a frame tells the headset (display time relative to now, both
 * eyes' FOV and position, their distance). Added 2026-11-09 to both Monado trees to compare the 2023 fork
 * with the upstream port, after the port felt like a wrong FOV in the headset.
 */
static void
ql_debug_log_view_info(const xrt::drivers::wivrn::to_headset::video_stream_data_shard::view_info_t & vi, const char * tree)
{
	static int enabled = -1;
	if (enabled < 0)
	{
		const char * env = getenv("QL_VIEW_DEBUG");
		enabled = env != NULL && atoi(env) != 0;
	}
	static int64_t last_ns = 0;
	const int64_t now = os_monotonic_get_ns();
	if (!enabled || now - last_ns < 2 * U_TIME_1S_IN_NS)
		return;
	last_ns = now;
	const double deg = 180.0 / 3.14159265358979323846;
	const XrVector3f & l = vi.pose[0].position;
	const XrVector3f & r = vi.pose[1].position;
	const double ipd_mm = 1000.0 * std::sqrt((r.x - l.x) * (r.x - l.x) + (r.y - l.y) * (r.y - l.y) + (r.z - l.z) * (r.z - l.z));
	U_LOG_I("QL view (%s): display %+.2f ms from now | L fov l %.1f r %.1f u %.1f d %.1f pos %+.3f %+.3f %+.3f q %+.3f %+.3f %+.3f %+.3f"
	        " | R fov l %.1f r %.1f u %.1f d %.1f pos %+.3f %+.3f %+.3f q %+.3f %+.3f %+.3f %+.3f | eye distance %.1f mm",
	        tree, ((int64_t)vi.display_time - now) / 1e6, vi.fov[0].angleLeft * deg, vi.fov[0].angleRight * deg,
	        vi.fov[0].angleUp * deg, vi.fov[0].angleDown * deg, l.x, l.y, l.z, vi.pose[0].orientation.x,
	        vi.pose[0].orientation.y, vi.pose[0].orientation.z, vi.pose[0].orientation.w, vi.fov[1].angleLeft * deg,
	        vi.fov[1].angleRight * deg, vi.fov[1].angleUp * deg, vi.fov[1].angleDown * deg, r.x, r.y, r.z,
	        vi.pose[1].orientation.x, vi.pose[1].orientation.y, vi.pose[1].orientation.z, vi.pose[1].orientation.w, ipd_mm);
}

/*!
 * The FOV of an image laid out with @p r = render_calc_uv_to_tangent_lengths_rect(fov): its inverse
 * (x = tan left, w = tan right - tan left, y = -tan up, h = tan up - tan down).
 */
static xrt_fov
fov_from_uv_to_tanangle(const xrt_normalized_rect & r)
{
	xrt_fov fov;
	fov.angle_left = atanf(r.x);
	fov.angle_right = atanf(r.x + r.w);
	fov.angle_up = atanf(-r.y);
	fov.angle_down = atanf(-r.y - r.h);
	return fov;
}

/*!
 * Make the next frames' image follow the driver's current FOV. Upstream's compute path lays the image
 * out with render_resources' uv_to_tanangle (copied into the shader's pre_transforms every frame), set
 * once at startup; this driver changes the outer FOV with the headset's IPD (ql_xrsp_pose.cpp: 52/49/43
 * deg for 58/63/68 mm), and the headset wants images for its current optics: the 2023 fork's
 * compositor followed it. With the startup FOV (49) on 68 mm lenses (43) the world looked too small
 * and the stereo was off (2026-11-09). The headset even reports a placeholder IPD (64.9 mm) while Link
 * starts, so this can't be settled once at startup. The distortion lookup images are in texture space
 * (ql_hmd_compute_distortion doesn't use the FOV), so only this rect changes. Runs on the compositor
 * thread between frames, like the rendering that reads it.
 */
static void
follow_distortion_fov(struct ql_comp_target * cn, const xrt_fov image_fov[2])
{
	struct ql_xrsp_host * host = cn->host;
	xrt_fov current[2];
	os_mutex_lock(&host->pose_mutex); // ql_xrsp_handle_pose writes it under this lock
	current[0] = cn->c->xdev->hmd->distortion.fov[0];
	current[1] = cn->c->xdev->hmd->distortion.fov[1];
	os_mutex_unlock(&host->pose_mutex);

	const float deg = 180.f / 3.14159265f;
	for (int eye = 0; eye < 2; eye++)
	{
		const xrt_fov & c = current[eye];
		const xrt_fov & i = image_fov[eye];
		if (std::fabs(c.angle_left - i.angle_left) < 0.05f / deg && std::fabs(c.angle_right - i.angle_right) < 0.05f / deg &&
		    std::fabs(c.angle_up - i.angle_up) < 0.05f / deg && std::fabs(c.angle_down - i.angle_down) < 0.05f / deg)
			continue;
		render_calc_uv_to_tangent_lengths_rect(&c, &cn->c->nr.distortion.uv_to_tanangle[eye]);
		U_LOG_I("Quest Link: eye %d's image FOV follows the headset's: l %.1f r %.1f u %.1f d %.1f deg (was l %.1f r %.1f)",
		        eye, c.angle_left * deg, c.angle_right * deg, c.angle_up * deg, c.angle_down * deg, i.angle_left * deg,
		        i.angle_right * deg);
	}
}

static VkResult comp_ql_present(struct comp_target * ct,
                                   struct vk_bundle_queue * present_queue,
                                   uint32_t index,
                                   uint64_t timeline_semaphore_value,
                                   int64_t desired_present_time_ns,
                                   int64_t present_slop_ns)
{
	struct ql_comp_target * cn = (struct ql_comp_target *)ct;
	struct vk_bundle * vk = get_vk(cn);

	assert(index < cn->image_count);

	if (cn->c->base.layer_accum.layer_count == 0)
	{
		// TODO: Tell the headset that there is no image to display
		assert(cn->psc.images[index].status == image_acquired);
		cn->psc.images[index].status = image_free;
		return VK_SUCCESS;
	}

	assert(index < ct->image_count);
	assert(ct->images != NULL);

	std::vector<VkCommandBuffer> cmdBuffers;

	// One command buffer per image, re-recorded each present. This allocated a new one every frame and
	// never freed it: on RADV each holds GPU-visible memory, so Monado's GTT grew ~9 MB/s (2026-11-09:
	// 4.5 GB after ~10 min of Half-Life: Alyx, VRAM full, the game sluggish). An encoder that skips a
	// frame frees its image without waiting for the GPU, so wait for this image's last submit first
	// (normally long done).
	auto & img = cn->psc.images[index];
	VkResult res;
	if (img.cmd == VK_NULL_HANDLE)
	{
		res = vk_cmd_create_cmd_buffer_locked(vk, cn->cmd_pool, &img.cmd);
		vk_check_error("vk_cmd_create_cmd_buffer_locked", res, res);
	}
	else if (img.submitted)
	{
		res = vk->vkWaitForFences(vk->device, 1, &img.fence, VK_TRUE, UINT64_MAX);
		vk_check_error("vkWaitForFences", res, res);
	}
	// The pool has RESET_COMMAND_BUFFER_BIT: vkBeginCommandBuffer resets the old recording.
	VkCommandBuffer cmdBuffer = img.cmd;
	VkCommandBufferBeginInfo begin_info{};
	begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	res = vk->vkBeginCommandBuffer(cmdBuffer, &begin_info);
	vk_check_error("vkBeginCommandBuffer", res, res);

	VkImageSubresourceRange first_color_level_subresource_range{};
	first_color_level_subresource_range.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	first_color_level_subresource_range.baseMipLevel = 0;
	first_color_level_subresource_range.levelCount = 1;
	first_color_level_subresource_range.baseArrayLayer = 0;
	first_color_level_subresource_range.layerCount = 1;
	vk_cmd_image_barrier_gpu_locked(vk, cmdBuffer, ct->images[index].handle, 0, VK_ACCESS_SHADER_READ_BIT, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, VK_IMAGE_LAYOUT_GENERAL, first_color_level_subresource_range);

	res = vk->vkEndCommandBuffer(cmdBuffer);
	vk_check_error("vkEndCommandBuffer", res, res);
	cmdBuffers.push_back(cmdBuffer);

	//if (cn->host)
	//	cn->host->flush_stream(cn->host);

	for (auto & encoder: cn->encoders)
	{
		VkCommandBuffer cmdBuffer = VK_NULL_HANDLE;
		encoder->PresentImage(index, &cmdBuffer);
		if (cmdBuffer != VK_NULL_HANDLE)
			cmdBuffers.push_back(cmdBuffer);
	}

	VkPipelineStageFlags sem_flags = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT;
	VkSubmitInfo submit{};
	submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submit.pNext = NULL;
	submit.waitSemaphoreCount = 1;
	submit.pWaitSemaphores = &cn->semaphores.render_complete;
	submit.pWaitDstStageMask = &sem_flags;
	submit.commandBufferCount = cmdBuffers.size();
	submit.pCommandBuffers = cmdBuffers.data();
	submit.signalSemaphoreCount = 0;
	submit.pSignalSemaphores = NULL;

	std::lock_guard lock(cn->psc.mutex);
	res = vk->vkResetFences(vk->device, 1, &cn->psc.images[index].fence);
	vk_check_error("vkResetFences", res, res);
	res = vk_cmd_submit_locked(vk, vk->graphics_queue, 1, &submit, cn->psc.images[index].fence);
	vk_check_error("vk_cmd_submit_locked", res, res);
	img.submitted = true;

	assert(cn->psc.images[index].status == image_acquired);
	cn->frame_index++;
	// set bits to 1 for index 1..num encoder threads + 1
	cn->psc.images[index].status = (1 << (cn->encoder_threads.size() + 1)) - 2;
	cn->psc.images[index].frame_index = cn->frame_index;

	auto & view_info = cn->psc.images[index].view_info;
	// What this image shows. Poses: upstream's renderer puts the absolute eye poses it rendered for (at
	// the frame's predicted display time) in frame_params, "for remote rendering targets" (the fork read
	// the app's layer pose and a head relation from its own per-frame slot instead).
	// FOV: the one this image was laid out with, read back from the uv_to_tanangle the compute path used
	// (not frame_params.fovs, the driver's current FOV, which can be newer: then the image and its
	// label disagreed, 49-deg content labelled 43 deg, unfusable stereo). follow_distortion_fov below
	// moves the layout to the current FOV for the next frames.
	view_info.display_time = cn->c->frame.rendering.predicted_display_time_ns;
	xrt_fov image_fov[2];
	for (int eye = 0; eye < 2; ++eye)
	{
		image_fov[eye] = fov_from_uv_to_tanangle(cn->c->nr.distortion.uv_to_tanangle[eye]);
		view_info.fov[eye] = xrt_cast(image_fov[eye]);
		view_info.pose[eye] = xrt_cast(cn->c->base.frame_params.poses[eye]);
	}
	follow_distortion_fov(cn, image_fov);
	ql_debug_log_view_info(view_info, "upstream port");
	cn->psc.cv.notify_all();

	return VK_SUCCESS;
}

static void comp_ql_flush(struct comp_target * ct)
{
	(void)ct;
}

// Helper function for clamping the time averages
static int64_t comp_ql_clamp_delta_ts(int64_t ts, int64_t min_ts, int64_t max_ts, int64_t max_ts_sub)
{
	if (ts < min_ts) {
		return min_ts;
	}
	if (ts > max_ts) {
		return max_ts_sub;
	}
	return ts;
}

static void comp_ql_calc_frame_pacing(struct comp_target * ct,
                                         int64_t * out_frame_id,
                                         int64_t * out_wake_up_time_ns,
                                         int64_t * out_desired_present_time_ns,
                                         int64_t * out_present_slop_ns,
                                         int64_t * out_predicted_display_time_ns)
{
	struct ql_comp_target* cn = (struct ql_comp_target *)ct;

	//
	// Weighted slightly heavier towards larger amounts, so if the user is rotating 
	// their head a lot, we will err towards longer encodes/transmits
	//

	// For encoding, we take the longest start-end difference
	int64_t avg_encode = cn->last_avg_enc;
	for (int i = 0; i < QL_SWAPCHAIN_DEPTH; i++)
	{
		int64_t largest_enc_diff = 0;
		for (int j = 0; j < QL_NUM_SLICES; j++)
		{
			int64_t enc_diff = cn->host->encode_duration_ns[QL_IDX_SLICE(j,i)];//cn->host->encode_done_ns[i] - cn->host->encode_started_ns[i];
			if (enc_diff > largest_enc_diff) {
				largest_enc_diff = enc_diff;
			}
		}
		avg_encode += largest_enc_diff;
		avg_encode /= 2;
	}

	// For transmission, we sum all of the transmission times across slices
	int64_t avg_tx = cn->last_avg_tx;
	for (int i = 0; i < QL_SWAPCHAIN_DEPTH; i++)
	{
		int64_t full_tx_time = 0;
		for (int j = 0; j < QL_NUM_SLICES; j++)
		{
			int64_t tx_diff = cn->host->tx_duration_ns[QL_IDX_SLICE(j,i)];
			full_tx_time += tx_diff;
		}
		avg_tx += full_tx_time;
		avg_tx /= 2;
	}

	// If tx or enc takes longer than 10ms, weight lower so that
	// it hopefully recovers to a normal value quicker?
	avg_tx = comp_ql_clamp_delta_ts(avg_tx, 0, U_TIME_1MS_IN_NS * 10, 5 * U_TIME_1MS_IN_NS);
	avg_encode = comp_ql_clamp_delta_ts(avg_encode, 0, U_TIME_1MS_IN_NS * 10, 5 * U_TIME_1MS_IN_NS);

	cn->last_avg_tx = avg_tx;
	cn->last_avg_enc = avg_encode;
	
	//static int64_t add_test = 0;
	static int limit_info_spam = 0;
	if (++limit_info_spam > 100) {
		QUEST_LINK_INFO("Avg: tx %fms, encode %fms, add %fms", (double)avg_tx / 1000000.0, (double)avg_encode / 1000000.0, (double)cn->host->add_test / 1000000.0);
		limit_info_spam = 0;

		//add_test += (1 * U_TIME_HALF_MS_IN_NS);
	}

	int64_t encode_display_delay = avg_encode + avg_tx + cn->host->add_test;// + (1 * U_TIME_HALF_MS_IN_NS) + (5 * U_TIME_1MS_IN_NS);

	int64_t frame_id = ++cn->current_frame_id; //-1;
	int64_t now_ns = os_monotonic_get_ns();
	int64_t desired_present_time_ns = now_ns + (U_TIME_1S_IN_NS / (cn->fps));
	int64_t wake_up_time_ns = desired_present_time_ns - 40 * U_TIME_1MS_IN_NS - encode_display_delay;
	int64_t present_slop_ns = encode_display_delay;//U_TIME_HALF_MS_IN_NS;
	int64_t predicted_display_time_ns = desired_present_time_ns + encode_display_delay;

	int64_t predicted_display_period_ns = U_TIME_1S_IN_NS / (cn->fps);// + encode_display_delay;
	int64_t min_display_period_ns = predicted_display_period_ns;
	
	//u_pc_update_present_offset(cn->upc, frame_id, encode_display_delay);

	u_pc_predict(cn->upc,                      //
	             now_ns,                       //
	             &frame_id,                    //
	             &wake_up_time_ns,             //
	             &desired_present_time_ns,     //
	             &present_slop_ns,             //
	             &predicted_display_time_ns,   //
	             &predicted_display_period_ns, //
	             &min_display_period_ns);      //

	cn->current_frame_id = frame_id;

	*out_frame_id = frame_id;
	*out_wake_up_time_ns = wake_up_time_ns;
	*out_desired_present_time_ns = desired_present_time_ns;
	*out_predicted_display_time_ns = predicted_display_time_ns;
	*out_present_slop_ns = present_slop_ns;


}

static void comp_ql_mark_timing_point(struct comp_target * ct,
                                         enum comp_target_timing_point point,
                                         int64_t frame_id,
                                         int64_t when_ns)
{
	struct ql_comp_target * cn = (struct ql_comp_target *)ct;
	assert(frame_id == cn->current_frame_id);

	switch (point)
	{
		case COMP_TARGET_TIMING_POINT_WAKE_UP:
			u_pc_mark_point(cn->upc, U_TIMING_POINT_WAKE_UP, cn->current_frame_id, when_ns);
			break;
		case COMP_TARGET_TIMING_POINT_BEGIN:
			u_pc_mark_point(cn->upc, U_TIMING_POINT_BEGIN, cn->current_frame_id, when_ns);
			break;
		case COMP_TARGET_TIMING_POINT_SUBMIT_BEGIN:
			u_pc_mark_point(cn->upc, U_TIMING_POINT_SUBMIT_BEGIN, cn->current_frame_id, when_ns);
			break;
		case COMP_TARGET_TIMING_POINT_SUBMIT_END:
			u_pc_mark_point(cn->upc, U_TIMING_POINT_SUBMIT_END, cn->current_frame_id, when_ns);
			break;
		default:
			assert(false);
	}
}

static VkResult comp_ql_update_timings(struct comp_target * ct)
{
	// TODO
	return VK_SUCCESS;
}

static void comp_ql_set_title(struct comp_target * ct, const char * title)
{}

static void comp_ql_destroy(struct comp_target * ct)
{
	struct ql_comp_target * cn = (struct ql_comp_target *)ct;

	vk_bundle * vk = get_vk(cn);
	vk->vkDeviceWaitIdle(vk->device);
	destroy_images(cn);

	delete cn;
}

static void comp_ql_info_gpu(struct comp_target * ct, int64_t frame_id, int64_t gpu_start_ns, int64_t gpu_end_ns, int64_t when_ns)
{
	COMP_TRACE_MARKER();

	struct ql_comp_target * cn = (struct ql_comp_target *)ct;

	u_pc_info_gpu(cn->upc, frame_id, gpu_start_ns, gpu_end_ns, when_ns);
}

/*
 * Callbacks upstream's comp_target added since 2023. The compositor calls the first three without
 * checking for NULL: the first upstream run with this target segfaulted right after startup.
 * We don't present through a WSI swapchain, so like upstream's debug-image target: no shared
 * presentable images, no present-wait, and any queue is fine.
 */

static bool comp_ql_is_shared_presentable_image(struct comp_target * ct)
{
	return false;
}

static VkResult comp_ql_wait_for_present(struct comp_target * ct, time_duration_ns timeout_ns)
{
	return VK_ERROR_EXTENSION_NOT_PRESENT; // wait_for_present_supported is false
}

static VkResult comp_ql_queue_supports_present(struct comp_target * ct, struct vk_bundle_queue * queue, VkBool32 * out_supported)
{
	*out_supported = VK_TRUE;
	return VK_SUCCESS;
}

// The refresh rate is the one the headset asked for when Link was accepted (QL_OVERRIDE_FPS).
static xrt_result_t comp_ql_get_refresh_rates(struct comp_target * ct, uint32_t * out_count, float * out_display_refresh_rates_hz)
{
	struct ql_comp_target * cn = (struct ql_comp_target *)ct;
	*out_count = 1;
	out_display_refresh_rates_hz[0] = cn->fps;
	return XRT_SUCCESS;
}

static xrt_result_t comp_ql_get_current_refresh_rate(struct comp_target * ct, float * out_display_refresh_rate_hz)
{
	struct ql_comp_target * cn = (struct ql_comp_target *)ct;
	*out_display_refresh_rate_hz = cn->fps;
	return XRT_SUCCESS;
}

static xrt_result_t comp_ql_request_refresh_rate(struct comp_target * ct, float display_refresh_rate_hz)
{
	return XRT_ERROR_COMPOSITOR_NOT_SUPPORTED; // fixed by the Link session
}

/*
 *
 * 'Exported' functions.
 *
 */

extern "C" comp_target * comp_target_ql_create(struct ql_xrsp_host* host, float fps)
{
	ql_comp_target * cn = new ql_comp_target{};

	cn->host = host;
	//cn->cnx = NULL;
	cn->check_ready = comp_ql_check_ready;
	cn->create_images = comp_ql_create_images;
	cn->has_images = comp_ql_has_images;
	cn->acquire = comp_ql_acquire;
	cn->present = comp_ql_present;
	cn->calc_frame_pacing = comp_ql_calc_frame_pacing;
	cn->mark_timing_point = comp_ql_mark_timing_point;
	cn->update_timings = comp_ql_update_timings;
	cn->info_gpu = comp_ql_info_gpu;
	cn->destroy = comp_ql_destroy;
	cn->init_pre_vulkan = comp_ql_init_pre_vulkan;
	cn->init_post_vulkan = comp_ql_init_post_vulkan;
	cn->set_title = comp_ql_set_title;
	cn->flush = comp_ql_flush;
	cn->is_shared_presentable_image = comp_ql_is_shared_presentable_image;
	cn->wait_for_present = comp_ql_wait_for_present;
	cn->wait_for_present_supported = false;
	cn->queue_supports_present = comp_ql_queue_supports_present;
	cn->get_refresh_rates = comp_ql_get_refresh_rates;
	cn->get_current_refresh_rate = comp_ql_get_current_refresh_rate;
	cn->request_refresh_rate = comp_ql_request_refresh_rate;
	cn->fps = fps;

	return cn;
}
}