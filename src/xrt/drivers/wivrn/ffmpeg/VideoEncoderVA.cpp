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

#include "VideoEncoderVA.h"
#include "util/u_logging.h"
#include <cstdlib>
#include <string>
#include "ffmpeg_helper.h"
#include "quest_link/ql_vk_compat.h" // vk/vk_helpers.h with the 2023 helper names
#include <cassert>
#include <chrono>
#include <stdexcept>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavfilter/avfilter.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavutil/hwcontext.h>
#include <libavutil/hwcontext_drm.h>
//#include <libavutil/hwcontext_vaapi.h>
#include <libavutil/opt.h>
#ifdef QL_HAVE_LIBVA
#include <libavutil/hwcontext_vaapi.h>
#include <va/va.h>
#endif
}

// Normally set by CMake (QL_VAAPI_GOP_DEFAULT cache variable); see the keyframe interval comment below.
#ifndef QL_VAAPI_GOP_DEFAULT
#define QL_VAAPI_GOP_DEFAULT 432000
#endif

namespace
{

const char *
encoder(VideoEncoderFFMPEG::Codec codec)
{
	switch (codec)
	{
		case VideoEncoderFFMPEG::Codec::h264:
			return "h264_vaapi";
		case VideoEncoderFFMPEG::Codec::h265:
			return "hevc_vaapi";
	}
	throw std::runtime_error("invalid codec " + std::to_string(int(codec)));
}

// QL_VAAPI_10BIT=1: HEVC Main10 (P010 surfaces) instead of Main (NV12). An experiment: Meta's own
// Link host only ever encodes 8-bit NV12 (its RemoteHeadset.dll converts RGBA->NV12, no P010 or
// Main10 anywhere), and the handshake has no known bit-depth field; HEVC carries its profile in the
// stream, so the Quest's decoder may still take it. 10 bits keep dark gradients from banding.
// Tried 2026-10-29 (Quest 2, current firmware): no picture, the headset kept re-creating its video
// receiver. Kept as a switch for future firmware.
static bool
want_10bit()
{
	static const bool on = getenv("QL_VAAPI_10BIT") != nullptr && atoi(getenv("QL_VAAPI_10BIT")) != 0;
	return on;
}

void set_hwframe_ctx(AVCodecContext * ctx, AVBufferRef * hw_device_ctx)
{
	av_buffer_ptr hw_frames_ref(av_hwframe_ctx_alloc(hw_device_ctx));
	int err = 0;

	if (not hw_frames_ref)
	{
		throw std::runtime_error("Failed to create VAAPI frame context.");
	}
	auto frames_ctx = (AVHWFramesContext *)(hw_frames_ref->data);
	frames_ctx->format = AV_PIX_FMT_VAAPI;
	frames_ctx->sw_format = ctx->profile == AV_PROFILE_HEVC_MAIN_10 ? AV_PIX_FMT_P010 : AV_PIX_FMT_NV12;
	frames_ctx->width = ctx->width;
	frames_ctx->height = ctx->height;
	frames_ctx->initial_pool_size = 3;
	if ((err = av_hwframe_ctx_init(hw_frames_ref.get())) < 0)
	{
		throw std::system_error(err, av_error_category(), "Failed to initialize VAAPI frame context");
	}
	ctx->hw_frames_ctx = av_buffer_ref(hw_frames_ref.get());
	if (!ctx->hw_frames_ctx)
		err = AVERROR(ENOMEM);
}

std::string
get_render_device(vk_bundle * vk)
{
	VkPhysicalDeviceDrmPropertiesEXT drmProps{};
	drmProps.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DRM_PROPERTIES_EXT;

	VkPhysicalDeviceProperties2 props{};
	props.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
	props.pNext = &drmProps;

	vk->vkGetPhysicalDeviceProperties2(vk->physical_device, &props);

	if (!drmProps.hasRender)
	{
		U_LOG_E("Failed to find render DRM device");
		throw std::runtime_error("Failed to find render DRM device");
	}
	return "/dev/dri/renderD" + std::to_string(drmProps.renderMinor);
}

} // namespace

VideoEncoderVA::VideoEncoderVA(vk_bundle * vk, const encoder_settings & settings, float fps) :
        vk(vk), width(settings.width), height(settings.height), offset_x(settings.offset_x), offset_y(settings.offset_y)
{
	codec = settings.codec;
	/* VAAPI Encoding pipeline
	 * The encoding pipeline has 3 frame types:
	 * - input vulkan frames, only used to initialize the mapped frames
	 * - mapped frames, one per input frame, same format, and point to the same memory on the device
	 * - encoder frame, with a format compatible with the encoder, created by the filter
	 * Each frame type has a corresponding hardware frame context, the vulkan one is provided
	 *
	 * The pipeline is simply made of a scale_vaapi object, that does the conversion between formats
	 * and the encoder that takes the converted frame and produces packets.
	 */

	AVBufferRef * tmp;
	const std::string device = get_render_device(vk);
	int err = av_hwdevice_ctx_create(&tmp, AV_HWDEVICE_TYPE_VAAPI, device.c_str(), NULL, 0);
	if (err < 0)
	{
		throw std::system_error(err, av_error_category(), "Failed create a VAAPI device");
	}
	hw_ctx_vaapi.reset(tmp);

	const char * encoder_name = encoder(codec);
	const AVCodec * codec = avcodec_find_encoder_by_name(encoder_name);
	if (codec == nullptr)
	{
		throw std::runtime_error(std::string("Failed to find encoder ") + encoder_name);
	}

	encoder_ctx.reset(avcodec_alloc_context3(codec));
	if (not encoder_ctx)
	{
		throw std::runtime_error("failed to allocate VAAPI encoder");
	}

	AVDictionary * opts = nullptr;
	// async_depth=1 avoids a frame of latency but makes avcodec_send_frame wait for the previous
	// frame to fully clear the GPU, which showed up as periodic ~16ms stalls. QL_VAAPI_ASYNC_DEPTH
	// overrides (ffmpeg's default is 2); settings.options can override too.
	const char * async_depth = getenv("QL_VAAPI_ASYNC_DEPTH");
	av_dict_set(&opts, "async_depth", async_depth ? async_depth : "1", 0);
	for (auto option: settings.options)
	{
		av_dict_set(&opts, option.first.c_str(), option.second.c_str(), 0);
	}
	// QL_VAAPI_OPTIONS="key=value:key=value": any of the encoder's ffmpeg options, for experiments
	// without a rebuild, e.g. rc_mode=CQP:qp=18 (constant quality) or quality=0 (slowest, best
	// preset). Applied last, so they override everything above. `ffmpeg -h encoder=hevc_vaapi` lists
	// them.
	if (const char * extra = getenv("QL_VAAPI_OPTIONS"); extra and *extra)
	{
		if (av_dict_parse_string(&opts, extra, "=", ":", 0) < 0)
			U_LOG_W("QL_VAAPI_OPTIONS='%s' couldn't be parsed (key=value:key=value)", extra);
		else
			U_LOG_I("VAAPI extra encoder options: %s", extra);
	}
	switch (this->codec)
	{
		case Codec::h264: {
			// x264 (ultrafast) emits Constrained Baseline, which the Quest decodes reliably; Main/High
			// (CABAC) is unverified there. QL_VAAPI_PROFILE=main|high overrides.
			const char * profile = getenv("QL_VAAPI_PROFILE");
			if (profile and std::string(profile) == "main")
				encoder_ctx->profile = AV_PROFILE_H264_MAIN;
			else if (profile and std::string(profile) == "high")
				encoder_ctx->profile = AV_PROFILE_H264_HIGH;
			else
				encoder_ctx->profile = AV_PROFILE_H264_CONSTRAINED_BASELINE;
			break;
		}
		case Codec::h265:
			encoder_ctx->profile = want_10bit() ? AV_PROFILE_HEVC_MAIN_10 : AV_PROFILE_HEVC_MAIN;
			if (want_10bit())
				U_LOG_W("QL_VAAPI_10BIT: encoding HEVC Main10 (experimental: Meta's Link only sends 8-bit)");
			break;
	}

	encoder_ctx->width = width;
	encoder_ctx->height = height;
	encoder_ctx->time_base = {std::chrono::steady_clock::duration::period::num,
	                          std::chrono::steady_clock::duration::period::den};
	encoder_ctx->framerate = AVRational{(int)fps, 1};
	encoder_ctx->sample_aspect_ratio = AVRational{1, 1};
	encoder_ctx->pix_fmt = AV_PIX_FMT_VAAPI;
	encoder_ctx->max_b_frames = 0;
	encoder_ctx->bit_rate = settings.bitrate;

	// Scene changes: after cheap frames the rate control picks a low QP, so the first frames of a new
	// scene came out at 0.5-2 MB (seen 2026-10-29) where normal frames take 20-40 KB. At ~60 MB/s over
	// USB a 2 MB frame takes 33 ms to send: four frames late, a visible drop. Tested on the RX 6700 XT
	// (radeonsi, hevc_vaapi, 130 Mbit/s, 120 fps, a burst of noise frames):
	// - a minimum QP bounds the burst without touching calm frames: qmin 20 took the burst from
	//   15 MB to 11 MB (worst frame 4.0 -> 2.8 MB) and QP 20 is visually near-lossless.
	//   QL_VAAPI_QMIN overrides (0 = the encoder's default).
	// - a small rate control buffer only works in CBR (VBR ignores it): 3 frames took the burst to
	//   7 MB but CBR pads every calm frame to the average size (0 -> 132 KB), ~2 ms more USB time per
	//   frame. So it's opt-in: QL_VAAPI_VBV_FRAMES=N (buffer of N average frames).
	{
		int qmin = 20;
		if (const char * env = getenv("QL_VAAPI_QMIN"))
			qmin = atoi(env);
		if (qmin > 0)
			encoder_ctx->qmin = qmin;
		int vbv_frames = 0;
		if (const char * env = getenv("QL_VAAPI_VBV_FRAMES"))
			vbv_frames = atoi(env);
		if (vbv_frames > 0 and fps > 0)
		{
			encoder_ctx->rc_buffer_size = (int)(settings.bitrate / fps * vbv_frames);
			encoder_ctx->rc_max_rate = settings.bitrate; // CBR: the only mode that honours the buffer
		}
		U_LOG_I("VAAPI rate control: min QP %d, buffer %s", qmin,
		        vbv_frames > 0 ? (std::to_string(vbv_frames) + " frames (CBR)").c_str() : "default (VBR)");
	}

	// Keyframe (IDR) interval. ffmpeg's VAAPI encoders default to g=120: at 120 Hz one keyframe per
	// second, 5-10x a normal frame (500-930 KB) that took 12-21 ms to encode and 10-16 ms to send over
	// USB, so it missed its 8.3 ms slot every second (visible late frames, "Frame late by 8.33ms").
	// Over USB nothing gets lost, so keyframes only matter when the stream starts (the first frame is
	// always one): default to a very long interval (QL_VAAPI_GOP_DEFAULT, CMake, 432000 = 1 h at
	// 120 Hz). QL_VAAPI_GOP overrides at run time, e.g. 120 for ffmpeg's old behaviour if the headset
	// turns out to need periodic keyframes to recover from glitches. Values <= 1 would make every frame
	// a keyframe (ffmpeg treats gop_size <= 1 as intra-only), so they're refused.
	{
		int gop = QL_VAAPI_GOP_DEFAULT;
		if (const char * env = getenv("QL_VAAPI_GOP"))
		{
			int v = atoi(env);
			if (v > 1)
				gop = v;
			else
				U_LOG_W("QL_VAAPI_GOP=%s ignored (must be > 1), using %d", env, gop);
		}
		encoder_ctx->gop_size = gop;
		U_LOG_I("VAAPI keyframe interval: %d frames", gop);
	}

	set_hwframe_ctx(encoder_ctx.get(), hw_ctx_vaapi.get());

	err = avcodec_open2(encoder_ctx.get(), codec, &opts);
	av_dict_free(&opts);
	if (err < 0)
	{
		throw std::system_error(err, av_error_category(), "Cannot open video encoder codec");
	}
}

void VideoEncoderVA::PushFrame(uint32_t frame_index, bool idr, std::chrono::steady_clock::time_point pts)
{
	assert(frame_index < mapped_frames.size());
	using clk = std::chrono::steady_clock;
	auto t0 = clk::now();
	av_frame_ptr encoder_frame = make_av_frame();
	int err = av_buffersrc_add_frame_flags(filter_in, mapped_frames[frame_index].get(), AV_BUFFERSRC_FLAG_PUSH | AV_BUFFERSRC_FLAG_KEEP_REF);
	if (err != 0)
	{
		throw std::system_error(err, av_error_category(), "av_buffersrc_add_frame failed");
	}
	err = av_buffersink_get_frame(filter_out, encoder_frame.get());
	if (err != 0)
	{
		throw std::system_error(err, av_error_category(), "av_buffersink_get_frame failed");
	}
#ifdef QL_HAVE_LIBVA
	// Wait for scale_vaapi to finish: it reads the compositor's image asynchronously, and once we
	// return the image is free again, so the compositor may render the next frame into it while the
	// scaler still reads it (RADV doesn't make Vulkan writes wait for VAAPI reads). Under GPU load
	// that showed as a white noise rectangle for one frame every so often (2026-10-29, after ~20 min
	// of HL2:VR), encoded as 2-2.5 MB frames. The wait is on the scaler's output surface: done
	// writing means done reading the input.
	{
		auto * device = (AVHWDeviceContext *)hw_ctx_vaapi->data;
		auto * va = (AVVAAPIDeviceContext *)device->hwctx;
		VASurfaceID surface = (VASurfaceID)(uintptr_t)encoder_frame->data[3];
		VAStatus st = vaSyncSurface(va->display, surface);
		if (st != VA_STATUS_SUCCESS)
		{
			static bool logged = false;
			if (!logged)
			{
				logged = true;
				U_LOG_W("vaSyncSurface failed: %s", vaErrorStr(st));
			}
		}
		// The compositor's image has been read: hand it back now rather than after the encode.
		// Holding it for the whole encode (~8.5 ms at 90 Hz, scale 0.95) left the compositor with
		// no free image whenever an encode ran long, so it spun in acquire, missed its slot and
		// SteamVR's frame was dropped (felt when turning, 2026-10-30).
		if (st == VA_STATUS_SUCCESS)
			InputDone();
	}
#endif
	auto t1 = clk::now();

	encoder_frame->pict_type = idr ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_NONE;
	encoder_frame->pts = pts.time_since_epoch().count();
	err = avcodec_send_frame(encoder_ctx.get(), encoder_frame.get());
	if (err)
	{
		throw std::system_error(err, av_error_category(), "avcodec_send_frame failed");
	}
	auto t2 = clk::now();
	auto ms = [](clk::duration d) { return std::chrono::duration<double, std::milli>(d).count(); };
	if (ms(t2 - t0) > 8.0)
		U_LOG_W("Slow PushFrame %u: scale_vaapi %.1fms, avcodec_send_frame %.1fms", frame_index, ms(t1 - t0), ms(t2 - t1));
}

void VideoEncoderVA::SetImages(
        int width,
        int height,
        VkFormat format,
        int num_images,
        VkImage * images,
        VkImageView * views,
        VkDeviceMemory * memory)
{
	mapped_frames.clear();
	av_buffer_ptr va_ctx(av_hwframe_ctx_alloc(hw_ctx_vaapi.get()));
	if (!va_ctx)
	{
		throw std::runtime_error("Failed to create VAAPI frame context.");
	}
	AVHWFramesContext * va_frames_ctx = (AVHWFramesContext *)(va_ctx->data);
	va_frames_ctx->format = AV_PIX_FMT_VAAPI;
	va_frames_ctx->sw_format = vk_format_to_av_format(format);
	va_frames_ctx->width = width;
	va_frames_ctx->height = height;
	va_frames_ctx->initial_pool_size = num_images;
	int err = av_hwframe_ctx_init(va_ctx.get());
	if (err)
	{
		throw std::system_error(err, av_error_category(), "Failed to create VAAPI frame context");
	}

	mapped_frames.reserve(num_images);
	for (int i = 0; i < num_images; ++i)
	{
		VkMemoryRequirements req;
		vk->vkGetImageMemoryRequirements(vk->device, images[i], &req);

		int fd = -1;
		VkMemoryGetFdInfoKHR export_info{};
		export_info.sType = VK_STRUCTURE_TYPE_MEMORY_GET_FD_INFO_KHR;
		export_info.memory = memory[i];
		export_info.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT;
		VkResult fd_res = vk->vkGetMemoryFdKHR(vk->device, &export_info, &fd);
		if (fd_res != VK_SUCCESS or fd < 0)
			throw std::runtime_error("vkGetMemoryFdKHR(DMA_BUF) failed: " + std::string(vk_result_string(fd_res)) +
			                         " (is the image memory allocated as exportable?)");

		auto drm_frame = make_av_frame();
		drm_frame->format = AV_PIX_FMT_DRM_PRIME;
		drm_frame->width = width;
		drm_frame->height = height;
		// Zeroed: fields we don't set (e.g. the modifier for linear images) must not be garbage;
		// newer Mesa honours them and the import fails.
		drm_frame->buf[0] = av_buffer_allocz(sizeof(AVDRMFrameDescriptor));
		drm_frame->data[0] = drm_frame->buf[0]->data;
		auto & drm_frame_desc = *(AVDRMFrameDescriptor *)drm_frame->data[0];
		drm_frame_desc.nb_objects = 1;
		drm_frame_desc.objects[0].fd = fd;
		drm_frame_desc.objects[0].size = req.size;
		drm_frame_desc.nb_layers = 1;
		drm_frame_desc.layers[0].format = vk_format_to_fourcc(format);
		drm_frame_desc.layers[0].nb_planes = 1;
		drm_frame_desc.layers[0].planes[0].object_index = 0;

		// DRM format modifiers seem to be broken on ffmpeg or radeon, or both
		bool drm_tiling = use_drm_format_modifiers() and vk->has_EXT_image_drm_format_modifier;
		if (drm_tiling)
		{
			VkImageDrmFormatModifierPropertiesEXT modifiers{};
			modifiers.sType = VK_STRUCTURE_TYPE_IMAGE_DRM_FORMAT_MODIFIER_PROPERTIES_EXT;
			vk->vkGetImageDrmFormatModifierPropertiesEXT(vk->device, images[i], &modifiers);
			drm_frame_desc.objects[0].format_modifier = modifiers.drmFormatModifier;
		}
		else
		{
			drm_frame_desc.objects[0].format_modifier = 0; // DRM_FORMAT_MOD_LINEAR
		}
		// if we don't have modifiers, we did linear tiling
		VkImageSubresource subresource{(use_drm_format_modifiers() and vk->has_EXT_image_drm_format_modifier)
		                                       ? VK_IMAGE_ASPECT_MEMORY_PLANE_0_BIT_EXT
		                                       : VK_IMAGE_ASPECT_COLOR_BIT,
		                               0,
		                               0};
		VkSubresourceLayout layout;
		vk->vkGetImageSubresourceLayout(vk->device, images[i], &subresource, &layout);
		drm_frame_desc.layers[0].planes[0].offset = layout.offset;
		drm_frame_desc.layers[0].planes[0].pitch = layout.rowPitch;
		if (i == 0)
			U_LOG_I("VAAPI import: %dx%d fourcc 0x%08x, %s modifier 0x%016llx, offset %llu, pitch %llu, size %llu",
			        width, height, drm_frame_desc.layers[0].format, drm_tiling ? "DRM" : "linear (no)",
			        (unsigned long long)drm_frame_desc.objects[0].format_modifier,
			        (unsigned long long)layout.offset, (unsigned long long)layout.rowPitch,
			        (unsigned long long)drm_frame_desc.objects[0].size);

		auto va_frame = make_av_frame();
		err = av_hwframe_get_buffer(va_ctx.get(), va_frame.get(), 0);
		if (err)
			throw std::system_error(err, av_error_category(), "Failed to create VAAPI frame");
		err = av_hwframe_map(va_frame.get(), drm_frame.get(), AV_HWFRAME_MAP_DIRECT);
		if (err)
			throw std::system_error(err, av_error_category(), "Failed to map DRM frame to VAAPI frame");
		va_frame->crop_left = offset_x;
		va_frame->crop_right = width - this->width - offset_x;
		va_frame->crop_top = offset_y;
		va_frame->crop_bottom = height - this->height - offset_y;
		mapped_frames.push_back(std::move(va_frame));
	}

	InitFilterGraph();
}

void VideoEncoderVA::InitFilterGraph()
{
	assert(not mapped_frames.empty());

	filter_graph.reset(avfilter_graph_alloc());

	AVFilterInOut * outputs = avfilter_inout_alloc();
	AVFilterInOut * inputs = avfilter_inout_alloc();

	filter_in = avfilter_graph_alloc_filter(filter_graph.get(), avfilter_get_by_name("buffer"), "in");

	AVBufferSrcParameters * par = av_buffersrc_parameters_alloc();
	memset(par, 0, sizeof(*par));
	par->width = mapped_frames[0]->width;
	par->height = mapped_frames[0]->height;
	par->time_base = encoder_ctx->time_base;
	par->format = mapped_frames[0]->format;
	par->hw_frames_ctx = av_buffer_ref(mapped_frames[0]->hw_frames_ctx);
	av_buffersrc_parameters_set(filter_in, par);
	av_free(par);

	int err;
	// Filters from avfilter_graph_alloc_filter() must be initialised before linking (enforced by newer ffmpeg).
	if ((err = avfilter_init_dict(filter_in, NULL)) < 0)
	{
		throw std::system_error(err, av_error_category(), "filter_in initialisation failed");
	}
	if ((err = avfilter_graph_create_filter(&filter_out, avfilter_get_by_name("buffersink"), "out", NULL, NULL, filter_graph.get())))
	{
		throw std::system_error(err, av_error_category(), "filter_out creation failed");
	}

	outputs->name = av_strdup("in");
	outputs->filter_ctx = filter_in;
	outputs->pad_idx = 0;
	outputs->next = NULL;

	inputs->name = av_strdup("out");
	inputs->filter_ctx = filter_out;
	inputs->pad_idx = 0;
	inputs->next = NULL;

	const char * scale_format = encoder_ctx->profile == AV_PROFILE_HEVC_MAIN_10 ? "p010" : "nv12";
	std::string scale_param = std::string("scale_vaapi=format=") + scale_format + ":w=" + std::to_string(encoder_ctx->width) +
	                          ":h=" + std::to_string(encoder_ctx->height);
	err = avfilter_graph_parse_ptr(filter_graph.get(), scale_param.c_str(), &inputs, &outputs, NULL);
	avfilter_inout_free(&outputs);
	avfilter_inout_free(&inputs);
	if (err < 0)
	{
		throw std::system_error(err, av_error_category(), "avfilter_graph_parse_ptr failed");
	}

	if ((err = avfilter_graph_config(filter_graph.get(), NULL)))
	{
		throw std::system_error(err, av_error_category(), "avfilter_graph_config failed");
	}
}
