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

#include "video_encoder.h"
#include "external/rs.h"
#include "util/u_logging.h"

#include "../quest_link/ql_types.h"

#include <string>

#include "wivrn_config.h"

#ifdef WIVRN_HAVE_CUDA
#include "video_encoder_nvenc.h"
#endif
#ifdef WIVRN_HAVE_FFMPEG
#include "ffmpeg/VideoEncoderVA.h"
#endif
#ifdef WIVRN_HAVE_X264
#include "video_encoder_x264.h"
#endif
#ifdef XRT_HAVE_VT
#include "video_encoder_vt.h"
#endif

#include <stdio.h>

static void hex_dump(const uint8_t* b, size_t amt)
{
    for (size_t i = 0; i < amt; i++)
    {
        if (i && i % 16 == 0) {
            printf("\n");
        }
        printf("%02x ", b[i]);
    }
    printf("\n");
}

namespace xrt::drivers::wivrn
{

std::unique_ptr<VideoEncoder> VideoEncoder::Create(
        vk_bundle * vk,
        encoder_settings & settings,
        uint8_t stream_idx,
        uint8_t slice_idx,
        uint8_t num_slices,
        int input_width,
        int input_height,
        float fps)
{
	using namespace std::string_literals;
	std::unique_ptr<VideoEncoder> res;
#ifdef XRT_HAVE_VT
	if (settings.encoder_name == encoder_vt)
	{
		res = std::make_unique<VideoEncoderVT>(vk, settings, input_width, input_height, slice_idx, num_slices, fps);
	}
#endif
#ifdef WIVRN_HAVE_X264
	if (settings.encoder_name == encoder_x264)
	{
		res = std::make_unique<VideoEncoderX264>(vk, settings, input_width, input_height, slice_idx, num_slices, fps);
	}
#endif
#ifdef WIVRN_HAVE_CUDA
	if (settings.encoder_name == encoder_nvenc)
	{
		res = std::make_unique<VideoEncoderNvenc>(vk, settings, fps);
	}
#endif
#ifdef WIVRN_HAVE_FFMPEG
	if (settings.encoder_name == encoder_vaapi)
	{
		res = std::make_unique<VideoEncoderVA>(vk, settings, fps);
	}
#endif
	if (res)
	{
		res->stream_idx = stream_idx;
		res->slice_idx = slice_idx;
		res->num_slices = num_slices;
	}
	else
	{
		U_LOG_E("No video encoder %s", settings.encoder_name.c_str());
	}
	return res;
}

void VideoEncoder::InputDone()
{
	if (input_done)
	{
		auto done = std::move(input_done);
		input_done = nullptr;
		done();
	}
}

void VideoEncoder::Encode(wivrn_session* cnx,
                          const to_headset::video_stream_data_shard::view_info_t & view_info,
                          uint64_t frame_index,
                          int index,
                          bool idr,
                          std::function<void()> input_done)
{
	this->input_done = std::move(input_done);
	this->cnx = cnx;
	if (this->cnx)
		this->host = nullptr;
	this->frame_idx = frame_index;
	auto target_timestamp = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(view_info.display_time));

	shards.clear();
	try
	{
		Encode(index, idr, target_timestamp);
	}
	catch (...)
	{
		InputDone();
		throw;
	}
	InputDone();
	if (shards.empty())
		return;
	const size_t view_info_size = sizeof(to_headset::video_stream_data_shard::view_info_t);
	if (shards.back().payload.size() + view_info_size > to_headset::video_stream_data_shard::max_payload_size)
	{
		// Push empty data so previous shard is sent and counters are set
		PushShard({}, to_headset::video_stream_data_shard::start_of_slice | to_headset::video_stream_data_shard::end_of_slice);
	}
	shards.back().view_info = view_info;
	shards.back().flags |= to_headset::video_stream_data_shard::end_of_frame;
	//if (host)
	//	host->flush_stream(host);

	// WiVRn computed Reed-Solomon parity shards here for its network stream. Over Quest Link nothing
	// sent them: it only cost time on the encoder thread every frame, so it's gone (with the network
	// serialization it needed).
}

void VideoEncoder::FlushFrame(int64_t target_ns, int index)
{
	if (host)
	{
		host->flush_stream(host, target_ns, index, slice_idx);
		return;
	}
}

void VideoEncoder::SendCSD(std::vector<uint8_t> && data, int index)
{
	if (host)
	{
		host->send_csd(host, data.data(), data.size(), index, slice_idx);
		return;
	}
	auto & max_payload_size = to_headset::video_stream_data_shard::max_payload_size;
	std::lock_guard lock(mutex);

	//hex_dump(data.data(), data.size());

	uint8_t flags = to_headset::video_stream_data_shard::start_of_slice;
	if (data.size() <= max_payload_size)
	{
		PushShard(std::move(data), flags | to_headset::video_stream_data_shard::end_of_slice);
	}
	auto begin = data.begin();
	auto end = data.end();
	while (begin != end)
	{
		auto next = std::min(end, begin + max_payload_size);
		if (next == end)
		{
			flags |= to_headset::video_stream_data_shard::end_of_slice;
		}
		PushShard({begin, next}, flags);
		begin = next;
		flags = 0;
	}
}

void VideoEncoder::SendIDR(std::vector<uint8_t> && data, int index)
{
	if (host)
	{
		host->send_idr(host, data.data(), data.size(), index, slice_idx);
		return;
	}
	auto & max_payload_size = to_headset::video_stream_data_shard::max_payload_size;
	std::lock_guard lock(mutex);

	//hex_dump(data.data(), data.size());

	uint8_t flags = to_headset::video_stream_data_shard::start_of_slice;
	if (data.size() <= max_payload_size)
	{
		PushShard(std::move(data), flags | to_headset::video_stream_data_shard::end_of_slice);
	}
	auto begin = data.begin();
	auto end = data.end();
	while (begin != end)
	{
		auto next = std::min(end, begin + max_payload_size);
		if (next == end)
		{
			flags |= to_headset::video_stream_data_shard::end_of_slice;
		}
		PushShard({begin, next}, flags);
		begin = next;
		flags = 0;
	}
}

void VideoEncoder::PushShard(std::vector<uint8_t> && payload, uint8_t flags)
{
	to_headset::video_stream_data_shard shard;
	shard.stream_item_idx = stream_idx;
	shard.frame_idx = frame_idx;
	shard.shard_idx = shards.size();
	shard.flags = flags;
	shard.payload = std::move(payload);
	shards.emplace_back(std::move(shard));
}

} // namespace xrt::drivers::wivrn
