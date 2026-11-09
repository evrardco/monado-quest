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

#include "VideoEncoderFFMPEG.h"
#include "util/u_logging.h"
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>

extern "C"
{
#include <libavcodec/avcodec.h>
}

namespace
{

enum class nal_kind
{
	csd,   // parameter sets: SPS/PPS (+VPS for HEVC)
	slice, // coded picture data
	drop,  // AUD, SEI, ...
};

nal_kind classify_nal(const uint8_t * nal_header, VideoEncoderFFMPEG::Codec codec)
{
	if (codec == VideoEncoderFFMPEG::Codec::h264)
	{
		switch (nal_header[0] & 0x1F)
		{
			case 7: // SPS
			case 8: // PPS
				return nal_kind::csd;
			case 1: // non-IDR slice
			case 5: // IDR slice
				return nal_kind::slice;
			default:
				return nal_kind::drop;
		}
	}

	uint8_t nal_type = (nal_header[0] >> 1) & 0x3F;
	if (nal_type >= 32 and nal_type <= 34) // VPS, SPS, PPS
		return nal_kind::csd;
	if (nal_type <= 31) // VCL
		return nal_kind::slice;
	return nal_kind::drop;
}

} // namespace

// Split an Annex-B packet into NAL units (start codes included, as x264 emits them) and
// route them like VideoEncoderX264::ProcessCb: parameter sets via SendCSD, slices via SendIDR.
void VideoEncoderFFMPEG::SendNALs(const uint8_t * data, size_t size, int index)
{
	// QL_DUMP_STREAM=<file>: append the raw Annex-B stream (inspectable with ffprobe/ffplay).
	static FILE * dump = []() -> FILE * {
		const char * path = getenv("QL_DUMP_STREAM");
		return path ? fopen(path, "wb") : nullptr;
	}();
	if (dump)
	{
		fwrite(data, 1, size, dump);
		fflush(dump);
	}

	static const std::array<uint8_t, 3> start_code = {{0, 0, 1}};
	const uint8_t * end = data + size;
	const uint8_t * nal_start = std::search(data, end, start_code.begin(), start_code.end());
	if (nal_start != data and nal_start[-1] == 0)
		nal_start--; // 4-byte start code

	while (nal_start != end)
	{
		const uint8_t * payload = nal_start + (nal_start[2] == 0 ? 4 : 3);
		if (payload >= end)
			break;
		const uint8_t * next = std::search(payload, end, start_code.begin(), start_code.end());
		if (next != end and next[-1] == 0)
			next--;

		switch (classify_nal(payload, codec))
		{
			case nal_kind::csd: SendCSD({nal_start, next}, index); break;
			case nal_kind::slice: SendIDR({nal_start, next}, index); break;
			case nal_kind::drop: break;
		}
		nal_start = next;
	}
}

void VideoEncoderFFMPEG::Encode(int index, bool idr, std::chrono::steady_clock::time_point target_timestamp)
{
	int64_t pts = target_timestamp.time_since_epoch().count();
	{
		std::lock_guard lock(pts_mutex);
		frame_index_by_pts[pts] = index;
	}
	using clk = std::chrono::steady_clock;
	auto ms = [](clk::duration d) { return std::chrono::duration<double, std::milli>(d).count(); };
	auto t_start = clk::now();
	PushFrame(index, idr, target_timestamp);
	auto t_pushed = clk::now();

	while (true)
	{
		AVPacket * enc_pkt = av_packet_alloc();
		auto t_recv_start = clk::now();
		int err = avcodec_receive_packet(encoder_ctx.get(), enc_pkt);
		auto t_received = clk::now();
		if (err == AVERROR(EAGAIN))
		{
			av_packet_free(&enc_pkt);
			return;
		}
		if (err)
		{
			av_packet_free(&enc_pkt);
			throw std::runtime_error("frame encoding failed, code " + std::to_string(err));
		}

		// The encoder may hand a packet back a frame late; recover which swapchain index it was.
		int pkt_index = index;
		{
			std::lock_guard lock(pts_mutex);
			auto it = frame_index_by_pts.find(enc_pkt->pts);
			if (it != frame_index_by_pts.end())
				pkt_index = it->second;
			frame_index_by_pts.erase(frame_index_by_pts.begin(), frame_index_by_pts.upper_bound(enc_pkt->pts));
		}

		SendNALs(enc_pkt->data, enc_pkt->size, pkt_index);
		auto t_sent = clk::now();

		// Every 2 s: what the stream costs (for tuning QL_OVERRIDE_BITRATE_KBPS, QL_VAAPI_QMIN,
		// QL_VAAPI_OPTIONS, QL_OVERRIDE_SCALE against bandwidth and encode time).
		{
			stats_bytes += enc_pkt->size;
			stats_max = std::max<int64_t>(stats_max, enc_pkt->size);
			stats_frames++;
			stats_encode_ms += ms(t_sent - t_start);
			auto now = clk::now();
			if (stats_start == clk::time_point{})
				stats_start = now;
			double secs = std::chrono::duration<double>(now - stats_start).count();
			if (secs >= 2.0)
			{
				U_LOG_I("Stream: %.0f fps, %.1f Mbit/s, frames avg %.0f KB max %.0f KB, encode avg %.2f ms",
				        stats_frames / secs, stats_bytes * 8 / secs / 1e6, stats_bytes / 1024.0 / stats_frames,
				        stats_max / 1024.0, stats_encode_ms / stats_frames);
				stats_bytes = stats_max = stats_frames = 0;
				stats_encode_ms = 0;
				stats_start = now;
			}
		}
		FlushFrame(enc_pkt->pts, pkt_index);
		if (ms(t_sent - t_start) > 8.0)
		{
			U_LOG_W("Slow VAAPI frame %d: push %.1fms, receive %.1fms, hand-off (SendNALs) %.1fms, %d bytes",
			        pkt_index, ms(t_pushed - t_start), ms(t_received - t_recv_start), ms(t_sent - t_received), enc_pkt->size);
		}
		av_packet_free(&enc_pkt);
	}
}
