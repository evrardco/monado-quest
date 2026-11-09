// Copyright 2026, monadolink project
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  quest_link XRSP audio: PC audio to the headset, probe of the headset's packets, test tone
 *
 * Format (found 2026-10-29 with the probe and the tone below): PayloadAudio on TOPIC_AUDIO, 10 ms
 * per packet, dataUnk0 = timestamp in the headset's clock (ns). PC -> headset: 48 kHz stereo
 * float32 (the tone played cleanly). Headset -> PC (microphone): 48 kHz mono int16, 960 bytes per
 * packet, 100 packets/s.
 *
 * PC audio: a capture thread records a PulseAudio/PipeWire device (QL_AUDIO_SOURCE, default
 * "quest_link.monitor": the monitor of the null sink start-monado.sh creates and makes the default
 * output) in 10 ms chunks; the write thread sends them. At most k_max_queued packets wait (a stall
 * or a slow start drops the oldest), so audio latency can't build up. QL_AUDIO=0 disables it.
 *
 * Microphone: the read thread queues the headset's packets and a playback thread writes them into
 * the sink QL_MIC_SINK (default "quest_link_mic_sink"), whose monitor start-monado.sh remaps as the
 * "Quest-Link Microphone" source (the default input while Monado runs). Its own thread so the USB
 * read loop never waits on PulseAudio; at most k_mic_max_queued packets wait (drops the oldest), so
 * the mic's latency can't build up either. QL_MIC=0 disables it.
 *
 * History: Monado's Quest Link driver sends the audio control message (the headset then starts an
 * AudioPlayer that expects "pcm float32" at ~20 ms latency, per its own log) but never any audio
 * data, so all sound stays on the PC. The data message is PayloadAudio { dataUnk0: UInt64,
 * data: Data } on TOPIC_AUDIO; the headset also sends its microphone that way (its AudioRecorder
 * runs with voice detection). Before streaming PC audio, this learns the format:
 * - QL_AUDIO_PROBE=1: log the headset's audio packets every 2 s (rate, sizes, dataUnk0 steps, whether
 *   the data looks like float32 or int16 samples) and hex-dump the first one.
 * - QL_AUDIO_TEST_TONE=1: send a 440 Hz tone as 48 kHz stereo float32 in 10 ms packets
 *   (QL_AUDIO_TEST_CHANNELS, QL_AUDIO_TEST_RATE, QL_AUDIO_TEST_MS override; QL_AUDIO_TEST_UNK0 =
 *   ts (target-clock ns, default) | frames (sample counter) | zero).
 * @ingroup drv_quest_link
 */

#include "ql_xrsp_audio.h"
#include "ql_xrsp.h"
#include "ql_system.h"
#include "ql_xrsp_types.h"

#include "os/os_time.h"

#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include "protos/Audio.capnp.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef QL_HAVE_PULSE
#include <pulse/error.h>
#include <pulse/simple.h>
#endif

// Defined in ql_xrsp.cpp (C++ linkage).
void
xrsp_send_to_topic_capnp_segments(struct ql_xrsp_host *host, uint8_t topic, uint32_t idx,
                                  kj::ArrayPtr<const kj::ArrayPtr<const capnp::word>> &data);

namespace {

int
env_int(const char *name, int fallback)
{
	const char *v = getenv(name);
	return v != nullptr && *v != '\0' ? atoi(v) : fallback;
}

struct Probe
{
	int packets = 0, parse_failures = 0;
	uint64_t payload_bytes = 0, data_bytes = 0;
	uint64_t min_data = UINT64_MAX, max_data = 0;
	uint64_t prev_unk0 = 0;
	bool have_prev = false;
	int64_t unk0_step_min = INT64_MAX, unk0_step_max = INT64_MIN;
	double unk0_step_sum = 0;
	uint64_t floats = 0, floats_in_range = 0;
	float float_peak = 0.f;
	int int16_peak = 0;
	int64_t period_start_ns = 0;
	bool dumped = false;
};
Probe probe;

void
hex_line(const uint8_t *p, size_t n, std::string *out)
{
	char b[4];
	for (size_t i = 0; i < n; i++) {
		snprintf(b, sizeof(b), "%02x", p[i]);
		*out += b;
		if (i % 4 == 3) {
			*out += ' ';
		}
	}
}

} // namespace

namespace {
void
mic_push(const uint8_t *data, size_t n);
bool
mic_enabled();
} // namespace

void
ql_xrsp_handle_audio(struct ql_xrsp_host *host, struct ql_xrsp_topic_pkt *pkt)
{
	static const bool enabled = env_int("QL_AUDIO_PROBE", 0) != 0;
	if (pkt->payload_valid <= 8) {
		return;
	}
	if (mic_enabled()) {
		try {
			size_t num_words = pkt->payload_valid >> 3;
			kj::ArrayPtr<const capnp::word> segments[1] = {kj::arrayPtr((const capnp::word *)pkt->payload, num_words)};
			capnp::SegmentArrayMessageReader message(kj::arrayPtr(segments, 1));
			capnp::Data::Reader data = message.getRoot<PayloadAudio>().getData();
			mic_push(data.begin(), data.size());
		} catch (...) {
			// A malformed packet: the probe below counts these.
		}
	}
	if (!enabled) {
		return;
	}
	const int64_t now = os_monotonic_get_ns();
	if (probe.period_start_ns == 0) {
		probe.period_start_ns = now;
	}
	probe.packets++;
	probe.payload_bytes += pkt->payload_valid;
	if (!probe.dumped) {
		probe.dumped = true;
		std::string hex;
		hex_line(pkt->payload, pkt->payload_valid < 96 ? pkt->payload_valid : 96, &hex);
		QUEST_LINK_INFO("Audio probe: first packet, %d payload bytes: %s", (int)pkt->payload_valid, hex.c_str());
	}

	try {
		size_t num_words = pkt->payload_valid >> 3;
		kj::ArrayPtr<const capnp::word> segments[1] = {kj::arrayPtr((const capnp::word *)pkt->payload, num_words)};
		capnp::SegmentArrayMessageReader message(kj::arrayPtr(segments, 1));
		PayloadAudio::Reader audio = message.getRoot<PayloadAudio>();
		const uint64_t unk0 = audio.getDataUnk0();
		capnp::Data::Reader data = audio.getData();
		const size_t n = data.size();
		probe.data_bytes += n;
		probe.min_data = n < probe.min_data ? n : probe.min_data;
		probe.max_data = n > probe.max_data ? n : probe.max_data;
		if (probe.have_prev) {
			const int64_t step = (int64_t)(unk0 - probe.prev_unk0);
			probe.unk0_step_min = step < probe.unk0_step_min ? step : probe.unk0_step_min;
			probe.unk0_step_max = step > probe.unk0_step_max ? step : probe.unk0_step_max;
			probe.unk0_step_sum += (double)step;
		}
		probe.have_prev = true;
		probe.prev_unk0 = unk0;
		// The same bytes seen as float32 and as int16 samples: real float32 audio stays within ±1.
		for (size_t i = 0; i + 4 <= n; i += 4) {
			float f;
			memcpy(&f, data.begin() + i, 4);
			probe.floats++;
			if (std::isfinite(f) && std::fabs(f) <= 1.5f) {
				probe.floats_in_range++;
				probe.float_peak = std::fmax(probe.float_peak, std::fabs(f));
			}
		}
		for (size_t i = 0; i + 2 <= n; i += 2) {
			int16_t v;
			memcpy(&v, data.begin() + i, 2);
			probe.int16_peak = std::abs((int)v) > probe.int16_peak ? std::abs((int)v) : probe.int16_peak;
		}
	} catch (...) {
		probe.parse_failures++;
	}

	if (now - probe.period_start_ns >= 2000000000LL) {
		const double secs = (now - probe.period_start_ns) / 1e9;
		const int steps = probe.packets - 1 - probe.parse_failures;
		QUEST_LINK_INFO("Audio probe: %.0f packets/s, %.0f data bytes/s (%llu..%llu per packet), unk0 step %.0f "
		                "(%lld..%lld), float32 in range %.0f%% (peak %.3f), int16 peak %d, %d parse failures",
		                probe.packets / secs, probe.data_bytes / secs, (unsigned long long)probe.min_data,
		                (unsigned long long)probe.max_data, steps > 0 ? probe.unk0_step_sum / steps : 0.0,
		                (long long)probe.unk0_step_min, (long long)probe.unk0_step_max,
		                probe.floats ? 100.0 * probe.floats_in_range / probe.floats : 0.0, probe.float_peak,
		                probe.int16_peak, probe.parse_failures);
		const bool dumped = probe.dumped;
		const uint64_t prev = probe.prev_unk0;
		probe = Probe{};
		probe.dumped = dumped;
		probe.have_prev = true;
		probe.prev_unk0 = prev;
		probe.period_start_ns = now;
	}
}

namespace {

constexpr int k_rate = 48000, k_channels = 2, k_packet_ms = 10;
constexpr int k_packet_frames = k_rate * k_packet_ms / 1000;
constexpr size_t k_max_queued = 6; //!< 60 ms

struct Capture
{
	std::thread thread;
	std::atomic<bool> stop{false};
	std::mutex mutex;
	std::deque<std::vector<float>> queue;
	uint64_t dropped = 0;

	~Capture()
	{
		// Also at exit if ql_xrsp_audio_stop() wasn't reached: a joinable std::thread would abort.
		stop = true;
		if (thread.joinable()) {
			thread.join();
		}
	}

	void
	run()
	{
#ifdef QL_HAVE_PULSE
		const char *env = getenv("QL_AUDIO_SOURCE");
		const std::string source = env != nullptr && *env != '\0' ? env : "quest_link.monitor";
		pa_sample_spec spec{};
		spec.format = PA_SAMPLE_FLOAT32LE;
		spec.rate = k_rate;
		spec.channels = k_channels;
		pa_buffer_attr attr{};
		attr.maxlength = (uint32_t)-1;
		attr.tlength = attr.prebuf = attr.minreq = (uint32_t)-1;
		attr.fragsize = (uint32_t)(k_packet_frames * k_channels * sizeof(float)); // low latency
		bool logged_failure = false;
		while (!stop) {
			int err = 0;
			pa_simple *s = pa_simple_new(nullptr, "Monado Quest Link", PA_STREAM_RECORD, source.c_str(),
			                             "audio to the headset", &spec, nullptr, &attr, &err);
			if (s == nullptr) {
				if (!logged_failure) {
					logged_failure = true;
					QUEST_LINK_WARN("Audio: can't record '%s' (%s): no sound in the headset until it exists "
					                "(start-monado.sh creates it), retrying",
					                source.c_str(), pa_strerror(err));
				}
				for (int i = 0; i < 20 && !stop; i++) {
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
				}
				continue;
			}
			logged_failure = false;
			QUEST_LINK_INFO("Audio: streaming '%s' to the headset (48 kHz stereo float32)", source.c_str());
			std::vector<float> buf((size_t)k_packet_frames * k_channels);
			while (!stop) {
				if (pa_simple_read(s, buf.data(), buf.size() * sizeof(float), &err) < 0) {
					QUEST_LINK_WARN("Audio: recording '%s' failed (%s), reconnecting", source.c_str(), pa_strerror(err));
					break;
				}
				std::lock_guard<std::mutex> lock(mutex);
				queue.push_back(buf);
				while (queue.size() > k_max_queued) {
					queue.pop_front();
					dropped++;
				}
			}
			pa_simple_free(s);
		}
#endif
	}
};
Capture capture;
std::once_flag capture_started;

constexpr int k_mic_rate = 48000; //!< the headset's microphone: mono int16, 10 ms packets
constexpr size_t k_mic_max_queued = 8; //!< 80 ms

struct Mic
{
	std::thread thread;
	std::atomic<bool> stop{false};
	std::mutex mutex;
	std::condition_variable cv;
	std::deque<std::vector<int16_t>> queue;
	uint64_t packets = 0, dropped = 0;

	~Mic()
	{
		stop = true;
		cv.notify_all();
		if (thread.joinable()) {
			thread.join();
		}
	}

	void
	run()
	{
#ifdef QL_HAVE_PULSE
		const char *env = getenv("QL_MIC_SINK");
		const std::string sink = env != nullptr && *env != '\0' ? env : "quest_link_mic_sink";
		pa_sample_spec spec{};
		spec.format = PA_SAMPLE_S16LE;
		spec.rate = k_mic_rate;
		spec.channels = 1;
		// A short playback buffer (30 ms) that starts after one packet: a voice chat wants the mic now.
		const uint32_t packet_bytes = (uint32_t)(k_mic_rate / 100 * sizeof(int16_t));
		pa_buffer_attr attr{};
		attr.maxlength = (uint32_t)-1;
		attr.tlength = 3 * packet_bytes;
		attr.prebuf = packet_bytes;
		attr.minreq = (uint32_t)-1;
		attr.fragsize = (uint32_t)-1;
		bool logged_failure = false;
		while (!stop) {
			int err = 0;
			pa_simple *s = pa_simple_new(nullptr, "Monado Quest Link", PA_STREAM_PLAYBACK, sink.c_str(),
			                             "headset microphone", &spec, nullptr, &attr, &err);
			if (s == nullptr) {
				if (!logged_failure) {
					logged_failure = true;
					QUEST_LINK_WARN("Microphone: can't play into '%s' (%s): the headset's mic doesn't reach the PC "
					                "until it exists (start-monado.sh creates it), retrying",
					                sink.c_str(), pa_strerror(err));
				}
				for (int i = 0; i < 20 && !stop; i++) {
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
				}
				continue;
			}
			logged_failure = false;
			QUEST_LINK_INFO("Microphone: the headset's mic plays into '%s' (48 kHz mono int16)", sink.c_str());
			while (!stop) {
				std::vector<int16_t> samples;
				{
					std::unique_lock<std::mutex> lock(mutex);
					cv.wait_for(lock, std::chrono::milliseconds(100), [this] { return stop || !queue.empty(); });
					if (queue.empty()) {
						continue;
					}
					samples = std::move(queue.front());
					queue.pop_front();
				}
				if (pa_simple_write(s, samples.data(), samples.size() * sizeof(int16_t), &err) < 0) {
					QUEST_LINK_WARN("Microphone: playing into '%s' failed (%s), reconnecting", sink.c_str(),
					                pa_strerror(err));
					break;
				}
			}
			pa_simple_free(s);
		}
#endif
	}
};
Mic mic;
std::once_flag mic_started;

bool
mic_enabled()
{
#ifdef QL_HAVE_PULSE
	static const bool on = env_int("QL_MIC", 1) != 0;
	return on;
#else
	return false;
#endif
}

//! Read thread: queue one packet of the headset's microphone for the playback thread.
void
mic_push(const uint8_t *data, size_t n)
{
	if (n < 2) {
		return;
	}
	std::call_once(mic_started, [] { mic.thread = std::thread([] { mic.run(); }); });
	std::vector<int16_t> samples(n / 2);
	memcpy(samples.data(), data, samples.size() * sizeof(int16_t));
	{
		std::lock_guard<std::mutex> lock(mic.mutex);
		mic.queue.push_back(std::move(samples));
		mic.packets++;
		while (mic.queue.size() > k_mic_max_queued) {
			mic.queue.pop_front(); // PulseAudio fell behind (or the mic clock runs fast): stay current
			mic.dropped++;
		}
	}
	mic.cv.notify_one();
}

bool
audio_enabled()
{
#ifdef QL_HAVE_PULSE
	static const bool on = env_int("QL_AUDIO", 1) != 0 && env_int("QL_AUDIO_TEST_TONE", 0) == 0;
	return on;
#else
	return false;
#endif
}

//! Sends PC audio captured since the last tick (see the file comment).
void
stream_capture(struct ql_xrsp_host *host)
{
	static int64_t stream_ns = 0; //!< timestamp of the next packet (steady, 10 ms apart)
	std::call_once(capture_started, [] { capture.thread = std::thread([] { capture.run(); }); });

	std::vector<std::vector<float>> packets;
	{
		std::lock_guard<std::mutex> lock(capture.mutex);
		if (host->pairing_state != PAIRINGSTATE_PAIRED || !host->ready_to_send_frames) {
			capture.queue.clear(); // not streaming: start fresh, without stale audio, later
			stream_ns = 0;
			return;
		}
		while (!capture.queue.empty()) {
			packets.push_back(std::move(capture.queue.front()));
			capture.queue.pop_front();
		}
	}
	const int64_t now = os_monotonic_get_ns();
	const int64_t period_ns = (int64_t)k_packet_ms * 1000000;
	for (const auto &samples : packets) {
		if (stream_ns == 0 || stream_ns < now - 5 * period_ns || stream_ns > now + 5 * period_ns) {
			stream_ns = now; // first packet, or drifted (capture clock vs ours): resynchronise
		}
		::capnp::MallocMessageBuilder message;
		PayloadAudio::Builder msg = message.initRoot<PayloadAudio>();
		msg.setDataUnk0((uint64_t)xrsp_ts_ns_to_target(host, stream_ns));
		msg.setData(kj::arrayPtr((const kj::byte *)samples.data(), samples.size() * sizeof(float)));
		kj::ArrayPtr<const kj::ArrayPtr<const capnp::word>> out = message.getSegmentsForOutput();
		xrsp_send_to_topic_capnp_segments(host, TOPIC_AUDIO, 0, out);
		stream_ns += period_ns;
	}
}

} // namespace

void
ql_xrsp_audio_stop(void)
{
	capture.stop = true;
	if (capture.thread.joinable()) {
		capture.thread.join();
	}
	mic.stop = true;
	mic.cv.notify_all();
	if (mic.thread.joinable()) {
		mic.thread.join();
	}
}

void
ql_xrsp_audio_tick(struct ql_xrsp_host *host)
{
	if (audio_enabled()) {
		stream_capture(host);
		return;
	}
	static const bool enabled = env_int("QL_AUDIO_TEST_TONE", 0) != 0;
	static const int channels = env_int("QL_AUDIO_TEST_CHANNELS", 2);
	static const int rate = env_int("QL_AUDIO_TEST_RATE", 48000);
	static const int packet_ms = env_int("QL_AUDIO_TEST_MS", 10);
	static const std::string unk0_mode = getenv("QL_AUDIO_TEST_UNK0") ? getenv("QL_AUDIO_TEST_UNK0") : "ts";
	static int64_t next_ns = 0;
	static uint64_t frames_sent = 0;
	static double phase = 0.0;
	static bool logged = false;

	if (!enabled || host->pairing_state != PAIRINGSTATE_PAIRED || !host->ready_to_send_frames) {
		next_ns = 0;
		return;
	}
	const int64_t now = os_monotonic_get_ns();
	const int64_t period_ns = (int64_t)packet_ms * 1000000;
	if (next_ns == 0) {
		next_ns = now;
	}
	if (!logged) {
		logged = true;
		QUEST_LINK_INFO("Audio test tone: 440 Hz, %d Hz, %d channel(s), float32, %d ms packets, unk0 = %s", rate,
		                channels, packet_ms, unk0_mode.c_str());
	}
	const int frames = rate * packet_ms / 1000;
	std::vector<float> samples((size_t)frames * channels);
	for (int sent = 0; now >= next_ns && sent < 5; sent++) { // catch up, but not in a burst
		for (int f = 0; f < frames; f++) {
			const float v = 0.2f * (float)std::sin(phase);
			phase += 2.0 * M_PI * 440.0 / rate;
			for (int c = 0; c < channels; c++) {
				samples[(size_t)f * channels + c] = v;
			}
		}
		phase = std::fmod(phase, 2.0 * M_PI);

		::capnp::MallocMessageBuilder message;
		PayloadAudio::Builder msg = message.initRoot<PayloadAudio>();
		uint64_t unk0 = 0;
		if (unk0_mode == "ts") {
			unk0 = (uint64_t)xrsp_ts_ns_to_target(host, next_ns);
		} else if (unk0_mode == "frames") {
			unk0 = frames_sent;
		}
		msg.setDataUnk0(unk0);
		msg.setData(kj::arrayPtr((const kj::byte *)samples.data(), samples.size() * sizeof(float)));
		kj::ArrayPtr<const kj::ArrayPtr<const capnp::word>> out = message.getSegmentsForOutput();
		xrsp_send_to_topic_capnp_segments(host, TOPIC_AUDIO, 0, out);
		frames_sent += frames;
		next_ns += period_ns;
	}
	if (now - next_ns > 10 * period_ns) {
		next_ns = now; // fell far behind (a stall): don't try to catch up
	}
}
