// Copyright 2022, Collabora, Ltd.
// Copyright 2022 Max Thomas
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to quest_link XRSP protocol.
 * @author Max Thomas <mtinc2@gmail.com>
 * @ingroup drv_quest_link
 */

#include <libusb.h>
#include <stdio.h>

#include "util/u_debug.h"
#include "util/u_trace_marker.h"
#include "math/m_api.h"
#include "math/m_vec3.h"

#include "os/os_time.h"

#include "ql_system.h"
#include "ql_hmd.h"
#include "ql_xrsp.h"
#include "ql_xrsp_hostinfo.h"
#include "ql_xrsp_topic.h"
#include "ql_xrsp_types.h"
#include "ql_utils.h"
#include "ql_xrsp_pose.h"
#include "ql_xrsp_ipc.h"
#include "ql_xrsp_hands.h"
#include "ql_xrsp_logging.h"
#include "ql_xrsp_audio.h"

#include <atomic>
#include <string>
#include <vector>
#include <sys/stat.h>
#include "ql_xrsp_segmented_pkt.h"

#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include "protos/Audio.capnp.h"
#include "protos/HostInfo.capnp.h"
#include "protos/Slice.capnp.h"
#include "protos/Haptic.capnp.h"
#include "protos/RuntimeIPC.capnp.h"
#include "protos/Mesh.capnp.h"

DEBUG_GET_ONCE_NUM_OPTION(force_fps, "QL_OVERRIDE_FPS", -1)
DEBUG_GET_ONCE_NUM_OPTION(force_w, "QL_OVERRIDE_FB_W", -1)
DEBUG_GET_ONCE_NUM_OPTION(force_h, "QL_OVERRIDE_FB_H", -1)
DEBUG_GET_ONCE_FLOAT_OPTION(force_scale, "QL_OVERRIDE_SCALE", 0.0)

static void *
ql_xrsp_read_thread(void *ptr);
/*!
 * Stream reset on request: a fresh keyframe, which clears whatever the headset's decoder got wrong
 * (a smear or noise that persists). Requested by touching a file, which the monadolink SteamVR driver
 * does from an in-headset shortcut and the Quest VR tray from its menu (they share nothing else with
 * Monado). QL_RESET_TRIGGER overrides the path (default ~/.cache/quest-vr/reset-stream; the home
 * folder is visible inside SteamVR's container). Checked every 200 ms from the write thread.
 */
static void xrsp_check_stream_reset(struct ql_xrsp_host *host)
{
    static std::string path;
    static int64_t next_check_ns = 0;
    static struct timespec last = {0, 0};
    static bool have_last = false;
    const int64_t now = os_monotonic_get_ns();
    if (now < next_check_ns) {
        return;
    }
    next_check_ns = now + 200 * U_TIME_1MS_IN_NS;
    if (path.empty()) {
        const char *env = getenv("QL_RESET_TRIGGER");
        const char *home = getenv("HOME");
        path = env != NULL && *env ? env : std::string(home != NULL ? home : "/tmp") + "/.cache/quest-vr/reset-stream";
    }
    struct stat st;
    if (stat(path.c_str(), &st) != 0) {
        have_last = true; // a file created from now on is a request
        return;
    }
    const bool changed = st.st_mtim.tv_sec != last.tv_sec || st.st_mtim.tv_nsec != last.tv_nsec;
    const bool first = !have_last; // the file was already there at startup: an old request
    last = st.st_mtim;
    have_last = true;
    if (!changed || first) {
        return;
    }
    if (host->pairing_state == PAIRINGSTATE_PAIRED && host->ready_to_send_frames) {
        __atomic_add_fetch(&host->idr_generation, 1, __ATOMIC_RELEASE);
        QUEST_LINK_INFO("Stream reset requested (%s): sending a keyframe", path.c_str());
    }
}

static void *
ql_xrsp_write_thread(void *ptr);
static void xrsp_reset_echo(struct ql_xrsp_host *host);
static bool xrsp_read_usb(struct ql_xrsp_host *host);
static void xrsp_send_usb(struct ql_xrsp_host *host, const uint8_t* data, int32_t data_size);
static void xrsp_send_to_topic_raw(struct ql_xrsp_host *host, uint8_t topic, const uint8_t* data, int32_t data_size);

static void xrsp_flush_stream(struct ql_xrsp_host *host, int64_t target_ns, int index, int slice_idx);
static void xrsp_start_encode(struct ql_xrsp_host *host,  int64_t target_ns, int index, int slice_idx);
static void xrsp_send_csd(struct ql_xrsp_host *host, const uint8_t* data, size_t data_len, int index, int slice_idx);
static void xrsp_send_idr(struct ql_xrsp_host *host, const uint8_t* data, size_t data_len, int index, int slice_idx);
static void xrsp_send_video(struct ql_xrsp_host *host, int index, int slice_idx, int frame_idx, int64_t frame_started_ns, const uint8_t* csd_dat, size_t csd_len,
                            const uint8_t* video_dat, size_t video_len, int blit_y_pos);
static void xrsp_init_session_bye(struct ql_xrsp_host *host);
int ql_xrsp_usb_init(struct ql_xrsp_host* host, bool do_reset);
static void xrsp_send_mesh(struct ql_xrsp_host *host);
static void xrsp_send_audio_control(struct ql_xrsp_host *host, uint16_t a, uint16_t b, uint32_t c, float d, float e);
static void xrsp_send_input_control(struct ql_xrsp_host *host, uint16_t a, uint16_t b, uint32_t c, float d, float e);

int ql_xrsp_host_create(struct ql_xrsp_host* host, uint16_t vid, uint16_t pid, int if_num)
{
    int ret;

    *host = (struct ql_xrsp_host){0};
    host->if_num = if_num;
    host->vid = vid;
    host->pid = pid;

    host->num_slices = QL_NUM_SLICES;

    host->ready_to_send_frames = false;
    host->sent_first_frame = false;
    host->stream_read_idx = 0;
    host->stream_write_idx = 0;
    for (int i = 0; i < QL_SWAPCHAIN_DEPTH; i++)
    {
        for (int j = 0; j < QL_NUM_SLICES; j++)
        {
            host->csd_stream[QL_IDX_SLICE(j, i)] = (uint8_t*)malloc(0x1000000);
            host->idr_stream[QL_IDX_SLICE(j, i)] = (uint8_t*)malloc(0x1000000);
            host->csd_stream_len[QL_IDX_SLICE(j, i)] = 0;
            host->idr_stream_len[QL_IDX_SLICE(j, i)] = 0;

            host->stream_started_ns[QL_IDX_SLICE(j, i)] = 0;
            host->encode_started_ns[QL_IDX_SLICE(j, i)] = 0;
            host->encode_done_ns[QL_IDX_SLICE(j, i)] = 0;
            host->encode_duration_ns[QL_IDX_SLICE(j, i)] = 0;
            host->tx_started_ns[QL_IDX_SLICE(j, i)] = 0;
            host->tx_done_ns[QL_IDX_SLICE(j, i)] = 0;
            host->tx_duration_ns[QL_IDX_SLICE(j, i)] = 0;

            ret = os_mutex_init(&host->stream_mutex[QL_IDX_SLICE(j, i)]);
            if (ret != 0) {
                QUEST_LINK_ERROR("Failed to init usb mutex");
                goto cleanup;
            }
        }
    }
    
    host->frame_idx = 0;
    ret = os_mutex_init(&host->usb_mutex);
    if (ret != 0) {
        QUEST_LINK_ERROR("Failed to init usb mutex");
        goto cleanup;
    }

    ret = os_mutex_init(&host->pose_mutex);
    if (ret != 0) {
        QUEST_LINK_ERROR("Failed to init pose mutex");
        goto cleanup;
    }
    

    //
    // Thread and other state.
    ret = os_thread_helper_init(&host->read_thread);
    if (ret != 0) {
        QUEST_LINK_ERROR("Failed to init packet read processing thread");
        goto cleanup;
    }

    //
    // Thread and other state.
    ret = os_thread_helper_init(&host->write_thread);
    if (ret != 0) {
        QUEST_LINK_ERROR("Failed to init packet write processing thread");
        goto cleanup;
    }

    host->dev = NULL;

    ret = libusb_init(&host->ctx);
    if (ret < 0) {
        QUEST_LINK_ERROR("Failed libusb_init");
        goto cleanup;
    }

    ret = ql_xrsp_usb_init(host, false);
    if (ret != 0) {
        goto cleanup;
    }

    //QUEST_LINK_INFO("Endpoints %x %x", host->ep_out, host->ep_in);

    host->pairing_state = PAIRINGSTATE_WAIT_FIRST;
    host->start_ns = os_monotonic_get_ns();
    host->paired_ns = os_monotonic_get_ns()*2;
    host->last_read_ns = 0;
    xrsp_reset_echo(host);

    host->start_encode = xrsp_start_encode;
    host->send_csd = xrsp_send_csd;
    host->send_idr = xrsp_send_idr;
    host->flush_stream = xrsp_flush_stream;

    host->client_id = 0x4a60dcca;
    host->session_idx = 3;
    host->runtime_connected = false;
    host->bodyapi_connected = false;
    host->eyetrack_connected = false;

    // Start the packet reading thread
    ret = os_thread_helper_start(&host->read_thread, ql_xrsp_read_thread, host);
    if (ret != 0) {
        QUEST_LINK_ERROR("Failed to start packet processing thread");
        goto cleanup;
    }

    // Start the packet reading thread
    ret = os_thread_helper_start(&host->write_thread, ql_xrsp_write_thread, host);
    if (ret != 0) {
        QUEST_LINK_ERROR("Failed to start packet processing thread");
        goto cleanup;
    }

    return 0;
cleanup:
    return -1;
}

int ql_xrsp_usb_init(struct ql_xrsp_host* host, bool do_reset)
{
    int ret;
    const struct libusb_interface_descriptor* pIf = NULL;
    struct libusb_config_descriptor *config = NULL;
    libusb_device *usb_dev = NULL;

    QUEST_LINK_INFO("(Re)initializing Quest Link USB device...");

    host->last_init_ns = os_monotonic_get_ns();

    os_mutex_lock(&host->usb_mutex);

    if (host->dev) {
        libusb_close(host->dev);
    }

    host->usb_speed = LIBUSB_SPEED_LOW;
    host->usb_valid = false;
    host->pairing_state = PAIRINGSTATE_WAIT_FIRST;
    host->pairing2_invites = 0;
    host->active_session_type = 0;
    host->ready_to_send_frames = false;
    host->sent_first_frame = false;

    // Retry: right after (re)enumeration udev may not have granted access yet.
    for (int i = 0; i < 10; i++)
    {
        host->dev = libusb_open_device_with_vid_pid(host->ctx, host->vid, host->pid);
        if (host->dev) break;

        os_nanosleep(U_TIME_1MS_IN_NS * 200);
    }
    if (host->dev == NULL) {
        QUEST_LINK_ERROR("Failed initial libusb_open_device_with_vid_pid");
        QUEST_LINK_ERROR("libusb error: %s", libusb_strerror((libusb_error)ret));
        goto cleanup;
    }

    if (do_reset) {
        QUEST_LINK_INFO("Reset?");
        ret = libusb_reset_device(host->dev);
        if (ret == LIBUSB_ERROR_NOT_FOUND) {
            // We're reconnecting anyhow.
            QUEST_LINK_ERROR("libusb error: %s", libusb_strerror((libusb_error)ret));
            QUEST_LINK_INFO("Device needs reconnect...");
        }
        else if (ret != LIBUSB_SUCCESS) {
            QUEST_LINK_ERROR("Failed libusb_reset_device");
            QUEST_LINK_ERROR("libusb error: %s", libusb_strerror((libusb_error)ret));
            goto cleanup;
        }
        else {
            libusb_close(host->dev);
        }

        

        QUEST_LINK_INFO("Reset done?");

        for (int i = 0; i < 10; i++)
        {
            // Re-initialize the device
            host->dev = libusb_open_device_with_vid_pid(host->ctx, host->vid, host->pid);
            if (host->dev) break;

            os_nanosleep(U_TIME_1MS_IN_NS * 500);
        }

        if (host->dev == NULL) {
            QUEST_LINK_ERROR("Failed post-reset libusb_open_device_with_vid_pid");
            QUEST_LINK_ERROR("libusb error: %s", libusb_strerror((libusb_error)ret));
            goto cleanup;
        }
    }

    ret = libusb_claim_interface(host->dev, host->if_num);
    if (ret < 0) {
        QUEST_LINK_ERROR("Failed libusb_claim_interface");
        QUEST_LINK_ERROR("libusb error: %s", libusb_strerror((libusb_error)ret));

        // Reset, there's probably something weird.
        libusb_reset_device(host->dev);
        goto cleanup;
    }

    //libusb_set_interface_alt_setting(host->dev, host->if_num, 1);

    usb_dev = libusb_get_device(host->dev);
    ret = libusb_get_active_config_descriptor(usb_dev, &config);
    if (ret < 0 || !config) {
        QUEST_LINK_ERROR("Failed libusb_get_active_config_descriptor");
        QUEST_LINK_ERROR("libusb error: %s", libusb_strerror((libusb_error)ret));
        goto cleanup;
    }

    
    for (uint8_t i = 0; i < config->bNumInterfaces; i++) {
        if (pIf) break;
        const struct libusb_interface* pIfAlts = &config->interface[i];
        for (uint8_t j = 0; j < pIfAlts->num_altsetting; j++) {
            const struct libusb_interface_descriptor* pIfIt = &pIfAlts->altsetting[j];
            if (pIfIt->bInterfaceNumber == host->if_num) {
                pIf = pIfIt;
                break;
            }
        }
    }

    host->ep_out = 0;
    host->ep_in = 0;
    for (uint8_t i = 0; i < pIf->bNumEndpoints; i++) {
        const struct libusb_endpoint_descriptor* pEp = &pIf->endpoint[i];

        if (!host->ep_out && !(pEp->bEndpointAddress & LIBUSB_ENDPOINT_IN)) {
            host->ep_out = pEp->bEndpointAddress;
        }
        else if (!host->ep_in && (pEp->bEndpointAddress & LIBUSB_ENDPOINT_IN)) {
            host->ep_in = pEp->bEndpointAddress;
        }
    }

    host->usb_slow_cable = false;
    host->usb_speed = libusb_get_device_speed(usb_dev);
    switch (host->usb_speed)
    {
    case LIBUSB_SPEED_LOW:
        host->usb_slow_cable = true;
        QUEST_LINK_ERROR("Headset is operating at 1.5Mbit/s");
        break;
    case LIBUSB_SPEED_FULL:
        host->usb_slow_cable = true;
        QUEST_LINK_ERROR("Headset is operating at 12Mbit/s");
        break;
    case LIBUSB_SPEED_HIGH:
        host->usb_slow_cable = true;
        QUEST_LINK_ERROR("Headset is operating at 480Mbit/s");
        break;
    case LIBUSB_SPEED_SUPER:
        QUEST_LINK_INFO("Headset is operating at 5000Mbit/s");
        break;
    case LIBUSB_SPEED_SUPER_PLUS:
        QUEST_LINK_INFO("Headset is operating at 10000Mbit/s");
        break;
    default:
        host->usb_slow_cable = true;
        QUEST_LINK_ERROR("libusb_get_device_speed returned unknown value!");
        break;
    }

    libusb_clear_halt(host->dev, host->ep_in);
    libusb_clear_halt(host->dev, host->ep_out);
    libusb_clear_halt(host->dev, host->ep_in);
    libusb_clear_halt(host->dev, host->ep_out);

    host->usb_valid = true;    

    os_mutex_unlock(&host->usb_mutex);

    //xrsp_init_session_bye(host);

    return 0;

cleanup:
    os_mutex_unlock(&host->usb_mutex);
    return -1;
}

void ql_xrsp_host_destroy(struct ql_xrsp_host* host)
{
    ql_xrsp_audio_stop();
    libusb_release_interface(host->dev, host->if_num);
    libusb_close(host->dev);

    os_mutex_destroy(&host->pose_mutex);
    os_mutex_destroy(&host->usb_mutex);
    for (int i = 0; i < QL_SWAPCHAIN_DEPTH; i++)
    {
        for (int j = 0; j < QL_NUM_SLICES; j++)
        {
            free(host->csd_stream[QL_IDX_SLICE(j, i)]);
            free(host->idr_stream[QL_IDX_SLICE(j, i)]);
            os_mutex_destroy(&host->stream_mutex[QL_IDX_SLICE(j, i)]);
        }
    }
}

static void xrsp_flush_stream(struct ql_xrsp_host *host, int64_t target_ns, int index, int slice_idx)
{
    if (!host->ready_to_send_frames) return;

    int stream_write_idx = QL_IDX_SLICE(slice_idx, index);
    host->encode_done_ns[stream_write_idx] = xrsp_ts_ns(host);

    bool wait = false;
    os_mutex_lock(&host->stream_mutex[stream_write_idx]);

    if (host->csd_stream_len[stream_write_idx] || host->idr_stream_len[stream_write_idx]) {
        //QUEST_LINK_INFO("Write idx %x slice %x", index, slice_idx);
        host->needs_flush[stream_write_idx] = true;
        wait = true;
        host->stream_started_ns[stream_write_idx] = target_ns;

        struct ql_hmd* hmd = host->sys->hmd;
        struct xrt_space_relation out_head_relation;
        U_ZERO(&out_head_relation);

        host->encode_duration_ns[stream_write_idx] = host->encode_done_ns[stream_write_idx] - host->encode_started_ns[stream_write_idx];
        if (host->encode_duration_ns[stream_write_idx] > 8 * U_TIME_1MS_IN_NS) {
            QUEST_LINK_WARN("Slow encode: %.1fms (idx %d)", (double)host->encode_duration_ns[stream_write_idx] / U_TIME_1MS_IN_NS, stream_write_idx);
        }

        static int64_t last_ns = 0;
        int64_t delta = host->stream_started_ns[stream_write_idx] - last_ns;
        //QUEST_LINK_INFO("%zx -> %ffps", delta, 1000000000.0 / (double)delta);

        last_ns = target_ns;
        os_mutex_unlock(&host->stream_mutex[stream_write_idx]);
    }
    else {
        os_mutex_unlock(&host->stream_mutex[stream_write_idx]);
    }
}

static void xrsp_start_encode(struct ql_xrsp_host *host, int64_t target_ns, int index, int slice_idx)
{
    int write_index = QL_IDX_SLICE(slice_idx, index);

    while (host->needs_flush[write_index]) {
        os_nanosleep(U_TIME_1MS_IN_NS / 10);
    }
    os_mutex_lock(&host->stream_mutex[write_index]);
    host->encode_started_ns[write_index] = xrsp_ts_ns(host);

    struct ql_hmd* hmd = host->sys->hmd;
    struct xrt_space_relation out_head_relation;
    U_ZERO(&out_head_relation);

    xrt_device_get_tracked_pose(&hmd->base, XRT_INPUT_GENERIC_HEAD_POSE, target_ns, &out_head_relation);
    host->stream_poses[write_index] = out_head_relation.pose;
    host->stream_pose_ns[write_index] = target_ns;
    os_mutex_unlock(&host->stream_mutex[write_index]);
}

static void xrsp_send_csd(struct ql_xrsp_host *host, const uint8_t* data, size_t data_len, int index, int slice_idx)
{
    int write_index = QL_IDX_SLICE(slice_idx, index);
    //QUEST_LINK_INFO("CSD");
    //if (!host->ready_to_send_frames) return;
    while (host->needs_flush[write_index]) {
        os_nanosleep(U_TIME_1MS_IN_NS / 10);
    }
    os_mutex_lock(&host->stream_mutex[write_index]);
    //bool success = xrsp_read_usb(host); 
    
    //QUEST_LINK_INFO("CSD: %x into %x", data_len, host->csd_stream_len[write_index]);
    //hex_dump(data, data_len);

    // Parameter sets come with an IDR: from here on this slice's stream is decodable.
    __atomic_store_n(&host->awaiting_idr[slice_idx], false, __ATOMIC_RELEASE);

    if (host->csd_stream_len[write_index] + data_len < 0x1000000) {
        memcpy(host->csd_stream[write_index] + host->csd_stream_len[write_index], data, data_len);
        host->csd_stream_len[write_index] += data_len;
    }

    os_mutex_unlock(&host->stream_mutex[write_index]);
}

static void xrsp_send_idr(struct ql_xrsp_host *host, const uint8_t* data, size_t data_len, int index, int slice_idx)
{
    int write_index = QL_IDX_SLICE(slice_idx, index);

    while (host->needs_flush[write_index]) {
        os_nanosleep(U_TIME_1MS_IN_NS / 10);
    }
    os_mutex_lock(&host->stream_mutex[write_index]);

    //QUEST_LINK_INFO("IDR: %x into %x for slice %x, index %x", data_len, host->idr_stream_len[write_index], slice_idx, index);

    // A frame encoded before the stream (re)started: the headset couldn't decode it.
    if (__atomic_load_n(&host->awaiting_idr[slice_idx], __ATOMIC_ACQUIRE)) {
        // Safety net: if no parameter sets ever come (an encoder that doesn't repeat them with
        // IDRs), give up waiting rather than never sending video again.
        if (++host->dropped_before_idr[slice_idx] < 240) {
            os_mutex_unlock(&host->stream_mutex[write_index]);
            return;
        }
        QUEST_LINK_WARN("No keyframe on slice %d after %d frames: sending anyway", slice_idx,
                        host->dropped_before_idr[slice_idx]);
        __atomic_store_n(&host->awaiting_idr[slice_idx], false, __ATOMIC_RELEASE);
    }

    if (host->idr_stream_len[write_index] + data_len < 0x1000000) {
        memcpy(host->idr_stream[write_index] + host->idr_stream_len[write_index], data, data_len);
        host->idr_stream_len[write_index] += data_len;
    }
    
    os_mutex_unlock(&host->stream_mutex[write_index]);
}

// USB throughput statistics: the "USB:" line every 2 s and the libusb time in "Slow USB send".
// Sends run at only 30-60 MB/s (2026-10-29/30) on a 5 Gbit/s link; this tells time spent inside
// libusb_bulk_transfer (the bus or the headset not taking data fast enough) from time between
// transfers (our side). Out counters are only written under usb_mutex.
static struct
{
    uint64_t out_bytes, out_xfers, out_ns, out_max_ns;
    std::atomic<uint64_t> in_xfers, in_bytes, in_timeouts;
    uint64_t start_ns;
} usb_stats;

static void xrsp_log_usb_stats(void)
{
    uint64_t now = os_monotonic_get_ns();
    if (usb_stats.start_ns == 0) {
        usb_stats.start_ns = now;
        return;
    }
    if (now - usb_stats.start_ns < 2 * U_TIME_1S_IN_NS)
        return;
    double secs = (double)(now - usb_stats.start_ns) / U_TIME_1S_IN_NS;
    if (usb_stats.out_bytes > 0) {
        QUEST_LINK_INFO("USB: out %.1f MB/s, %.0f MB/s inside libusb (%.0f%% of the time), %.0f transfers/s (max %.1f ms); in %.0f reads/s, %.0f timeouts/s, %.0f KB/s",
                        usb_stats.out_bytes / secs / 1e6,
                        usb_stats.out_ns ? usb_stats.out_bytes * 1e3 / usb_stats.out_ns : 0.0,
                        100.0 * usb_stats.out_ns / (now - usb_stats.start_ns),
                        usb_stats.out_xfers / secs, (double)usb_stats.out_max_ns / U_TIME_1MS_IN_NS,
                        usb_stats.in_xfers.load() / secs, usb_stats.in_timeouts.load() / secs,
                        usb_stats.in_bytes.load() / secs / 1e3);
    }
    usb_stats.out_bytes = usb_stats.out_xfers = usb_stats.out_ns = usb_stats.out_max_ns = 0;
    usb_stats.in_xfers = usb_stats.in_bytes = usb_stats.in_timeouts = 0;
    usb_stats.start_ns = now;
}

/*!
 * USB benchmark on request (the Quest VR tray's "USB benchmark"): how fast frames of a given size go
 * out, to pick a bitrate that fits the link (sends ran at 30-60 MB/s on 5 Gbit/s, and frames that
 * don't fit in a frame's time make the stream drop to a lower rate, 2026-11-09). Touching
 * ~/.cache/quest-vr/usb-bench (QL_USB_BENCH_TRIGGER overrides) starts it; the file may hold the
 * steps as frame sizes in KB ("0 256 512 1024" by default, 0 = the stream as it is). Each step lasts
 * 2 s; frames smaller than the step's size are padded with a filler-data NAL unit (H.264 type 12,
 * HEVC type 38), which decoders must discard, so the picture stays right but stutters while frames
 * take longer to send than a frame lasts. Results go to usb-bench.txt next to the trigger
 * (QL_USB_BENCH_RESULT overrides), whatever the log level: a "fps=" line, one "step" line per step
 * (key=value), then "done" (or "error=..."). Runs on the write thread, like the video sends.
 */
static struct
{
    std::string trigger, result;
    int64_t next_check_ns;
    struct timespec last_mtime;
    bool have_last;

    bool active;
    std::vector<int> steps_kb; //!< front = running step
    int64_t step_end_ns;
    FILE *out;
    uint64_t frames, bytes, padded_frames, send_ns, send_max_ns, usb_ns, late;
    std::vector<uint8_t> padded; //!< the frame being sent, with its filler
} usb_bench;

static const int64_t k_usb_bench_step_ns = 2 * U_TIME_1S_IN_NS;
static const int k_usb_bench_max_kb = 2048; // 2-4 MB keyframes have gone through before

static bool usb_bench_hevc(void)
{
    const char *codec = getenv("QL_VIDEO_CODEC");
    return codec && (strcmp(codec, "hevc") == 0 || strcmp(codec, "h265") == 0);
}

/*!
 * Appends a filler-data NAL unit to `frame` so that it's `target` bytes long (no change if it's
 * already that long, or too short a gap to fit one). Filler payload is 0xFF bytes (no start code can
 * appear in it) and ends with the RBSP stop bit (0x80).
 */
static void usb_bench_pad(std::vector<uint8_t> &frame, size_t target, bool hevc)
{
    static const uint8_t hevc_hdr[] = {0, 0, 0, 1, 38 << 1, 1};
    static const uint8_t h264_hdr[] = {0, 0, 0, 1, 12};
    const uint8_t *hdr = hevc ? hevc_hdr : h264_hdr;
    const size_t hdr_len = hevc ? sizeof(hevc_hdr) : sizeof(h264_hdr);
    if (frame.size() + hdr_len + 2 > target)
        return;
    frame.insert(frame.end(), hdr, hdr + hdr_len);
    frame.resize(target - 1, 0xFF);
    frame.push_back(0x80);
}

static void usb_bench_stop(const char *error)
{
    if (usb_bench.out) {
        if (error)
            fprintf(usb_bench.out, "error=%s\n", error);
        else
            fprintf(usb_bench.out, "done\n");
        fclose(usb_bench.out);
        usb_bench.out = NULL;
    }
    if (error)
        QUEST_LINK_ERROR("USB benchmark stopped: %s", error);
    else
        QUEST_LINK_INFO("USB benchmark done: %s", usb_bench.result.c_str());
    usb_bench.active = false;
    usb_bench.steps_kb.clear();
    usb_bench.padded = std::vector<uint8_t>();
}

static void usb_bench_start_step(int64_t now)
{
    usb_bench.frames = usb_bench.bytes = usb_bench.padded_frames = 0;
    usb_bench.send_ns = usb_bench.send_max_ns = usb_bench.usb_ns = usb_bench.late = 0;
    usb_bench.step_end_ns = now + k_usb_bench_step_ns;
}

static void usb_bench_end_step(void)
{
    const double n = usb_bench.frames ? (double)usb_bench.frames : 1.0;
    const double usb_ms = usb_bench.usb_ns / 1e6;
    fprintf(usb_bench.out,
            "step size_kb=%d frames=%llu padded=%llu avg_kb=%.0f send_avg_ms=%.2f send_max_ms=%.2f "
            "libusb_mb_s=%.1f fps=%.1f late=%llu\n",
            usb_bench.steps_kb.front(), (unsigned long long)usb_bench.frames,
            (unsigned long long)usb_bench.padded_frames, usb_bench.bytes / n / 1024.0, usb_bench.send_ns / n / 1e6,
            usb_bench.send_max_ns / 1e6, usb_ms > 0 ? usb_bench.bytes / usb_ms / 1e3 : 0.0,
            usb_bench.frames / (k_usb_bench_step_ns / 1e9), (unsigned long long)usb_bench.late);
    fflush(usb_bench.out);
}

//! Write thread, every loop: start on a touched trigger, move through the steps.
static void xrsp_usb_bench_tick(struct ql_xrsp_host *host)
{
    const int64_t now = os_monotonic_get_ns();
    if (usb_bench.active) {
        if (host->pairing_state != PAIRINGSTATE_PAIRED || !host->ready_to_send_frames) {
            usb_bench_stop("the stream stopped");
            return;
        }
        if (now < usb_bench.step_end_ns)
            return;
        usb_bench_end_step();
        usb_bench.steps_kb.erase(usb_bench.steps_kb.begin());
        if (usb_bench.steps_kb.empty())
            usb_bench_stop(NULL);
        else
            usb_bench_start_step(now);
        return;
    }

    if (now < usb_bench.next_check_ns)
        return;
    usb_bench.next_check_ns = now + 200 * U_TIME_1MS_IN_NS;
    if (usb_bench.trigger.empty()) {
        const char *env = getenv("QL_USB_BENCH_TRIGGER");
        const char *home = getenv("HOME");
        usb_bench.trigger = env && *env ? env : std::string(home ? home : "/tmp") + "/.cache/quest-vr/usb-bench";
        env = getenv("QL_USB_BENCH_RESULT");
        usb_bench.result = env && *env ? env : usb_bench.trigger + ".txt";
    }
    // Same rules as the stream reset: a file already there at startup is an old request.
    struct stat st;
    if (stat(usb_bench.trigger.c_str(), &st) != 0) {
        usb_bench.have_last = true;
        return;
    }
    const bool changed = st.st_mtim.tv_sec != usb_bench.last_mtime.tv_sec || st.st_mtim.tv_nsec != usb_bench.last_mtime.tv_nsec;
    const bool first = !usb_bench.have_last;
    usb_bench.last_mtime = st.st_mtim;
    usb_bench.have_last = true;
    if (!changed || first)
        return;

    usb_bench.steps_kb.clear();
    if (FILE *f = fopen(usb_bench.trigger.c_str(), "r")) {
        int kb;
        while (fscanf(f, "%d", &kb) == 1)
            usb_bench.steps_kb.push_back(kb < 0 ? 0 : kb > k_usb_bench_max_kb ? k_usb_bench_max_kb : kb);
        fclose(f);
    }
    if (usb_bench.steps_kb.empty())
        usb_bench.steps_kb = {0, 256, 512, 1024};

    usb_bench.out = fopen((usb_bench.result + ".tmp").c_str(), "w");
    if (!usb_bench.out || rename((usb_bench.result + ".tmp").c_str(), usb_bench.result.c_str()) != 0) {
        QUEST_LINK_ERROR("USB benchmark: can't write %s", usb_bench.result.c_str());
        if (usb_bench.out)
            fclose(usb_bench.out);
        usb_bench.out = NULL;
        return;
    }
    struct ql_hmd *hmd = host->sys ? host->sys->hmd : NULL;
    fprintf(usb_bench.out, "fps=%d codec=%s\n", hmd ? (int)hmd->fps : 0, usb_bench_hevc() ? "hevc" : "h264");
    fflush(usb_bench.out);
    if (host->pairing_state != PAIRINGSTATE_PAIRED || !host->ready_to_send_frames) {
        usb_bench.active = true; // so stop() writes the error
        usb_bench_stop("not streaming (accept Quest Link in the headset first)");
        return;
    }
    QUEST_LINK_INFO("USB benchmark: %zu steps of 2 s, results in %s", usb_bench.steps_kb.size(), usb_bench.result.c_str());
    usb_bench.active = true;
    usb_bench_start_step(now);
}

static void xrsp_send_usb(struct ql_xrsp_host *host, const uint8_t* data, int32_t data_size)
{
    //QUEST_LINK_INFO("Send to usb:");
    //hex_dump(data, data_size);

    if (!host->usb_valid) return;

    int sent_len = 0;
    uint64_t t0 = os_monotonic_get_ns();
    int r = libusb_bulk_transfer(host->dev, host->ep_out, (uint8_t*)data, data_size, &sent_len, 1000);
    uint64_t dt = os_monotonic_get_ns() - t0;
    usb_stats.out_bytes += sent_len;
    usb_stats.out_xfers++;
    usb_stats.out_ns += dt;
    if (dt > usb_stats.out_max_ns)
        usb_stats.out_max_ns = dt;
    if (r != 0 || !sent_len) {
        QUEST_LINK_ERROR("Failed to send %x bytes (sent %x)", data_size, sent_len);
        QUEST_LINK_ERROR("libusb error: %s", libusb_strerror((libusb_error)r));

        if (r == LIBUSB_ERROR_NO_DEVICE || r == LIBUSB_ERROR_TIMEOUT) {
           host->usb_valid = false;
           host->pairing_state = PAIRINGSTATE_WAIT_FIRST;
        }
    }
    else {
        //QUEST_LINK_INFO("Sent %x bytes", sent_len);
    }
}

void xrsp_send_to_topic_capnp_wrapped(struct ql_xrsp_host *host, uint8_t topic, uint32_t idx, const uint8_t* data, int32_t data_size)
{
    uint32_t preamble[2] = {idx, static_cast<uint32_t>(data_size) >> 3};
    xrsp_send_to_topic(host, topic, (uint8_t*)preamble, sizeof(uint32_t) * 2);
    xrsp_send_to_topic(host, topic, data, data_size);
}

void xrsp_send_to_topic_capnp_segments(struct ql_xrsp_host *host, uint8_t topic, uint32_t idx, kj::ArrayPtr<const kj::ArrayPtr<const capnp::word>>& data)
{
    int num_segments = data.size();

    uint32_t* preamble = (uint32_t*)malloc(sizeof(uint32_t) * (num_segments + 1));
    preamble[0] = idx;
    for (int i = 0; i < num_segments; i++)
    {
        size_t packed_data_size = data[i].size()*sizeof(uint64_t);

        preamble[i+1] = static_cast<uint32_t>(packed_data_size) >> 3;
    }

    xrsp_send_to_topic(host, topic, (uint8_t*)preamble, sizeof(uint32_t) * (num_segments + 1));

    for (int i = 0; i < num_segments; i++)
    {
        uint8_t* packed_data = (uint8_t*)data[i].begin();
        size_t packed_data_size = data[i].size()*sizeof(uint64_t);

        xrsp_send_to_topic(host, topic, packed_data, packed_data_size);
    }
}

void xrsp_send_to_topic(struct ql_xrsp_host *host, uint8_t topic, const uint8_t* data, int32_t data_size)
{
    //QUEST_LINK_INFO("Send to topic %s", xrsp_topic_str(topic));
    //hex_dump(data, data_size);

    // Checked before locking: returning with usb_mutex held would block every later USB send.
    if (!host) return;
    if (data_size <= 0) return;

    os_mutex_lock(&host->usb_mutex);

    int32_t idx = 0;
    int32_t to_send = data_size;
    while (true)
    {
        if (idx >= to_send) break;

        int32_t amt = 0x3FFF8; // FFF8?
        if (idx+amt >= to_send) {
            amt = to_send - idx;
        }
        xrsp_send_to_topic_raw(host, topic, data + idx, amt);

        idx += amt;
    }
    os_mutex_unlock(&host->usb_mutex);
}

static void xrsp_send_to_topic_raw(struct ql_xrsp_host *host, uint8_t topic, const uint8_t* data, int32_t data_size)
{
    //QUEST_LINK_INFO("Send to topic raw %s", xrsp_topic_str(topic));
    //hex_dump(data, data_size);

    if (!host) return;
    //if (data_size <= 0) return;

    int32_t real_len = data_size;
    int32_t align_up_bytes = (((4+data_size) >> 2) << 2) - data_size;
    if (align_up_bytes == 4) {
        align_up_bytes = 0;
    }

    //QUEST_LINK_INFO("align up %x, %x, %x", align_up_bytes, data_size, real_len);

    // TODO place this in a fixed buffer?
    uint8_t* msg = (uint8_t*)malloc(data_size + align_up_bytes + sizeof(xrsp_topic_header) + 0x400);
    uint8_t* msg_payload = msg + sizeof(xrsp_topic_header);
    int32_t msg_size = data_size + align_up_bytes + sizeof(xrsp_topic_header);

    // Sometimes we can end up with 0x4 bytes leftover, so we have to pad a bit extra
    int32_t to_fill_check = 0x400 - ((msg_size + 0x400) & 0x3FF);
    if (to_fill_check >= 0 && to_fill_check < 8) {
        align_up_bytes += to_fill_check;

        msg_size = data_size + align_up_bytes + sizeof(xrsp_topic_header);
    }

    xrsp_topic_header* header = (xrsp_topic_header*)msg;
    header->version_maybe = 0;
    header->has_alignment_padding = align_up_bytes ? 1 : 0;
    header->packet_version_is_internal = 1;
    header->packet_version_number = 0;
    header->topic = topic;
    header->unk_14_15 = 0;

    header->num_words = ((data_size + align_up_bytes) >> 2) + 1;
    header->sequence_num = host->increment;
    header->pad = 0;

    memcpy(msg_payload, data, data_size);

    if (align_up_bytes)
    {
        if (align_up_bytes > 1)
        {
            memset(msg_payload+data_size, 0xDE, align_up_bytes-1);
        }
        msg_payload[data_size + align_up_bytes - 1] = align_up_bytes;
    }

    uint8_t* msg_end = msg + msg_size;
    memset(msg_end, 0, 0x400);

    int32_t to_fill = 0x400 - ((msg_size + 0x400) & 0x3FF) - 8;
    int32_t final_size = (msg_size + 8 + to_fill);
    //QUEST_LINK_INFO("final_size=%x, to_fill=%x, msg_size=%x", final_size, to_fill, msg_size);
    if (to_fill < 0x3f8 && to_fill >= 0) { // && final_size <= 0x10000
        xrsp_topic_header* fill_header = (xrsp_topic_header*)msg_end;
        fill_header->version_maybe = 0;
        fill_header->has_alignment_padding = 0;
        fill_header->packet_version_is_internal = 1;
        fill_header->packet_version_number = 0;
        fill_header->topic = 0;
        fill_header->unk_14_15 = 0;

        fill_header->num_words = (to_fill >> 2) + 1;
        fill_header->sequence_num = host->increment;
        fill_header->pad = 0;
        msg_size += to_fill + sizeof(xrsp_topic_header);
    }

    xrsp_send_usb(host, msg, msg_size);
    host->increment += 1;

    free(msg);
}

static void xrsp_reset_echo(struct ql_xrsp_host *host)
{
    host->echo_idx = 1;
    host->ns_offset = 0;
    host->ns_offset_from_target = 0;
    host->last_xmt = 0;

    host->echo_req_sent_ns = 0; // client ns
    host->echo_req_recv_ns = 0; // server ns
    host->echo_resp_sent_ns = 0; // server ns
    host->echo_resp_recv_ns = 0; // server ns

    host->frame_sent_ns = 0;
    host->add_test = 0;
    host->sent_mesh = false;
    host->is_inactive = false;

    ql_xrsp_segpkt_destroy(&host->pose_ctx);
    ql_xrsp_ipc_segpkt_destroy(&host->ipc_ctx);

    ql_xrsp_segpkt_init(&host->pose_ctx, 1, ql_xrsp_handle_pose);
    ql_xrsp_ipc_segpkt_init(&host->ipc_ctx, ql_xrsp_handle_ipc);

    if (!host->sys) return;

    struct ql_hmd* hmd = host->sys->hmd;
    if (hmd) {
        hmd->pose_ns = os_monotonic_get_ns();
    }
}

int64_t xrsp_ts_ns_from_target(struct ql_xrsp_host *host, int64_t ts)
{
    int64_t option_1 = (ts) - host->ns_offset;
    int64_t option_2 = (ts) + host->ns_offset_from_target;
    return option_1; // HACK: really need to figure out how to calculate ns_offset
    //return (option_1+option_2)>>1;
}

int64_t xrsp_ts_ns_to_target(struct ql_xrsp_host *host, int64_t ts)
{
    int64_t option_1 = (ts) + host->ns_offset;
    int64_t option_2 = (ts) - host->ns_offset_from_target;
    return option_1; // HACK: really need to figure out how to calculate ns_offset
    //return (option_1+option_2)>>1;
}
   
int64_t xrsp_target_ts_ns(struct ql_xrsp_host *host)
{
    return xrsp_ts_ns_to_target(host, xrsp_ts_ns(host));
}

int64_t xrsp_ts_ns(struct ql_xrsp_host *host)
{
    return os_monotonic_get_ns();
}

static void xrsp_send_ping(struct ql_xrsp_host *host)
{
    if (xrsp_ts_ns(host) - host->echo_req_sent_ns < 16000000) // 16ms
    {
        return;
    }

    host->echo_req_sent_ns = xrsp_ts_ns(host);

    //QUEST_LINK_INFO("Ping sent: xmt=%zx offs=%zx", host->echo_req_sent_ns, host->ns_offset);

    int32_t request_echo_ping_len = 0;
    uint8_t* request_echo_ping = ql_xrsp_craft_echo(ECHO_PING, host->echo_idx, 0, 0, host->echo_req_sent_ns, host->ns_offset, &request_echo_ping_len);

    xrsp_send_to_topic(host, TOPIC_HOSTINFO_ADV, request_echo_ping, request_echo_ping_len);
    free(request_echo_ping);

    host->echo_idx += 1;
}

// TODO: capnproto struct for these?
static void xrsp_init_session_bye(struct ql_xrsp_host *host)
{
    const uint8_t response_bye_payload[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int32_t response_bye_len = 0;
    uint8_t* response_bye = ql_xrsp_craft_capnp(BUILTIN_BYE, 0x3E6, 1, response_bye_payload, sizeof(response_bye_payload), &response_bye_len);

    QUEST_LINK_INFO("BYE send");

    xrsp_send_to_topic(host, TOPIC_HOSTINFO_ADV, response_bye, response_bye_len);
    free(response_bye);
}

// TODO: capnproto struct for these?
static void xrsp_init_session(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    //xrsp_read_usb(host);
    const uint8_t response_ok_payload[] = {0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x03, 0x00, 0x02, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int32_t response_ok_len = 0;
    uint8_t* response_ok = ql_xrsp_craft_capnp(BUILTIN_OK, 0x2C8, 1, response_ok_payload, sizeof(response_ok_payload), &response_ok_len);

    QUEST_LINK_INFO("OK send");

    xrsp_send_to_topic(host, TOPIC_HOSTINFO_ADV, response_ok, response_ok_len);
    free(response_ok);
}

// TODO: capnproto struct for these?
static void xrsp_send_codegen_1(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    const uint8_t response_codegen_payload[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int32_t response_codegen_len = 0;
    uint8_t* response_codegen = ql_xrsp_craft_capnp(BUILTIN_CODE_GENERATION, 0xC8, 1, response_codegen_payload, sizeof(response_codegen_payload), &response_codegen_len);

    QUEST_LINK_INFO("Codegen send");

    xrsp_send_to_topic(host, TOPIC_HOSTINFO_ADV, response_codegen, response_codegen_len);
    free(response_codegen);

    //xrsp_read_usb(host); // old
}

// TODO: capnproto struct for these?
static void xrsp_send_pairing_1(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    const uint8_t response_pairing_payload[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    int32_t response_pairing_len = 0;
    uint8_t* response_pairing = ql_xrsp_craft_capnp(BUILTIN_PAIRING, 0xC8, 1, response_pairing_payload, sizeof(response_pairing_payload), &response_pairing_len);

    QUEST_LINK_INFO("Pairing send");

    xrsp_send_to_topic(host, TOPIC_HOSTINFO_ADV, response_pairing, response_pairing_len);
    free(response_pairing);

    //xrsp_read_usb(host); // old
}

// TODO: capnproto struct for these?
static void xrsp_trigger_bye(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    const uint8_t request_video_idk[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    xrsp_send_to_topic_capnp_wrapped(host, TOPIC_VIDEO, 0, request_video_idk, sizeof(request_video_idk));

    //xrsp_init_session_bye(host);
}

// TODO: capnproto struct for these?
static void xrsp_finish_pairing_1(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    const uint8_t request_video_idk[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    QUEST_LINK_INFO("Echo send");
    xrsp_send_ping(host);

    QUEST_LINK_INFO("Video idk cmd send");
    xrsp_send_to_topic_capnp_wrapped(host, TOPIC_VIDEO, 0, request_video_idk, sizeof(request_video_idk));

    QUEST_LINK_INFO("Waiting for user to accept...");
}

// TODO: capnproto struct for these?
static void xrsp_init_session_2(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    xrsp_reset_echo(host);
    // No nested xrsp_read_usb() here (the other stages dropped theirs too): we're called from
    // inside xrsp_read_usb(), so a nested read re-handles the still-current INVITE and
    // discards whatever it read (usually the headset's ACK), then double-frees working_pkt.

    struct ql_hmd* hmd = host->sys->hmd;

    uint8_t fps = (uint8_t)hmd->fps;
    uint8_t session_type = 0x03;
    uint8_t error_code = 0x01;

    // 0x0 = AVC/H264, 0x1 = HEVC/H265. Must match the encoder (encoder_settings.cpp reads the same
    // QL_VIDEO_CODEC for VAAPI; VideoToolbox always encodes HEVC).
#ifdef XRT_HAVE_VT
    uint8_t encoding_type = 0x1;
#else
    const char * codec_env = getenv("QL_VIDEO_CODEC");
    bool hevc = codec_env && (strcmp(codec_env, "hevc") == 0 || strcmp(codec_env, "h265") == 0);
    uint8_t encoding_type = hevc ? 0x1 : 0x0;
#endif
    QUEST_LINK_INFO("Announcing %s video stream", encoding_type ? "HEVC" : "H.264");
    uint8_t response_ok_2_payload[] = {0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, session_type, 0x00, error_code, 0x00, 0x1F, 0x00, encoding_type, 0x00, (uint8_t)(host->num_slices & 0xF), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, fps, 0x00, /* invalid certs?*/0x00, /* invalid certs?*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x1B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x55, 0x53, 0x42, 0x33, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x03, 0x00, 0x02, 0x00, 0x00, 0x00};
    int32_t response_ok_2_len = 0;
    uint8_t* response_ok_2 = ql_xrsp_craft_capnp(BUILTIN_OK, 0x2C8, 1, response_ok_2_payload, sizeof(response_ok_2_payload), &response_ok_2_len);

    QUEST_LINK_INFO("Done?");

    QUEST_LINK_INFO("OK send #2");
    xrsp_send_to_topic(host, TOPIC_HOSTINFO_ADV, response_ok_2, response_ok_2_len);
    free(response_ok_2);

    QUEST_LINK_INFO("OK read #2");
}

// TODO: capnproto struct for these?
static void xrsp_send_codegen_2(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    const uint8_t response_codegen_payload[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int32_t response_codegen_len = 0;
    uint8_t* response_codegen = ql_xrsp_craft_capnp(BUILTIN_CODE_GENERATION, 0xC8, 1, response_codegen_payload, sizeof(response_codegen_payload), &response_codegen_len);

    QUEST_LINK_INFO("Codegen send #2");
    xrsp_send_to_topic(host, TOPIC_HOSTINFO_ADV, response_codegen, response_codegen_len);
    free(response_codegen);

    QUEST_LINK_INFO("Codegen read #2");
}

// TODO: capnproto struct for these?
static void xrsp_send_pairing_2(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    const uint8_t response_pairing_payload[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int32_t response_pairing_len = 0;
    uint8_t* response_pairing = ql_xrsp_craft_capnp(BUILTIN_PAIRING, 0xC8, 1, response_pairing_payload, sizeof(response_pairing_payload), &response_pairing_len);

    QUEST_LINK_INFO("Pairing send #2");
    xrsp_send_to_topic(host, TOPIC_HOSTINFO_ADV, response_pairing, response_pairing_len);
    free(response_pairing);

    QUEST_LINK_INFO("Pairing read #2");
}

typedef struct cmd_pkt_idk
{
    uint64_t a;
    uint32_t cmd_idx;

    uint32_t b;
    uint32_t c;
    uint32_t d;
    uint32_t e;
    uint32_t f;
} cmd_pkt_idk;

typedef struct body_pkt_idk
{
    uint32_t a;
    uint32_t b;
} body_pkt_idk;

static void xrsp_finish_pairing_2(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    // TODO: are those hex values just timestamps
    struct cmd_pkt_idk send_cmd_chemx_toggle = {0x0005EC94E91B9D4F, COMMAND_TOGGLE_CHEMX, 0, 0, 0, 0, 0};
    struct cmd_pkt_idk send_cmd_asw_toggle = {0x0005EC94E91B9D83, COMMAND_TOGGLE_ASW, 0, 0, 0, 0, 0};
    struct cmd_pkt_idk send_cmd_asw_disable = {0x0005EC94E91B9D83, COMMAND_TOGGLE_ASW, 0, 1, 0, 0, 0};
    struct cmd_pkt_idk send_cmd_dropframestate_toggle = {0x0005EC94E91B9D83, COMMAND_DROP_FRAMES_STATE, 0, 0, 0, 0, 0};
    struct cmd_pkt_idk send_cmd_dropframestate_disable = {0x0005EC94E91B9D83, COMMAND_DROP_FRAMES_STATE, 0, 1, 0, 0, 0};
    struct cmd_pkt_idk send_cmd_camerastream = {0x0005EC94E91B9D83, COMMAND_ENABLE_CAMERA_STREAM, 0, 0, 0, 0, 0};

    QUEST_LINK_INFO("Echo send");
    xrsp_send_ping(host);

    //xrsp_send_mesh(host);

    QUEST_LINK_INFO("Audio Control cmd send");
    xrsp_send_audio_control(host, 1, 1, 0, 0.0, 0.0);

    xrsp_send_to_topic(host, TOPIC_COMMAND, (uint8_t*)&send_cmd_chemx_toggle, sizeof(send_cmd_chemx_toggle)); // link sharpening
    xrsp_send_to_topic(host, TOPIC_COMMAND, (uint8_t*)&send_cmd_asw_toggle, sizeof(send_cmd_asw_toggle));
    //xrsp_send_to_topic(host, TOPIC_COMMAND, (uint8_t*)&send_cmd_asw_disable, sizeof(send_cmd_asw_disable));
    //xrsp_send_to_topic(host, TOPIC_COMMAND, (uint8_t*)&send_cmd_dropframestate_toggle, sizeof(send_cmd_dropframestate_toggle));
    //xrsp_send_to_topic(host, TOPIC_COMMAND, &send_cmd_camerastream, sizeof(send_cmd_camerastream));
    xrsp_send_to_topic(host, TOPIC_COMMAND, (uint8_t*)&send_cmd_dropframestate_disable, sizeof(send_cmd_dropframestate_disable));

    xrsp_send_input_control(host, 1, 1, 0, 0.0, 0.0); // Hands enable
    xrsp_send_input_control(host, 2, 1, 0, 0.0, 0.0); // Body enable
    

    // Packages?
    // com.oculus.systemdriver
    // com.facebook.spatial_persistence_service
    // com.oculus.bodyapiservice
    // com.oculus.qplservice
    // com.oculus.presence
    // com.oculus.os.dialoghost
    // com.oculus.vrguardianservice?

    // Client: com.oculus.vrshell:com.oculus.vrshell:Overlay:2352, Server: com.oculus.os.dialoghost:com.oculus.os.dialoghost (DialogHostService)

    xrsp_ripc_ensure_service_started(host, host->client_id, "com.oculus.systemdriver", "com.oculus.vrruntimeservice.VrRuntimeService");
    xrsp_ripc_connect_to_remote_server(host, RIPC_FAKE_CLIENT_1, "com.oculus.systemdriver", "com.oculus.vrruntimeservice", "RuntimeServiceServer");


    xrsp_ripc_ensure_service_started(host, host->client_id+1, "com.oculus.bodyapiservice", "com.oculus.bodyapiservice.BodyApiService");
    xrsp_ripc_connect_to_remote_server(host, RIPC_FAKE_CLIENT_2, "com.oculus.bodyapiservice", "com.oculus.bodyapiservice", "BodyApiServiceServer");

    xrsp_ripc_ensure_service_started(host, host->client_id+2, "com.oculus.bodyapiservice", "com.oculus.eyetrackingservice.EyeTrackingService");
    xrsp_ripc_connect_to_remote_server(host, RIPC_FAKE_CLIENT_3, "com.oculus.bodyapiservice", "com.oculus.eyetrackingservice", "EyeTrackingServiceServer");

    //xrsp_ripc_ensure_service_started(host, host->client_id+4, "com.oculus.vrshell", "com.oculus.panelapp.dogfood.DogfoodPanelService");
    //xrsp_ripc_ensure_service_started(host, host->client_id+4, "com.oculus.vrshell", "com.oculus.panelapp.debug.ShellDebugMultiInstanceService");

    //xrsp_ripc_ensure_service_started(host, host->client_id+3, "com.oculus.os.dialoghost", "com.oculus.os.dialoghost.DialogHostService");
    //xrsp_ripc_connect_to_remote_server(host, RIPC_FAKE_CLIENT_4, "com.oculus.os.dialoghost", "com.oculus.os.dialoghost", "DialogHostService");

    //if (!host->sent_mesh)
    {
        xrsp_send_mesh(host);
    }
}

static void xrsp_handle_echo(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    ql_xrsp_echo_payload* payload = (ql_xrsp_echo_payload*)pkt->payload;
    if ((pkt->result & 1) == 1) // PONG
    {
        host->echo_req_recv_ns = payload->recv; // server recv ns
        host->echo_resp_sent_ns = payload->xmt; // server tx ns
        host->echo_resp_recv_ns = pkt->recv_ns; // client rx ns
        host->echo_req_sent_ns = xrsp_ts_ns(host);

        int64_t calc_ns_offset = ((host->echo_req_recv_ns-host->echo_req_sent_ns) + (host->echo_resp_sent_ns-pkt->recv_ns)) >> 1; // 2

        if (!host->ns_offset) {
            host->ns_offset = calc_ns_offset;
        }
        else {
            //host->ns_offset = calc_ns_offset;
            host->ns_offset += calc_ns_offset;
            host->ns_offset /= 2;
        }

        //QUEST_LINK_INFO("Ping offs: %zx %zx %zd/%zd", host->ns_offset, -host->ns_offset_from_target, host->ns_offset-host->ns_offset_from_target, host->ns_offset_from_target-host->ns_offset);

        //QUEST_LINK_INFO("Pong get: org=%zx recv=%zx xmt=%zx offs=%zx", payload->org, payload->recv, payload->xmt, payload->offset);

        /*if (payload->offset) {
            host->ns_offset_from_target = payload->offset;
        }*/

        if (host->pairing_state == PAIRINGSTATE_PAIRED) {
            xrsp_send_ping(host);
        }
    }
    else //PING
    {
        host->last_xmt = payload->xmt;

        if (payload->offset) {
            host->ns_offset_from_target = payload->offset;
            host->ns_offset -= host->ns_offset_from_target;
            host->ns_offset /= 2;
        }

        //QUEST_LINK_INFO("Ping get: org=%zx recv=%zx xmt=%zx offs=%zx", payload->org, payload->recv, payload->xmt, payload->offset);

        int32_t request_echo_ping_len = 0;
        int64_t send_xmt = xrsp_ts_ns(host);
        uint8_t* request_echo_ping = ql_xrsp_craft_echo(ECHO_PONG, pkt->unk_4, host->last_xmt, pkt->recv_ns, send_xmt, host->ns_offset, &request_echo_ping_len);

        //QUEST_LINK_INFO("Pong sent: org=%zx recv=%zx xmt=%zx offs=%zx", host->last_xmt, pkt->recv_ns, send_xmt, host->ns_offset);

        xrsp_send_to_topic(host, TOPIC_HOSTINFO_ADV, request_echo_ping, request_echo_ping_len);
        free(request_echo_ping);

        if (host->pairing_state == PAIRINGSTATE_PAIRED) {
            xrsp_send_ping(host);
        }
    }  
}

static void xrsp_handle_invite(struct ql_xrsp_host *host, struct ql_xrsp_hostinfo_pkt* pkt)
{
    ql_xrsp_hostinfo_capnp_payload* payload = (ql_xrsp_hostinfo_capnp_payload*)pkt->payload;
    uint8_t* capnp_data = pkt->payload + sizeof(*payload);

    try
    {
        kj::ArrayPtr<const capnp::word> dataptr[1] = {kj::arrayPtr((capnp::word*)capnp_data, payload->len_u64s)};
        capnp::SegmentArrayMessageReader message(kj::arrayPtr(dataptr, 1));

        PayloadHostInfo::Reader info = message.getRoot<PayloadHostInfo>();

        // Low 16 bits of the first field are the requested session type: 1 for the basic (pairing)
        // session, 3 for the streaming session. Monado's OK payloads already echo these values.
        host->invite_session_type = info.getPayloadUnk1p0() & 0xFFFF;

        // Diagnostics: the Windows host tracks a 64-bit session id per builtin message and ignores
        // mismatches; these five unidentified u64 fields are the candidates.
        QUEST_LINK_INFO("INVITE hdr unk_4=%x result=%x fields: %08x%08x %08x%08x %08x%08x %08x%08x %08x%08x",
                        pkt->unk_4, pkt->result,
                        info.getPayloadUnk1p1(), info.getPayloadUnk1p0(), info.getPayloadUnk2p1(), info.getPayloadUnk2p0(),
                        info.getPayloadUnk3p1(), info.getPayloadUnk3p0(), info.getPayloadUnk4p1(), info.getPayloadUnk4p0(),
                        info.getPayloadUnk5p1(), info.getPayloadUnk5p0());

        HeadsetConfig::Reader config = info.getConfig();
        HeadsetDescription::Reader description = config.getDescription();
        HeadsetLens::Reader lensLeft = description.getLeftLens();
        HeadsetLens::Reader lensRight = description.getRightLens();

        struct ql_hmd* hmd = host->sys->hmd;

        // TODO mutex

        os_mutex_lock(&host->pose_mutex);
        hmd->device_type = description.getDeviceType();

        if (hmd->device_type == DEVICE_TYPE_QUEST_2) {
            hmd->fps = 120;
        }
        else if (hmd->device_type == DEVICE_TYPE_QUEST_PRO) {
            hmd->fps = 90;
        }
        else if (hmd->device_type == DEVICE_TYPE_QUEST_3) {
            hmd->fps = 90;
        }
        else {
            hmd->fps = 72;
        }

        float scale = 0.75;
        if (host->usb_slow_cable) {
            scale = 0.5;
            if (hmd->device_type == DEVICE_TYPE_QUEST_2) {
                hmd->fps = 90;
            }
        }

        int fps_override = debug_get_num_option_force_fps();
        if (fps_override > 0) {
            hmd->fps = fps_override;
        }

        float scale_override = debug_get_float_option_force_scale();
        if (scale_override > 0) {
            scale = scale_override;
        }
        
        // Quest 2:
        // 58mm (0.057928182) angle_left -> -52deg
        // 65mm (0.065298356) angle_left -> -49deg
        // 68mm (0.068259589) angle_left -> -43deg

        // Pull FOV information
        hmd->base.hmd->distortion.fov[0].angle_up = lensLeft.getAngleUp() * M_PI / 180;
        hmd->base.hmd->distortion.fov[0].angle_down = -lensLeft.getAngleDown() * M_PI / 180;
        hmd->base.hmd->distortion.fov[0].angle_left = -lensLeft.getAngleLeft() * M_PI / 180;
        hmd->base.hmd->distortion.fov[0].angle_right = lensLeft.getAngleRight() * M_PI / 180;

        hmd->base.hmd->distortion.fov[1].angle_up = lensRight.getAngleUp() * M_PI / 180;
        hmd->base.hmd->distortion.fov[1].angle_down = -lensRight.getAngleDown() * M_PI / 180;
        hmd->base.hmd->distortion.fov[1].angle_left = -lensRight.getAngleLeft() * M_PI / 180;
        hmd->base.hmd->distortion.fov[1].angle_right = lensRight.getAngleRight() * M_PI / 180;

        hmd->fov_angle_left = lensLeft.getAngleLeft();

        int w = (int)((float)description.getResolutionWidth() * scale);
        int h = (int)((float)description.getResolutionHeight() * scale);
        int w_override = debug_get_num_option_force_w();
        if (w_override > 0) {
            w = w_override;
        }
        int h_override = debug_get_num_option_force_h();
        if (h_override > 0) {
            h = h_override;
        }

        QUEST_LINK_INFO("HMD FPS is %.0f, scale is %f, w=%d, h=%d", hmd->fps, scale, w, h);
        ql_hmd_set_per_eye_resolution(hmd, w, h, /*description.getRefreshRateHz()*/ hmd->fps);

        os_mutex_unlock(&host->pose_mutex);
    }
    catch(...) {

    }
}

static void xrsp_handle_hostinfo_adv(struct ql_xrsp_host *host)
{
    int ret;
    struct ql_xrsp_topic_pkt* pkt = &host->working_pkt;

    struct ql_xrsp_hostinfo_pkt hostinfo;
    ret = ql_xrsp_hostinfo_pkt_create(&hostinfo, pkt, host);
    if (ret < 0) {
        // TODO
    }

    if (hostinfo.message_type == BUILTIN_ECHO) {
        xrsp_handle_echo(host, &hostinfo);
        return;
    }

    if (hostinfo.message_type == BUILTIN_INVITE) {
        // Pull lens and distortion info, and the requested session type.
        host->invite_session_type = 0;
        xrsp_handle_invite(host, &hostinfo);

        int session_type = host->invite_session_type;
        if (session_type == 0) {
            // Couldn't parse it: fall back to guessing from where we are in the handshake.
            session_type = (host->pairing_state == PAIRINGSTATE_WAIT_FIRST) ? 1 : 3;
        }

        // Answer the session the headset asks for. After a reconnect it can open with the
        // streaming INVITE, or interleave both; answering by our own state sent the wrong OK and
        // the headset dropped the link.
        if (session_type == 3) {
            // The headset re-sends INVITE while waiting, so repeats alone aren't a failure. Only if
            // nothing progresses for a long time do a bye + USB reset and a fresh handshake.
            int64_t now_ns = os_monotonic_get_ns();
            if (host->active_session_type != 3 || host->pairing2_invites == 0) {
                host->pairing2_invites = 0;
                host->pairing2_first_ns = now_ns;
            }
            host->pairing2_invites++;
            double waited_s = (double)(now_ns - host->pairing2_first_ns) / U_TIME_1S_IN_NS;
            QUEST_LINK_INFO("Streaming INVITE #%d, %.1fs since the first", host->pairing2_invites, waited_s);
            if (waited_s > 30.0) {
                QUEST_LINK_WARN("No pairing progress for %.0fs; resetting USB to restart pairing", waited_s);
                xrsp_trigger_bye(host, NULL);
                ql_xrsp_usb_init(host, true);
                return;
            }
            host->active_session_type = 3;
            host->pairing_state = PAIRINGSTATE_PAIRING;
            xrsp_init_session_2(host, &hostinfo);
        }
        else {
            if (session_type != 1) {
                QUEST_LINK_WARN("INVITE for unknown session type %d, answering as basic session", session_type);
            }
            QUEST_LINK_INFO("Basic (pairing) INVITE");
            host->active_session_type = 1;
            host->pairing_state = PAIRINGSTATE_WAIT_FIRST;
            host->pairing2_invites = 0;
            xrsp_init_session(host, &hostinfo);
        }
        return;
    }

    // Follow-up messages belong to whichever session the last INVITE opened.
    if (host->active_session_type == 3)
    {
        if (hostinfo.message_type == BUILTIN_ACK) {
            xrsp_send_codegen_2(host, &hostinfo);
        }
        else if (hostinfo.message_type == BUILTIN_CODE_GENERATION_ACK) {
            xrsp_send_pairing_2(host, &hostinfo);
        }
        else if (hostinfo.message_type == BUILTIN_PAIRING_ACK) {
            xrsp_finish_pairing_2(host, &hostinfo);

            host->pairing_state = PAIRINGSTATE_PAIRED;
            host->pairing2_invites = 0;

            host->paired_ns = xrsp_ts_ns(host);
        }
    }
    else
    {
        if (hostinfo.message_type == BUILTIN_ACK) {
            xrsp_send_codegen_1(host, &hostinfo);
        }
        else if (hostinfo.message_type == BUILTIN_CODE_GENERATION_ACK) {
            xrsp_send_pairing_1(host, &hostinfo);
        }
        else if (hostinfo.message_type == BUILTIN_PAIRING_ACK) {
            xrsp_finish_pairing_1(host, &hostinfo);

            host->pairing_state = PAIRINGSTATE_WAIT_SECOND;
        }
    }
}

static void xrsp_handle_pkt(struct ql_xrsp_host *host)
{
    struct ql_xrsp_topic_pkt* pkt = &host->working_pkt;

    ql_xrsp_topic_pkt_dump(pkt);

    if (pkt->topic == TOPIC_HOSTINFO_ADV)
    {
        xrsp_handle_hostinfo_adv(host);
    }
    else if (pkt->topic == TOPIC_POSE)
    {
        ql_xrsp_segpkt_consume(&host->pose_ctx, host, pkt);
    }
    else if (pkt->topic == TOPIC_HANDS)
    {
        ql_xrsp_handle_hands(host, pkt);
    }
    else if (pkt->topic == TOPIC_SKELETON)
    {
        ql_xrsp_handle_skeleton(host, pkt);
    }
    else if (pkt->topic == TOPIC_BODY)
    {
        ql_xrsp_handle_body(host, pkt);
    }
    else if (pkt->topic == TOPIC_LOGGING)
    {
        ql_xrsp_handle_logging(host, pkt);
    }
    else if (pkt->topic == TOPIC_AUDIO)
    {
        ql_xrsp_handle_audio(host, pkt); // the headset's microphone (see ql_xrsp_audio.cpp)
    }
    else if (pkt->topic == TOPIC_RUNTIME_IPC)
    {
        ql_xrsp_ipc_segpkt_consume(&host->ipc_ctx, host, pkt);
    }

    if ((pkt->topic == TOPIC_POSE || pkt->topic == TOPIC_SKELETON || pkt->topic == TOPIC_LOGGING) && host->pairing_state != PAIRINGSTATE_PAIRED)
    {
        // Stray telemetry from a just-ended session is normal while a new handshake is underway
        // (e.g. the headset opening with the streaming INVITE after a reconnect). Only force the
        // disruptive bye + USB reset once we've genuinely been stuck unpaired for a while.
        // (Grace period from triscuitcircuit/Monando_MacOS_Fork.)
        int64_t since_init_ns = os_monotonic_get_ns() - host->last_init_ns;
        if (since_init_ns > 2000LL * U_TIME_1MS_IN_NS) {
            xrsp_trigger_bye(host, NULL);
            ql_xrsp_usb_init(host, true);
        }
    }

    if (host->pairing_state == PAIRINGSTATE_PAIRED && xrsp_ts_ns(host) - host->echo_req_sent_ns > 1000000000) {
        xrsp_send_ping(host);
    }
}

static bool xrsp_read_usb(struct ql_xrsp_host *host)
{
    int ret;

    if (!host->usb_valid) return false;

    while (true)
    {
        unsigned char data[0x400];
        int32_t data_consumed = 0;

        int amt_to_read = 0x400;
        if (host->have_working_pkt) {
            amt_to_read = 0x400;
        }

        // QL_USB_READ_TIMEOUT_MS (default 1): each timeout cancels the read (a URB cancel, extra
        // xHCI commands); a longer one blocks this thread until data comes, which it does every
        // ~2 ms (poses). For USB throughput experiments.
        static const int read_timeout_ms = getenv("QL_USB_READ_TIMEOUT_MS") ? atoi(getenv("QL_USB_READ_TIMEOUT_MS")) : 1;
        int read_len = 0;
        int r = libusb_bulk_transfer(host->dev, host->ep_in, data, amt_to_read, &read_len, read_timeout_ms > 0 ? read_timeout_ms : 1);
        if (r == LIBUSB_ERROR_TIMEOUT)
            usb_stats.in_timeouts++;
        if (read_len > 0) {
            usb_stats.in_xfers++;
            usb_stats.in_bytes += read_len;
        }
        if (r != 0 || !read_len) {
            if (r != LIBUSB_ERROR_TIMEOUT) {
                QUEST_LINK_ERROR("libusb error: %s", libusb_strerror((libusb_error)r));
            }

            if (r == LIBUSB_ERROR_NO_DEVICE) {
                ql_xrsp_usb_init(host, true);
            }
            break;
        }

        if (read_len) {
            host->last_read_ns = xrsp_ts_ns(host);
        }

        //QUEST_LINK_INFO("Read %x bytes", read_len);
        //hex_dump(data, read_len);

        if (!host->have_working_pkt) {
            ret = ql_xrsp_topic_pkt_create(&host->working_pkt, data, read_len, host->last_read_ns);
            if (ret < 0) {
                // TODO
                data_consumed += 0x8;
                host->have_working_pkt = false;
            }
            else {
                data_consumed += ret;
                host->have_working_pkt = true;
            }
        }
        else if (host->working_pkt.missing_bytes == 0) {
            try {
                xrsp_handle_pkt(host);
            }
            catch(...) {
                QUEST_LINK_ERROR("Exception while parsing packet...");
            }
            

            QUEST_LINK_INFO("Is remaining data possible?");

            int32_t remaining_data = read_len - data_consumed;
        }
        else {
            ret = ql_xrsp_topic_pkt_append(&host->working_pkt, data, read_len);
            if (ret < 0) {
                // TODO
                data_consumed += 0x8;
                host->have_working_pkt = false;
            }
            else {
                data_consumed += ret;
            }
        }

        while (host->have_working_pkt) {
            if (host->working_pkt.missing_bytes == 0) {
                xrsp_handle_pkt(host);
                ql_xrsp_topic_pkt_destroy(&host->working_pkt);
                host->have_working_pkt = false;
            }

            int32_t remaining_data = read_len - data_consumed;
            if (remaining_data <= 0) {
                break;
            }

            if (remaining_data > 0 && remaining_data < 8) {
                hex_dump(&data[data_consumed], read_len-data_consumed);
                ql_xrsp_topic_pkt_destroy(&host->working_pkt);
                host->have_working_pkt = false;
            }
            else if (remaining_data > 0) {
                ret = ql_xrsp_topic_pkt_create(&host->working_pkt, &data[data_consumed], remaining_data, host->last_read_ns);
                if (ret < 0) {
                    // TODO
                    data_consumed += 0x8;
                    host->have_working_pkt = false;
                }
                else {
                    data_consumed += ret;
                    host->have_working_pkt = true;
                }
            }
        }
    }
    
    return true;
}

static void xrsp_send_mesh(struct ql_xrsp_host *host)
{
    struct ql_hmd* hmd = host->sys->hmd;

    ::capnp::MallocMessageBuilder message;
    PayloadRectifyMesh::Builder msg = message.initRoot<PayloadRectifyMesh>();

    // TODO how are the resolutions determined?
    msg.setMeshId(QL_MESH_FOVEATED);
    msg.setInputResX(hmd->encode_width); // 3680
    msg.setInputResY(hmd->encode_height); // 1920
    msg.setOutputResX(hmd->encode_width); // 4128
    msg.setOutputResY(hmd->encode_height); // 2096
    msg.setUnk2p1(0);

    ::capnp::List<MeshVtx>::Builder vertices = msg.initVertices(hmd->quest_vtx_count);
    ::capnp::List<uint16_t>::Builder indices = msg.initIndices(hmd->quest_index_count);

    for (int i = 0; i < hmd->quest_vtx_count; i++)
    {
        vertices[i].setU1(hmd->quest_vertices[(i*4)+0]);
        vertices[i].setV1(hmd->quest_vertices[(i*4)+1]);
        vertices[i].setU2(hmd->quest_vertices[(i*4)+2]);
        vertices[i].setV2(hmd->quest_vertices[(i*4)+3]);
    }

    //msg.setIndices(kj::arrayPtr(hmd->quest_indices, hmd->quest_index_count));

    for (int i = 0; i < hmd->quest_index_count; i++)
    {
        indices.set(i, hmd->quest_indices[i]);
    }

    kj::ArrayPtr<const kj::ArrayPtr<const capnp::word>> out = message.getSegmentsForOutput();

    xrsp_send_to_topic_capnp_segments(host, TOPIC_MESH, 2, out);
    
    host->sent_mesh = true;
}

// TODO: figure out the params
static void xrsp_send_audio_control(struct ql_xrsp_host *host, uint16_t a, uint16_t b, uint32_t c, float d, float e)
{
    ::capnp::MallocMessageBuilder message;
    PayloadAudioControl::Builder msg = message.initRoot<PayloadAudioControl>();

    msg.setDataUnk0(a);
    msg.setDataUnk1(b);
    msg.setDataUnk2(c);
    msg.setDataUnk3(d);
    msg.setDataUnk4(e);

    kj::ArrayPtr<const kj::ArrayPtr<const capnp::word>> out = message.getSegmentsForOutput();
    xrsp_send_to_topic_capnp_segments(host, TOPIC_AUDIO_CONTROL, 0, out);
}

// TODO: figure out the params
static void xrsp_send_input_control(struct ql_xrsp_host *host, uint16_t a, uint16_t b, uint32_t c, float d, float e)
{
    ::capnp::MallocMessageBuilder message;
    PayloadAudioControl::Builder msg = message.initRoot<PayloadAudioControl>();

    msg.setDataUnk0(a);
    msg.setDataUnk1(b);
    msg.setDataUnk2(c);
    msg.setDataUnk3(d);
    msg.setDataUnk4(e);

    kj::ArrayPtr<const kj::ArrayPtr<const capnp::word>> out = message.getSegmentsForOutput();
    xrsp_send_to_topic_capnp_segments(host, TOPIC_INPUT_CONTROL, 0, out);
}

// TODO: get this to work lol (is it possible for it to work...?)
static void xrsp_send_buffered_haptic(struct ql_xrsp_host *host, int64_t ts, ovr_haptic_target_t controller_id)
{
    if (host->pairing_state != PAIRINGSTATE_PAIRED || !host->ready_to_send_frames) {
        return;
    }
    ::capnp::MallocMessageBuilder message;
    PayloadHaptics::Builder msg = message.initRoot<PayloadHaptics>();

    msg.setTimestamp(ts); // TODO: idk what this timestamp is?
    msg.setInputType(controller_id);
    msg.setHapticType(OVR_HAPTIC_BUFFERED);
    msg.setDataUnk1p2(0x1919); // unk, usually 0
    msg.setDataUnk1p3(0x1919); // unk, usually 0
    msg.setAmplitude(1.0); // TODO idk if this is set for buffered
    msg.setPoseTimestamp(ts); // Timestamp identical to poseTimestamp in Slice, sometimes 0 if hapticType is simple?

    uint8_t* test_data = (uint8_t*)malloc(0x20);
    memset(test_data, 0xFF, 0x20);
    for (int i = 0; i < 0x20; i += 1) {
        test_data[i] = 0xFF;
    }
    
    // TODO where is this maximum defined? It seems hardcoded in XRSP tho.
    msg.setData(kj::arrayPtr(test_data, 0x19));

    kj::ArrayPtr<const kj::ArrayPtr<const capnp::word>> out = message.getSegmentsForOutput();
    xrsp_send_to_topic_capnp_segments(host, TOPIC_HAPTIC, 0, out);
    free(test_data);
}

void xrsp_send_simple_haptic(struct ql_xrsp_host *host, int64_t ts, ovr_haptic_target_t controller_id, float amplitude)
{
    if (host->pairing_state != PAIRINGSTATE_PAIRED || !host->ready_to_send_frames) {
        return;
    }
    ::capnp::MallocMessageBuilder message;
    PayloadHaptics::Builder msg = message.initRoot<PayloadHaptics>();

    msg.setTimestamp(ts);
    msg.setInputType(controller_id);
    msg.setHapticType(OVR_HAPTIC_SIMPLE);
    msg.setDataUnk1p2(0); // unk, usually 0
    msg.setDataUnk1p3(0); // unk, usually 0
    msg.setAmplitude(amplitude);
    msg.setPoseTimestamp(0);
    
    kj::ArrayPtr<const kj::ArrayPtr<const capnp::word>> out = message.getSegmentsForOutput();
    xrsp_send_to_topic_capnp_segments(host, TOPIC_HAPTIC, 0, out);
}

static void xrsp_send_video(struct ql_xrsp_host *host, int index, int slice_idx, int frame_idx, int64_t frame_started_ns, const uint8_t* csd_dat, size_t csd_len,
                            const uint8_t* video_dat, size_t video_len, int blit_y_pos)
{
    int64_t sending_pose_ns = host->stream_pose_ns[QL_IDX_SLICE(0, index)];
    int read_index = QL_IDX_SLICE(slice_idx, index);

    // Pause frame sending
    if (host->pairing_state != PAIRINGSTATE_PAIRED || !host->ready_to_send_frames)
    {
        host->tx_started_ns[read_index] = 0;
        host->tx_done_ns[read_index] = 0;
        host->tx_duration_ns[read_index] = 0;
        return;
    }

    // USB benchmark: pad the frame (its last slice) up to the step's size.
    const bool bench = usb_bench.active && slice_idx == host->num_slices - 1;
    if (bench && usb_bench.steps_kb.front() > 0) {
        const size_t target = (size_t)usb_bench.steps_kb.front() * 1024;
        if (csd_len + video_len < target) {
            usb_bench.padded.assign(video_dat, video_dat + video_len);
            usb_bench_pad(usb_bench.padded, target - csd_len, usb_bench_hevc());
            if (usb_bench.padded.size() != video_len) {
                video_dat = usb_bench.padded.data();
                video_len = usb_bench.padded.size();
                usb_bench.padded_frames++;
            }
        }
    }

    ::capnp::MallocMessageBuilder message;
    PayloadSlice::Builder msg = message.initRoot<PayloadSlice>();

    struct ql_hmd* hmd = host->sys->hmd;

    struct xrt_pose sending_pose;
    U_ZERO(&sending_pose);

    static uint64_t last_roundtrip = 0;
    uint64_t roundtrip_ns = xrsp_ts_ns(host) - last_roundtrip;
    last_roundtrip = xrsp_ts_ns(host);

    uint64_t ts_before = xrsp_ts_ns(host);
    host->tx_started_ns[read_index] = ts_before;
    uint64_t usb_ns_before = usb_stats.out_ns, usb_xfers_before = usb_stats.out_xfers;

    int bits = 0;
    if (csd_len > 0)
        bits |= 1;
    if (slice_idx == host->num_slices-1)
        bits |= 2;

    sending_pose = host->stream_poses[QL_IDX_SLICE(0, index)]; // always pull slice 0's pose

#if 0
    // If we haven't gotten a pose for a whole second, reset
    if (xrsp_ts_ns(host) - sending_pose_ns > 1000000000) {
        //host->ready_to_send_frames = false;
        libusb_reset_device(host->dev);
    }
#endif

    msg.setFrameIdx(frame_idx);
    msg.setUnk0p1(0);
    msg.setRectifyMeshId(QL_MESH_FOVEATED); // QL_MESH_NONE

    // TODO mutex

    //TODO: we need some way to know the pose as it was when the frame was rendered,
    // so that the Quest can handle timewarp for us.
    msg.setPoseQuatX(sending_pose.orientation.x);
    msg.setPoseQuatY(sending_pose.orientation.y);
    msg.setPoseQuatZ(sending_pose.orientation.z);
    msg.setPoseQuatW(sending_pose.orientation.w);
    msg.setPoseX(sending_pose.position.x);
    msg.setPoseY(sending_pose.position.y);
    msg.setPoseZ(sending_pose.position.z);

    // TODO this might include render time?
    uint64_t pipeline_pred_delta_ma = host->encode_done_ns[QL_IDX_SLICE(slice_idx, index)] - host->encode_started_ns[QL_IDX_SLICE(0, index)];//2916100;
    //uint64_t pipeline_pred_delta_ma = 0;
    //QUEST_LINK_INFO("%llu", pipeline_pred_delta_ma);

    // TODO maybe pull a round-trip delta time?
    uint64_t duration_a = (uint64_t)(1000000000.0/hmd->fps) /*+ 9116997*/; // 9ms this might also include the slice 0 pipeline_pred_delta_ma, but we don't include render time yet?
    uint64_t duration_c = pipeline_pred_delta_ma; // 4ms
    uint64_t duration_b = duration_a+duration_c; // 14ms
    uint64_t base_ts = xrsp_ts_ns_to_target(host, host->encode_started_ns[QL_IDX_SLICE(0, index)]);
    uint64_t tx_start_ts = host->tx_started_ns[QL_IDX_SLICE(0, index)];

    // all timestamps are all the same between different slices, only pipeline_pred_delta_ma changes
    msg.setPoseTimestamp(xrsp_ts_ns_to_target(host, sending_pose_ns));//xrsp_target_ts_ns(host)+41540173 // Deadline //18278312488115 // xrsp_ts_ns(host)
    msg.setSliceNum(slice_idx);
    msg.setUnk6p1(bits);
    msg.setUnk6p2(0);
    msg.setUnk6p3(0);
    msg.setBlitYPos((hmd->encode_height / host->num_slices) * slice_idx);
    msg.setCropBlocks((hmd->encode_height/16) / host->num_slices); // 24 for slice count 5
    
    /*
    unk0p0 = 74,
  unk0p1 = 0,
  unk1p0 = 1000,
  poseQuatX = -0.50216717,
  poseQuatY = 0.10189699,
  poseQuatZ = -0.093296453,
  poseQuatW = -0.85366327,
  poseX = 0.010952883,
  poseY = 0.17921059,
  poseZ = 0.18543391,
  poseTimestamp = 18789777081583,
  sliceNum = 4,
  unk6p1 = 2,
  unk6p2 = 0,
  unk6p3 = 0,
  blitYPos = 1536,
  unk7p0 = 24,
  csdSize = 0,
  videoSize = 1387,
  unk8p1 = 0,
  timestamp09 = 18789735622294,
  unkA = 5472800,
  timestamp0B = 18789764254886,
  timestamp0C = 18789759255729,
  timestamp0D = 18789744739291,


    unkA = 2916100 ... 5472800,    2.92ms ... 5.47ms
  
  timestamp09   = 18789735622294, +0          0.00ms   +0          0.00ms       transmission start?? more likely, encoding start?
  timestamp0D   = 18789744739291, +9116997    9.11ms   +9116997    9.11ms       estimated GPU end
  timestamp0C   = 18789759255729, +14516438  14.51ms   +23633435   23.63ms      deadline?
  timestamp0B   = 18789764254886, +4999157    4.99ms   +28632592   28.62ms      unknown B
  poseTimestamp = 18789777081583, +12826697  12.82ms   +41459289   41.45ms      predicted pose
  
  ( unk0 = 0,
      timestampUs = 18789759065,
      data = "Frame 74 decoded, delta in prediction time: 0.000000ms" ),
    ( unk0 = 0,
      timestampUs = 18789759084,
      data = "Glitches: 1, Mispredicts: 44, Deadline: 12.06ms, Transmission:  3.02ms, Pipeline Prediction Delta M" ),
    ( unk0 = 0,
      timestampUs = 18789759180,
      data = "Rectify: Frame using rectify meshId = 1000" ) ] )
    */

    //
    msg.setUnk8p1(0);
    msg.setTimestamp09(xrsp_ts_ns_to_target(host, tx_start_ts)-pipeline_pred_delta_ma);// transmission start
    msg.setUnkA(pipeline_pred_delta_ma); // pipeline prediction delta MA?
    msg.setTimestamp0B(base_ts+duration_a+duration_b+duration_c); // unknown
    msg.setTimestamp0C(base_ts+duration_a+duration_b); // deadline
    msg.setTimestamp0D(base_ts+duration_a); // unknown
    //QUEST_LINK_INFO("%x", host->ns_offset);

    // left eye orientation? for foveated compression weirdness?
    msg.getQuat1().setX(0.0);
    msg.getQuat1().setY(0.0);
    msg.getQuat1().setZ(0.0);
    msg.getQuat1().setW(0.0);

    // right eye orientation? for foveated compression weirdness?
    msg.getQuat1().setX(0.0);
    msg.getQuat2().setY(0.0);
    msg.getQuat2().setZ(0.0);
    msg.getQuat2().setW(0.0);

    msg.setCsdSize(csd_len);
    msg.setVideoSize(video_len);

    kj::ArrayPtr<const kj::ArrayPtr<const capnp::word>> out = message.getSegmentsForOutput();

    // The first frame of every session *must* be a keyframe with a CSD.
    int should_send = 1;
    if (!csd_len && !host->sent_first_frame)
    {
        should_send = 0;
    }

    if (should_send)
    {
        xrsp_send_to_topic_capnp_segments(host, TOPIC_SLICE_0+slice_idx, 0, out);
            
        if (csd_len)
            xrsp_send_to_topic(host, TOPIC_SLICE_0+slice_idx, csd_dat, csd_len);
            
        xrsp_send_to_topic(host, TOPIC_SLICE_0+slice_idx, video_dat, video_len);

        host->sent_first_frame = true;
    }

    uint64_t ts_after = xrsp_ts_ns(host);
    host->tx_done_ns[read_index] = ts_after;

    int64_t ts_diff = ts_after - ts_before;
    host->tx_duration_ns[read_index] = ts_diff;
    if (bench && should_send) {
        usb_bench.frames++;
        usb_bench.bytes += csd_len + video_len;
        usb_bench.send_ns += ts_diff;
        usb_bench.usb_ns += usb_stats.out_ns - usb_ns_before;
        if ((uint64_t)ts_diff > usb_bench.send_max_ns)
            usb_bench.send_max_ns = ts_diff;
        if (hmd && hmd->fps > 0 && ts_diff > (int64_t)(U_TIME_1S_IN_NS / hmd->fps))
            usb_bench.late++;
    }
    if (ts_diff > 8 * U_TIME_1MS_IN_NS) {
        double usb_ms = (double)(usb_stats.out_ns - usb_ns_before) / U_TIME_1MS_IN_NS;
        QUEST_LINK_WARN("Slow USB send: %.1fms for %zu+%zu bytes (idx %d): %llu transfers, %.1fms inside libusb (%.0f MB/s)",
                        (double)ts_diff / U_TIME_1MS_IN_NS, csd_len, video_len, read_index,
                        (unsigned long long)(usb_stats.out_xfers - usb_xfers_before), usb_ms,
                        usb_ms > 0 ? (csd_len + video_len) / usb_ms / 1e3 : 0.0);
    }

    // TODO: ehhhhh    
    xrsp_ripc_void_bool_cmd(host, host->client_id, "EnableEyeTrackingForPCLink"); 
    //xrsp_ripc_void_bool_cmd(host, host->client_id, "EnableFaceTrackingForPCLink");
    //xrsp_ripc_void_bool_cmd(host, host->client_id, "SendSwitchToHandsNotif");
    //xrsp_ripc_void_bool_cmd(host, host->client_id, "SystemButtonEventRequest");
}

static void *
ql_xrsp_read_thread(void *ptr)
{
    DRV_TRACE_MARKER();

    struct ql_xrsp_host *host = (struct ql_xrsp_host *)ptr;

    os_thread_helper_lock(&host->read_thread);
    while (os_thread_helper_is_running_locked(&host->read_thread)) {
        os_thread_helper_unlock(&host->read_thread);

        if (xrsp_ts_ns(host) - host->last_read_ns > 1000000000 && host->pairing_state == PAIRINGSTATE_WAIT_FIRST && !host->usb_valid)
        {
            ql_xrsp_usb_init(host, false);
            host->last_read_ns = xrsp_ts_ns(host);
        }

        xrsp_read_usb(host); 
        
        os_thread_helper_lock(&host->read_thread);
        if (os_thread_helper_is_running_locked(&host->read_thread)) {
            os_nanosleep(U_TIME_1MS_IN_NS / 10);
        }
    }
    os_thread_helper_unlock(&host->read_thread);

    QUEST_LINK_DEBUG("Exiting packet reading thread");

    return NULL;
}

static void *
ql_xrsp_write_thread(void *ptr)
{
    DRV_TRACE_MARKER();

    struct ql_xrsp_host *host = (struct ql_xrsp_host *)ptr;

    os_thread_helper_lock(&host->write_thread);
    while (os_thread_helper_is_running_locked(&host->write_thread)) {
        os_thread_helper_unlock(&host->write_thread);

        

        int64_t present_ns = 0x7FFFFFFFFFFFFFFF;
        int to_present = -1;
        for (int i = 0; i < QL_SWAPCHAIN_DEPTH; i++)
        {
            bool all_slices_present = true;
            for (int j = 0; j < QL_NUM_SLICES; j++)
            {
                int full_idx = QL_IDX_SLICE(j, i);
                os_mutex_lock(&host->stream_mutex[full_idx]);
                //QUEST_LINK_INFO("%x %zx %zx", host->needs_flush[i], host->stream_started_ns[i], present_ns);
                if (!host->needs_flush[full_idx]) {
                    all_slices_present = false;
                }
                os_mutex_unlock(&host->stream_mutex[full_idx]);
            }

            int first_idx = QL_IDX_SLICE(0, i);
            os_mutex_lock(&host->stream_mutex[first_idx]);
            //QUEST_LINK_INFO("%x %zx %zx", host->needs_flush[i], host->stream_started_ns[i], present_ns);
            if (all_slices_present && host->stream_started_ns[first_idx] < present_ns) {
                present_ns = host->stream_started_ns[first_idx];
                to_present = i;
            }
            os_mutex_unlock(&host->stream_mutex[first_idx]);
        }

        // TODO: merge frames together if needed
        if (to_present >= 0) { //  && (xrsp_ts_ns(host) - host->frame_sent_ns) >= 13890000
            
            for (int slice = 0; slice < QL_NUM_SLICES; slice++)
            {
                int to_present_idx = QL_IDX_SLICE(slice, to_present);
                os_mutex_lock(&host->stream_mutex[to_present_idx]);
                //QUEST_LINK_INFO("Flush: %x %x", slice, to_present);
                
                if (host->csd_stream_len[to_present_idx] || host->idr_stream_len[to_present_idx])
                    xrsp_send_video(host, to_present, slice, host->frame_idx, present_ns, (const uint8_t*)host->csd_stream[to_present_idx], host->csd_stream_len[to_present_idx], (const uint8_t*)host->idr_stream[to_present_idx], host->idr_stream_len[to_present_idx], 0);
    
                if (!slice)
                    host->frame_sent_ns = xrsp_ts_ns(host);

                host->csd_stream_len[to_present_idx] = 0;
                host->idr_stream_len[to_present_idx] = 0;
                host->needs_flush[to_present_idx] = false;

                //QUEST_LINK_INFO("Flush: %x", host->stream_read_idx);
                os_mutex_unlock(&host->stream_mutex[to_present_idx]);
            }
            host->frame_idx++;
        }

        

        //QUEST_LINK_INFO("%zx", xrsp_ts_ns(host) - host->paired_ns);
        if (xrsp_ts_ns(host) - host->paired_ns > 1000000000 && host->pairing_state == PAIRINGSTATE_PAIRED && !host->ready_to_send_frames) // && xrsp_ts_ns(host) - host->frame_sent_ns >= 16000000
        {
            host->ready_to_send_frames = true;
            host->sent_first_frame = false;

            for (int i = 0; i < QL_NUM_SLICES*QL_SWAPCHAIN_DEPTH; i++)
            {
                host->csd_stream_len[i] = 0;
                host->idr_stream_len[i] = 0;
                host->needs_flush[i] = false;
            }

            // Restart the video stream with a keyframe (see idr_generation in ql_types.h).
            for (int i = 0; i < QL_NUM_SLICES; i++) {
                __atomic_store_n(&host->dropped_before_idr[i], 0, __ATOMIC_RELAXED);
                __atomic_store_n(&host->awaiting_idr[i], true, __ATOMIC_RELEASE);
            }
            __atomic_add_fetch(&host->idr_generation, 1, __ATOMIC_RELEASE);
            QUEST_LINK_INFO("Streaming starts: requesting a keyframe");
        }

        //QUEST_LINK_INFO("%zx", xrsp_ts_ns(host) - host->last_read_ns);
        if (xrsp_ts_ns(host) - host->last_read_ns > 1000000000 && host->pairing_state == PAIRINGSTATE_WAIT_FIRST && host->usb_valid)
        {
            xrsp_trigger_bye(host, NULL);
            //xrsp_init_session_bye(host);
            //ql_xrsp_usb_init(host, true);
            host->last_read_ns = xrsp_ts_ns(host);
        }

        // Audio goes out from this thread too: messages are sent in pieces, and video from another
        // thread could slip between them.
        ql_xrsp_audio_tick(host);
        xrsp_check_stream_reset(host);
        xrsp_usb_bench_tick(host);
        os_mutex_lock(&host->usb_mutex);
        xrsp_log_usb_stats();
        os_mutex_unlock(&host->usb_mutex);

        if (host->sys) {
            struct ql_hmd* hmd = host->sys->hmd;

            if (hmd && xrsp_ts_ns(host) - hmd->pose_ns > 1000000000) {
                host->is_inactive = true;
            }
        }

        os_thread_helper_lock(&host->write_thread);

        if (os_thread_helper_is_running_locked(&host->write_thread)) {
            os_nanosleep(U_TIME_1MS_IN_NS);
        }
    }
    os_thread_helper_unlock(&host->write_thread);

    QUEST_LINK_DEBUG("Exiting packet writing thread");

    return NULL;
}
