// Copyright 2022, Collabora, Ltd.
// Copyright 2022 Max Thomas
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  quest_link XRSP type helpers
 * @author Max Thomas <mtinc2@gmail.com>
 * @ingroup drv_quest_link
 */

#include <stdlib.h>
#include <stdio.h>

#include "ql_xrsp_types.h"
#include "ql_utils.h"

static const char* xrsp_result_lut[35] = {
    // Our wording for the XRSP result codes (index = code); the meanings come from reverse engineering.
    "ok",
    "bad argument",
    "bad data",
    "wrong state for this operation",
    "buffer is too small",
    "allocation failed",
    "no such topic",
    "topic exists already",
    "topic can't be written",
    "topic can't be read",
    "device missing",
    "bad transport description",
    "transport was closed",
    "input/output failure",
    "timed out",
    "a packet went missing",
    "packet version not supported",
    "terminated by force",
    "no such property",
    "no active session",
    "not supported yet",
    "error of unknown cause",
    "the network host went away",
    "TLS: allocation failed",
    "TLS: cipher list rejected",
    "TLS: certificate rejected",
    "TLS: private key rejected",
    "TLS: certificate and key don't match",
    "TLS: couldn't attach the read/write descriptors",
    "TLS: handshake failure",
    "TLS: the peer sent no certificate",
    "wrong pairing code",
    "pairing was declined",
    "pairing took too long",
    "pairing: bad certificate"
};

static const char* xrsp_topics[34] = {
    "aui4a-adv",
    "hostinfo-adv",
    "Command",
    "Pose",
    "Mesh",
    "Video",
    "Audio",
    "Haptic",
    "Hands",
    "Skeleton",
    "Slice 0",
    "Slice 1",
    "Slice 2",
    "Slice 3",
    "Slice 4",
    "Slice 5",
    "Slice 6",
    "Slice 7",
    "Slice 8",
    "Slice 9",
    "Slice 10",
    "Slice 11",
    "Slice 12",
    "Slice 13",
    "Slice 14",
    "Slice 15",
    "AudioControl",
    "UserSettingsSync",
    "InputControl",
    "Asw",
    "Body",
    "RuntimeIPC",
    "CameraStream",
    "Logging",
};

const char* xrsp_topic_str(int idx) {
    if (idx < 0 || idx > TOPIC_LOGGING) {
        return "unk topic";
    }
    return xrsp_topics[idx];
}

const char* xrsp_builtin_type_str(int idx) {
    switch (idx) {
    case BUILTIN_PAIRING_ACK: return "PAIRING_ACK";
    case BUILTIN_INVITE: return "INVITE";
    case BUILTIN_OK: return "OK";
    case BUILTIN_ACK: return "ACK";
    case BUILTIN_ERROR: return "ERROR";
    case BUILTIN_BYE: return "BYE";
    case BUILTIN_ECHO: return "ECHO";
    case BUILTIN_PAIRING: return "PAIRING";
    case BUILTIN_CODE_GENERATION: return "CODE_GENERATION";
    case BUILTIN_CODE_GENERATION_ACK: return "CODE_GENERATION_ACK";
    case BUILTIN_RESERVED: return "RESERVED";
    default: return "unknown";
    }
}