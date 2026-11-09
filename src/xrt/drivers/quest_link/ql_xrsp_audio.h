// Copyright 2026, monadolink project
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  quest_link XRSP audio: PC audio to the headset, its microphone to the PC, probe, test tone
 * @ingroup drv_quest_link
 */

#pragma once

#include "ql_types.h"

#ifdef __cplusplus
extern "C" {
#endif

//! A packet on TOPIC_AUDIO from the headset (its microphone): played into the PC's mic sink (QL_MIC),
//! QL_AUDIO_PROBE=1 also logs the format.
void ql_xrsp_handle_audio(struct ql_xrsp_host *host, struct ql_xrsp_topic_pkt *pkt);

//! Called from the write thread's loop (~1 kHz): streams PC audio (or the test tone) to the headset.
void ql_xrsp_audio_tick(struct ql_xrsp_host *host);

//! Stops the capture thread (before the host goes away).
void ql_xrsp_audio_stop(void);

#ifdef __cplusplus
}
#endif
