// Copyright 2019-2022, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Utils for to @ref drv_quest_link.
 * @author Max Thomas <mtinc2@gmail.com>
 * @ingroup drv_quest_link
 */

#pragma once

#include "ql_types.h"

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

void hex_dump(const uint8_t* b, size_t amt);
uint32_t hash_djb2_(const char* s, uint32_t h);

#ifdef __cplusplus
}
#endif

// Constexpr djb2 hash with C++ code
#ifdef __cplusplus
constexpr uint32_t hash_djb2(const char *s, uint32_t h = 5381) {
    return !*s ? h : hash_djb2(s + 1, (33 * h) + (uint8_t)*s);
}

constexpr uint32_t ripc_field_hash(const char *typestr, const char *namestr) {
    return hash_djb2(namestr, hash_djb2(typestr));
}
#else
inline uint32_t hash_djb2(const char* s)
{
    return hash_djb2_(s, 5381);
}

inline uint32_t ripc_field_hash(const char *typestr, const char *namestr)
{
    return hash_djb2_(namestr, hash_djb2_(typestr, 5381));
}
#endif

/*!
 * Pose prediction for the Quest's head and controllers. The driver used to pass the headset's
 * velocities without the VELOCITY_VALID flags, so m_predict_relation did nothing: every "pose at
 * display time" was the last received pose, and through OpenXR apps (and SteamVR via monadolink)
 * got no velocities to predict with either. Measured 2026-10-29 with QL_POSE_STATS: poses arrive at
 * ~535 Hz (head) / ~480 Hz (controllers), 3-6 ms old, and the reported angular and linear
 * velocities match finite differences of the poses in the base frame (correlation 1.00, scale
 * 1.00), i.e. exactly what m_predict_relation expects. QL_PREDICT=0 restores the old behaviour.
 * The horizon is capped so a stalled headset (Link paused) can't send poses drifting away.
 */
static inline bool
ql_predict_enabled(void)
{
	static int on = -1;
	if (on < 0) {
		const char *env = getenv("QL_PREDICT");
		on = env == NULL || atoi(env) != 0;
	}
	return on != 0;
}

#define QL_PREDICT_MAX_S 0.05

//! Velocity flags (when prediction is on) and the clamped prediction horizon.
static inline double
ql_prediction_s(struct xrt_space_relation *relation, double prediction_s)
{
	if (!ql_predict_enabled()) {
		return prediction_s; // no velocity flags: m_predict_relation only copies the pose
	}
	relation->relation_flags = (enum xrt_space_relation_flags)(relation->relation_flags |
	                                                           XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT |
	                                                           XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
	return prediction_s > QL_PREDICT_MAX_S ? QL_PREDICT_MAX_S : prediction_s;
}
