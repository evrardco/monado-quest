// Copyright 2020-2021, Collabora, Ltd.
// Copyright 2022 Jan Schmidt
// Copyright 2022-2023 Max Thomas
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to quest_link driver.
 * @author Max Thomas <mtinc2@gmail.com>
 * @ingroup drv_quest_link
 */

#pragma once

#include "xrt/xrt_prober.h"

#include "ql_prober.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @defgroup drv_quest_link Meta Quest Link driver
 * @ingroup drv
 *
 * @brief Driver for Meta Quest Link and touch controllers
 *
 */

#define META_PLATFORMS_TECH_LLC_VID 0x2833
#define QUEST_SLEEPING_UMS_PID      0x0083
#define QUEST_XRSP_PID              0x0137
#define QUEST_MTP_XRSP_PID          0x0182
#define QUEST_MTP_XRSP_ADB_PID      0x0183
#define QUEST_XRSP_ADB_PID          0x0186

// Newer composite modes with an XRSP interface, from Meta's android_winusb.inf
// (oculus-drivers 1.78). The XRSP interface is found by class, not by MI number.
#define QUEST_XRSP_COMPOSITE_PIDS(X)                                                                                   \
	X(0x5007) X(0x5008) X(0x500B) X(0x500C) X(0x500D) X(0x500E) X(0x5010) X(0x5011) X(0x5012) X(0x5013) X(0x5017) \
	    X(0x5018) X(0x5019) X(0x501A)

/*!
 * Builder setup for Meta Quest Link HMD.
 *
 * @ingroup drv_quest_link
 */
struct xrt_builder *
ql_builder_create(void);

struct comp_target_factory;
struct xrt_device;

/*!
 * The compositor target that streams to the headset, if @p head is a Quest Link HMD (NULL otherwise).
 * Blocks until Quest Link is accepted in the headset. For comp_main_create_system_compositor.
 */
const struct comp_target_factory *
ql_compositor_target_factory(struct xrt_device *head);

/*!
 * @dir drivers/quest_link
 *
 * @brief @ref drv_quest_link files.
 */

#ifdef __cplusplus
}
#endif
