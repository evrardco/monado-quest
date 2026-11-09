// Copyright 2022, Collabora, Ltd.
// Copyright 2022 Max Thomas
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  quest_link XRSP Pose topic packets
 * @author Max Thomas <mtinc2@gmail.com>
 * @ingroup drv_quest_link
 */

#include <cmath>
#include <algorithm>
#include "math/m_api.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "math/m_api.h"
#include "math/m_vec3.h"

#include "ql_xrsp_segmented_pkt.h"
#include "ql_xrsp_pose.h"
#include "ql_xrsp_hostinfo.h"
#include "ql_xrsp_types.h"
#include "ql_xrsp.h"
#include "ql_types.h"
#include "ql_utils.h"
#include "ql_system.h"

#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include "protos/Pose.capnp.h"

extern "C"
{

// Diagnostics for enabling pose prediction (QL_POSE_STATS=1, logged every 2 s): Monado's Quest Link
// driver passes the headset's velocities without the VELOCITY_VALID flags, so nothing (Monado, or
// SteamVR through monadolink) predicts head or controller poses. Before trusting them, check against
// finite differences of the poses themselves: which frame the angular velocity is in (base or
// body), its scale, how often poses arrive, and how old they are on arrival (headset clock
// conversion, xrsp_ts_ns_from_target, is marked as a hack).
namespace {
struct PoseStats
{
    bool have_prev = false;
    struct xrt_pose prev = {};
    int64_t prev_ns = 0;
    int n = 0;
    double dt_sum = 0, dt_min = 1e9, dt_max = 0, age_sum = 0, age_min = 1e9, age_max = -1e9;
    // angular: reported r vs finite difference in base (w) and body (b) frames; linear: v vs d
    double rr = 0, ww = 0, bb = 0, rw = 0, rb = 0, vv = 0, dd = 0, vd = 0;
    int64_t next_log_ns = 0;

    void add(const char *name, const struct xrt_pose &pose, const struct xrt_vec3 &angvel,
             const struct xrt_vec3 &vel, int64_t pose_ns, int64_t now_ns)
    {
        const double age = (now_ns - pose_ns) / 1e6;
        if (have_prev && pose_ns > prev_ns) {
            const double dt = (pose_ns - prev_ns) / 1e9;
            struct xrt_quat inv_prev, dq_base, dq_body;
            math_quat_invert(&prev.orientation, &inv_prev);
            math_quat_rotate(&pose.orientation, &inv_prev, &dq_base); // q * prev^-1: base frame
            math_quat_rotate(&inv_prev, &pose.orientation, &dq_body); // prev^-1 * q: body frame
            const double sb = dq_base.w < 0 ? -1 : 1, sd = dq_body.w < 0 ? -1 : 1;
            const double w[3] = {2 * sb * dq_base.x / dt, 2 * sb * dq_base.y / dt, 2 * sb * dq_base.z / dt};
            const double b[3] = {2 * sd * dq_body.x / dt, 2 * sd * dq_body.y / dt, 2 * sd * dq_body.z / dt};
            const double r[3] = {angvel.x, angvel.y, angvel.z};
            const double d[3] = {(pose.position.x - prev.position.x) / dt, (pose.position.y - prev.position.y) / dt,
                                 (pose.position.z - prev.position.z) / dt};
            const double v[3] = {vel.x, vel.y, vel.z};
            for (int i = 0; i < 3; i++) {
                rr += r[i] * r[i]; ww += w[i] * w[i]; bb += b[i] * b[i]; rw += r[i] * w[i]; rb += r[i] * b[i];
                vv += v[i] * v[i]; dd += d[i] * d[i]; vd += v[i] * d[i];
            }
            dt_sum += dt * 1e3;
            dt_min = std::min(dt_min, dt * 1e3);
            dt_max = std::max(dt_max, dt * 1e3);
            age_sum += age;
            age_min = std::min(age_min, age);
            age_max = std::max(age_max, age);
            n++;
        }
        have_prev = true;
        prev = pose;
        prev_ns = pose_ns;
        if (now_ns < next_log_ns || n < 10) {
            return;
        }
        auto corr = [](double xy, double xx, double yy) { return xx > 0 && yy > 0 ? xy / std::sqrt(xx * yy) : 0.0; };
        QUEST_LINK_INFO("Pose stats %s: %.0f Hz, dt %.2f ms (%.2f..%.2f), age on arrival %.2f ms (%.2f..%.2f) | "
                        "angvel vs poses: corr base %+.2f body %+.2f, scale %.2f | linvel vs poses: corr %+.2f, scale %.2f",
                        name, 1e3 * n / dt_sum, dt_sum / n, dt_min, dt_max, age_sum / n, age_min, age_max,
                        corr(rw, rr, ww), corr(rb, rr, bb), ww > 0 ? std::sqrt(rr / ww) : 0.0, corr(vd, vv, dd),
                        dd > 0 ? std::sqrt(vv / dd) : 0.0);
        *this = PoseStats{};
        next_log_ns = now_ns + 2000000000LL;
    }
};
PoseStats head_stats, controller_stats;
bool pose_stats_enabled()
{
    static const bool on = getenv("QL_POSE_STATS") != nullptr && atoi(getenv("QL_POSE_STATS")) != 0;
    return on;
}
} // namespace

void ql_xrsp_handle_pose(struct ql_xrsp_segpkt* segpkt, struct ql_xrsp_host* host)
{
    // TODO parse segment header
    os_mutex_lock(&host->pose_mutex);

    size_t num_words = segpkt->segs_valid[0] >> 3;

    kj::ArrayPtr<const capnp::word> dataptr[1] = {kj::arrayPtr((capnp::word*)segpkt->segs[0], num_words)};
    capnp::SegmentArrayMessageReader message(kj::arrayPtr(dataptr, 1));

    PayloadPose::Reader pose;

    try
    {
        pose = message.getRoot<PayloadPose>();
    }
    catch(...)
    {
        return;
    }
    
    for (PoseTrackedController::Reader controller: pose.getControllers()) {
        int idx = 1;
        if (!(controller.getFeatures() & OVR_TOUCH_FEAT_RIGHT)) {
            idx = 0;
        }

        struct ql_controller* ctrl = host->sys->controllers[idx];
        OvrPoseF::Reader controllerPose = controller.getPose();

        ctrl->pose.position.x = controllerPose.getPosX();
        ctrl->pose.position.y = controllerPose.getPosY();
        ctrl->pose.position.z = controllerPose.getPosZ();

        ctrl->pose.orientation.x = controllerPose.getQuatX();
        ctrl->pose.orientation.y = controllerPose.getQuatY();
        ctrl->pose.orientation.z = controllerPose.getQuatZ();
        ctrl->pose.orientation.w = controllerPose.getQuatW();

        ctrl->vel = {controllerPose.getLinVelX(), controllerPose.getLinVelY(), controllerPose.getLinVelZ()};
        ctrl->acc = {controllerPose.getLinAccX(), controllerPose.getLinAccY(), controllerPose.getLinAccZ()};
        ctrl->angvel = {controllerPose.getAngVelX(), controllerPose.getAngVelY(), controllerPose.getAngVelZ()};
        ctrl->angacc = {controllerPose.getAngAccX(), controllerPose.getAngAccY(), controllerPose.getAngAccZ()};
    
        int64_t pose_ns = xrsp_ts_ns_from_target(host, controllerPose.getTimestamp());
        ctrl->pose_ns = pose_ns;
        if (pose_stats_enabled() && idx == 1) {
            controller_stats.add("right controller", ctrl->pose, ctrl->angvel, ctrl->vel, pose_ns, xrsp_ts_ns(host));
        }

        uint32_t features_raw = controller.getFeatures();

        ctrl->features = features_raw & 0xFF;
        ctrl->battery = (features_raw >> 8) & 0x7F;
        ctrl->feat_2 = (features_raw >> 15);
        ctrl->buttons = controller.getButtons();
        ctrl->capacitance = controller.getCapacitance();
        ctrl->joystick_x = controller.getStickX();
        ctrl->joystick_y = controller.getStickY();
        ctrl->grip_z = controller.getGripZ();
        ctrl->trigger_z = controller.getTriggerZ();
        ctrl->stylus_pressure = controller.getStylusPressure();
    }

    OvrPoseF::Reader headsetPose = pose.getHeadset();
    struct ql_hmd* hmd = host->sys->hmd;

    int64_t pose_ns = xrsp_ts_ns_from_target(host, headsetPose.getTimestamp());
    host->is_inactive = false;


    hmd->pose_ns = pose_ns;
    hmd->pose.position.x = headsetPose.getPosX();
    hmd->pose.position.y = headsetPose.getPosY();
    hmd->pose.position.z = headsetPose.getPosZ();

    hmd->pose.orientation.x = headsetPose.getQuatX();
    hmd->pose.orientation.y = headsetPose.getQuatY();
    hmd->pose.orientation.z = headsetPose.getQuatZ();
    hmd->pose.orientation.w = headsetPose.getQuatW();

    // The headset sometimes reports an IPD of 0 (seen 2026-10-29 when an app started while the
    // headset was held rather than worn): both views then sat at the same point, apps rendered
    // without stereo and hands looked huge up close. Keep the last plausible value instead
    // (ql_hmd_create starts with 63 mm); the Quest 2's three slider positions are 58/63/68 mm.
    const float reported_ipd = pose.getIpd();
    if (reported_ipd >= 0.050f && reported_ipd <= 0.080f) {
        if (std::fabs(reported_ipd - hmd->ipd_meters) > 0.0005f) {
            QUEST_LINK_INFO("Headset IPD: %.1f mm", reported_ipd * 1000.f);
        }
        hmd->ipd_meters = reported_ipd;
    }

    hmd->vel = {0.0, 0.0, 0.0};
    hmd->acc = {0.0, 0.0, 0.0};
    hmd->angvel = {0.0, 0.0, 0.0};
    hmd->angacc = {0.0, 0.0, 0.0};

    hmd->vel = {headsetPose.getLinVelX(), headsetPose.getLinVelY(), headsetPose.getLinVelZ()};
    hmd->acc = {headsetPose.getLinAccX(), headsetPose.getLinAccY(), headsetPose.getLinAccZ()};
    hmd->angvel = {headsetPose.getAngVelX(), headsetPose.getAngVelY(), headsetPose.getAngVelZ()};
    hmd->angacc = {headsetPose.getAngAccX(), headsetPose.getAngAccY(), headsetPose.getAngAccZ()};
    if (pose_stats_enabled()) {
        head_stats.add("head", hmd->pose, hmd->angvel, hmd->vel, pose_ns, xrsp_ts_ns(host));
    }
    
    // TODO: how is this even calculated??
    // Quest 2:
    // 58mm (0.057928182) angle_left -> -52deg
    // 65mm (0.065298356) angle_left -> -49deg
    // 68mm (0.068259589) angle_left -> -43deg
    float angle_calc = hmd->fov_angle_left;

    if (hmd->device_type == DEVICE_TYPE_QUEST_2)
    {
        if (hmd->ipd_meters <= 0.059)
        {
            angle_calc -= 0.0;
        }
        else if (hmd->ipd_meters <= 0.066)
        {
            angle_calc -= 3.0;
        }
        else {
            angle_calc -= 9.0;
        }
    }

    // Pull FOV information
    float next_angle_left = -angle_calc * M_PI / 180;
    float next_angle_right = angle_calc * M_PI / 180;

    if (hmd->base.hmd->distortion.fov[0].angle_left != next_angle_left || hmd->base.hmd->distortion.fov[1].angle_right != next_angle_right)
    {
        hmd->base.hmd->distortion.fov[0].angle_left = next_angle_left;
        hmd->base.hmd->distortion.fov[1].angle_right = next_angle_right;
    }
    
    os_mutex_unlock(&host->pose_mutex);
}

}
