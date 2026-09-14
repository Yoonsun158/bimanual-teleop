#include "bridge.h"
#include "sdk_hooks.h"
#include "kinematicsSDK/FxRobot.h"
#include <cmath>
#include <cstring>
#include <mutex>

namespace {
std::mutex kine_mutex;
bool initialized[2]{};
bool values(const double *data, int count) {
    if (!data) return false;
    for (int i = 0; i < count; ++i) if (!std::isfinite(data[i])) return false;
    return true;
}
}
extern "C" {
int tj_init_kine(int32_t arm, int32_t type, const double dh[32],
                 const double limits[28], const double j67[12], const double tool[16]) {
    std::lock_guard<std::mutex> lock(kine_mutex);
    if (arm < 0 || arm > 1 || type != 1017 || !values(dh, 32) || !values(limits, 28)
        || !values(j67, 12) || !values(tool, 16)) return tj_failure(-3, "invalid kinematics model or arm");
    initialized[arm] = false;
    double h[8][4], l[7][4], b[4][3], t[4][4];
    std::memcpy(h, dh, sizeof h); std::memcpy(l, limits, sizeof l);
    std::memcpy(b, j67, sizeof b); std::memcpy(t, tool, sizeof t);
    FX_LOG_SWITCH(0);
    const bool ok = FX_Robot_Init_Type(arm, type) && FX_Robot_Init_Kine(arm, h)
        && FX_Robot_Init_Lmt(arm, l, b) && FX_Robot_Tool_Set(arm, t)
        && FX_Robot_UserFrame_Rmv(arm);
    initialized[arm] = ok;
    return ok ? 0 : tj_failure(-1, "SDK kinematics initialization failed");
}
int tj_fk(int32_t arm, const double q[7], double pose[16]) {
    std::lock_guard<std::mutex> lock(kine_mutex);
    if (arm < 0 || arm > 1 || !values(q, 7) || !pose) return tj_failure(-3, "invalid FK arguments");
    if (!initialized[arm]) return tj_failure(-4, "kinematics not initialized");
    double joints[7], matrix[4][4]; std::memcpy(joints, q, sizeof joints);
    if (!FX_Robot_Kine_FK(arm, joints, matrix)) return tj_failure(-1, "SDK FK failed");
    std::memcpy(pose, matrix, sizeof matrix); return 0;
}
int tj_ik(int32_t arm, const double pose[16], const double reference[7], TjIkResult *out) {
    std::lock_guard<std::mutex> lock(kine_mutex);
    if (arm < 0 || arm > 1 || !values(pose, 16) || !values(reference, 7) || !out)
        return tj_failure(-3, "invalid IK arguments");
    if (!initialized[arm]) return tj_failure(-4, "kinematics not initialized");
    FX_InvKineSolvePara solve{};
    std::memcpy(solve.m_Input_IK_TargetTCP, pose, sizeof(double) * 16);
    std::memcpy(solve.m_Input_IK_RefJoint, reference, sizeof(double) * 7);
    solve.m_Input_IK_ZSPType = 0;
    const bool ok = FX_Robot_Kine_IK(arm, &solve);
    *out = {};
    std::memcpy(out->q, solve.m_Output_RetJoint, sizeof(out->q));
    out->solution_count = solve.m_OutPut_Result_Num;
    out->out_of_range = solve.m_Output_IsOutRange;
    for (int i = 0; i < 7; ++i) {
        if (solve.m_Output_IsDeg[i]) out->singular_mask |= 1 << i;
        if (solve.m_Output_JntExdTags[i]) out->limit_mask |= 1 << i;
    }
    return ok ? 0 : tj_failure(-1, "SDK IK failed; inspect reachability and singularity flags");
}
}
