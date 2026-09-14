#include "bridge.h"
#include "sdk_hooks.h"
#include "MarvinSDK.h"
#include <arpa/inet.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <ctime>
#include <fstream>
#include <string>

std::recursive_mutex tj_sdk_mutex;
namespace {
constexpr size_t capacity = 1024;
template<class T> struct Queue {
    std::array<T, capacity> data{};
    size_t head = 0, size = 0;
    bool push(const T &value) {
        if (size == capacity) return false;
        data[(head + size++) % capacity] = value;
        return true;
    }
    int pop(T *out) {
        if (!size) return 0;
        *out = data[head]; head = (head + 1) % capacity; --size;
        return 1;
    }
    void clear() { head = size = 0; }
};
std::mutex api_mutex, queue_mutex;
std::condition_variable_any send_ready;
Queue<TjFeedback> feedback;
Queue<TjSend> sent;
TjStats counters{};
TjFeedback latest{};
bool have_latest = false, connected = false, pending = false;
uint32_t position_motion_mask = 0;
uint64_t last_token = 0, pending_token = 0, pending_expiry = 0;
thread_local uint64_t context_token = 0, context_expiry = 0;
thread_local std::string error_text;
int64_t controller_version = 0;

int fail(int code, const char *message) { error_text = message; return code; }
bool finite(const double *values, size_t count) {
    if (!values) return false;
    for (size_t i = 0; i < count; ++i) if (!std::isfinite(values[i])) return false;
    return true;
}
bool mask_valid(uint32_t mask) { return mask && !(mask & ~3u); }
int ready() {
    if (!connected) return fail(-4, "not connected");
    std::lock_guard<std::recursive_mutex> lock(tj_sdk_mutex);
    if (pending) return fail(-2, "a datagram is pending; it cannot be overwritten");
    return 0;
}
int begin(uint64_t token) {
    int result = ready();
    if (result) return result;
    if (!token || token <= last_token) return fail(-3, "command token must increase");
    if (!OnClearSet()) return fail(-2, "SDK has a pending datagram");
    context_token = token;
    context_expiry = 0;
    return 0;
}
int commit(bool valid) {
    if (!valid || !OnSetSend()) {
        OnClearSet(); context_token = context_expiry = 0;
        return fail(-1, "SDK rejected command construction");
    }
    last_token = context_token;
    context_token = context_expiry = 0;
    return 0;
}
int target(uint32_t mask, const double *q, uint64_t token, uint64_t expiry, bool engage) {
    if (!mask_valid(mask) || !q || expiry <= tj_now_ns())
        return fail(-3, "invalid mask or already expired target");
    for (int arm = 0; arm < 2; ++arm)
        if ((mask & (1u << arm)) && !finite(q + arm * 7, 7)) return fail(-3, "nonfinite target");
    std::unique_lock<std::recursive_mutex> lock(tj_sdk_mutex);
    const uint64_t now = tj_now_ns();
    if (expiry <= now) return fail(-3, "target expired before construction");
    // The SDK timer sends asynchronously. Release its mutex while the previous
    // packet drains, bounded by one control period and this target's lifetime.
    send_ready.wait_for(lock, std::chrono::nanoseconds(std::min<uint64_t>(expiry - now, 5000000)),
                        [] { return !pending; });
    if (expiry <= tj_now_ns()) return fail(-3, "target expired waiting for the send slot");
    int result = begin(token);
    if (result) return result;
    context_expiry = expiry;
    bool ok = true;
    for (int arm = 0; arm < 2; ++arm) if (mask & (1u << arm)) {
        double joints[7]; std::memcpy(joints, q + 7 * arm, sizeof(joints));
        ok &= arm == 0 ? OnSetJointCmdPos_A(joints) : OnSetJointCmdPos_B(joints);
        if (engage) {
            ok &= arm == 0 ? OnSetImpType_A(2) : OnSetImpType_B(2);
            ok &= arm == 0 ? OnSetTargetState_A(3) : OnSetTargetState_B(3);
        }
    }
    return commit(ok);
}
} // namespace

int tj_failure(int code, const char *message) { return fail(code, message); }

uint64_t tj_now_ns() {
    timespec time{}; clock_gettime(CLOCK_MONOTONIC, &time);
    return uint64_t(time.tv_sec) * 1000000000ULL + time.tv_nsec;
}

void tj_hook_received(const void *data, int length) {
    const uint64_t now = tj_now_ns();
    if (length != int(sizeof(DCSS) + 2) || !data) return;
    const auto *bytes = static_cast<const unsigned char *>(data);
    if (bytes[0] != 'F' || bytes[1] != 'X') return;
    DCSS packet; std::memcpy(&packet, bytes + 2, sizeof(packet));
    TjFeedback value{}; value.received_ns = now;
    for (int arm = 0; arm < 2; ++arm) {
        const auto &state = packet.m_State[arm];
        const auto &input = packet.m_In[arm];
        const auto &output = packet.m_Out[arm];
        value.sequence[arm] = output.m_OutFrameSerial;
        value.input_sequence[arm] = input.m_InFrameSerial;
        value.state[arm] = state.m_CurState;
        value.commanded_state[arm] = state.m_CmdState;
        value.error[arm] = state.m_ERRCode;
        value.impedance_type[arm] = input.m_ImpType;
        value.low_speed[arm] = output.m_LowSpdFlag;
        value.velocity_ratio[arm] = input.m_Joint_Vel_Ratio;
        value.acceleration_ratio[arm] = input.m_Joint_Acc_Ratio;
        value.force_type[arm] = input.m_Force_Type;
        value.pvt_run_id[arm] = input.m_Pvt_RunID;
        value.pvt_run_state[arm] = input.m_Pvt_RunState;
        value.identification_tag[arm] = output.m_EST_Joint_Firc[6];
        for (int joint = 0; joint < 7; ++joint) {
            const int i = arm * 7 + joint;
            value.q[i] = output.m_FB_Joint_Pos[joint];
            value.dq[i] = output.m_FB_Joint_Vel[joint];
            value.current[i] = output.m_FB_Joint_CToq[joint];
            value.torque[i] = output.m_FB_Joint_SToq[joint];
            value.external_torque[i] = output.m_EST_Joint_Force[joint];
            value.target[i] = output.m_FB_Joint_Cmd[joint];
            value.cart_k[i] = joint == 6 ? input.m_Cart_KN : input.m_Cart_K[joint];
            value.cart_d[i] = joint == 6 ? input.m_Cart_DN : input.m_Cart_D[joint];
            value.impedance_rotation[i] = input.m_Force_PIDUL[joint];
        }
        for (int i = 0; i < 6; ++i) {
            value.external_wrench[arm * 6 + i] = output.m_EST_Cart_FN[i];
            value.tool_pose[arm * 6 + i] = input.m_ToolKine[i];
        }
        for (int i = 0; i < 10; ++i) value.tool_dynamics[arm * 10 + i] = input.m_ToolDyn[i];
    }
    std::lock_guard<std::mutex> lock(queue_mutex);
    value.packet_index = counters.received++;
    latest = value; have_latest = true;
    if (!feedback.push(value)) {
        if (counters.feedback_dropped++ == 0) counters.first_feedback_lost = value.packet_index;
        counters.last_feedback_lost = value.packet_index;
    }
}

void tj_hook_publish() {
    pending = true; pending_token = context_token; pending_expiry = context_expiry;
}

void tj_hook_send(int fd, const void *data, int length,
                  const sockaddr *address, socklen_t address_length) {
    TjSend value{};
    value.token = pending_token;
    value.sent_ns = tj_now_ns();
    value.size = uint32_t(length);
    if (length > 0 && length <= 1500) std::memcpy(value.payload, data, length);
    if (pending_expiry && value.sent_ns >= pending_expiry) {
        value.result = -1; value.error_number = ETIMEDOUT;
    } else {
        value.attempted = 1;
        value.result = sendto(fd, data, length, 0, address, address_length);
        value.error_number = value.result < 0 ? errno : 0;
    }
    pending = false; pending_token = pending_expiry = 0;
    send_ready.notify_all();
    std::lock_guard<std::mutex> lock(queue_mutex);
    value.packet_index = counters.sends++;
    if (!sent.push(value)) {
        if (counters.send_dropped++ == 0) counters.first_send_lost = value.packet_index;
        counters.last_send_lost = value.packet_index;
    }
}

extern "C" {
int tj_abi_version() { return 2; }
const char *tj_last_error() { return error_text.c_str(); }
int tj_open(const char *ipv4) {
    std::lock_guard<std::mutex> lock(api_mutex);
    if (connected) return fail(-2, "the SDK singleton already has an owner");
    in_addr address{};
    if (!ipv4 || inet_pton(AF_INET, ipv4, &address) != 1) return fail(-3, "invalid IPv4 address");
    {
        std::lock_guard<std::mutex> queue_lock(queue_mutex);
        feedback.clear(); sent.clear(); counters = {}; have_latest = false;
    }
    last_token = pending_token = pending_expiry = 0; pending = false; position_motion_mask = 0;
    const auto *ip = reinterpret_cast<const unsigned char *>(&address.s_addr);
    OnLocalLogOff(); OnLogOff();
    // The vendor C wrapper rejects loopback; its underlying implementation is
    // used for the hardware-free local peer test. All real IPs use the wrapper.
    const bool linked = ip[0] == 127 ? CRobot::OnLinkTo(ip[0], ip[1], ip[2], ip[3])
                                     : OnLinkTo(ip[0], ip[1], ip[2], ip[3]);
    if (!linked) {
        OnRelease(); return fail(-1, "SDK connection failed");
    }
    connected = true;
    long version = 0; char name[30] = "VERSION";
    if (OnGetIntPara(name, &version) != 0 || version == 0) {
        OnRelease(); connected = false;
        return fail(-1, "controller VERSION query failed");
    }
    controller_version = version;
    return 0;
}
int tj_close() {
    std::lock_guard<std::mutex> lock(api_mutex);
    if (!connected) return 0;
    const bool result = OnRelease(); connected = false;
    // A still pending target was never handed to sendto. Retain an explicit
    // cancellation result after the SDK timer and receiver have joined.
    if (pending) {
        TjSend value{};
        value.token = pending_token; value.sent_ns = tj_now_ns();
        value.result = -1; value.error_number = ECANCELED;
        std::lock_guard<std::mutex> queue_lock(queue_mutex);
        value.packet_index = counters.sends++;
        if (!sent.push(value)) {
            if (counters.send_dropped++ == 0) counters.first_send_lost = value.packet_index;
            counters.last_send_lost = value.packet_index;
        }
        pending = false; pending_token = pending_expiry = 0;
    }
    return result ? 0 : fail(-1, "SDK release failed");
}
int tj_versions(int64_t *sdk, int64_t *controller) {
    std::lock_guard<std::mutex> lock(api_mutex);
    if (!sdk || !controller) return fail(-3, "null version output");
    *sdk = SDK_VERSION; *controller = controller_version;
    return connected ? 0 : fail(-4, "not connected");
}
int tj_download_config(const char *path) {
    std::lock_guard<std::mutex> lock(api_mutex);
    int result = ready(); if (result) return result;
    if (!path || !*path) return fail(-3, "empty local config path");
    char remote[] = "/home/FUSION/Config/cfg/robot.ini";
    std::string temporary = std::string(path) + ".partial";
    const bool ok = OnRecvFile(temporary.data(), remote);
    std::ifstream input(temporary, std::ios::binary | std::ios::ate);
    if (!ok || !input || input.tellg() <= 0) {
        std::remove(temporary.c_str()); return fail(-1, "config download failed or empty");
    }
    input.close();
    if (std::rename(temporary.c_str(), path) != 0) {
        std::remove(temporary.c_str()); return fail(-1, "config download rename failed");
    }
    return 0;
}
int tj_get_int(const char *name, int64_t *value) {
    std::lock_guard<std::mutex> lock(api_mutex);
    int result = ready(); if (result) return result;
    if (!name || !value || std::strlen(name) >= 30) return fail(-3, "invalid parameter name/output");
    char key[30]{}; std::strcpy(key, name); long raw = 0;
    if (OnGetIntPara(key, &raw) != 0) return fail(-1, "integer parameter query failed");
    *value = raw; return 0;
}
int tj_get_float(const char *name, double *value) {
    std::lock_guard<std::mutex> lock(api_mutex);
    int result = ready(); if (result) return result;
    if (!name || !value || std::strlen(name) >= 30) return fail(-3, "invalid parameter name/output");
    char key[30]{}; std::strcpy(key, name);
    return OnGetFloatPara(key, value) == 0 ? 0 : fail(-1, "float parameter query failed");
}
int tj_poll_feedback(TjFeedback *out) {
    if (!out) return fail(-3, "null feedback output");
    std::lock_guard<std::mutex> lock(queue_mutex); return feedback.pop(out);
}
int tj_latest(TjFeedback *out) {
    if (!out) return fail(-3, "null feedback output");
    std::lock_guard<std::mutex> lock(queue_mutex);
    if (!have_latest) return 0;
    *out = latest; return 1;
}
int tj_poll_send(TjSend *out) {
    if (!out) return fail(-3, "null send output");
    std::lock_guard<std::mutex> lock(queue_mutex); return sent.pop(out);
}
int tj_stats(TjStats *out) {
    if (!out) return fail(-3, "null stats output");
    std::lock_guard<std::mutex> lock(queue_mutex); *out = counters; return 0;
}
int tj_configure(uint32_t mask, const TjProfile profiles[2], uint64_t token) {
    std::lock_guard<std::mutex> lock(api_mutex);
    if (!mask_valid(mask) || !profiles) return fail(-3, "invalid profile or arm mask");
    for (int arm = 0; arm < 2; ++arm) if (mask & (1u << arm)) {
        const auto &p = profiles[arm];
        if (!finite(p.k, 7) || !finite(p.d, 7) || !finite(p.tool_pose, 6) || !finite(p.tool_dynamics, 10)
            || p.velocity_ratio < 1 || p.velocity_ratio > 100 || p.acceleration_ratio < 1 || p.acceleration_ratio > 100)
            return fail(-3, "invalid control profile values");
        for (int i = 0; i < 7; ++i)
            if (p.k[i] < 0 || p.d[i] < 0 || p.d[i] > 1) return fail(-3, "invalid K or D");
    }
    std::lock_guard<std::recursive_mutex> io_lock(tj_sdk_mutex);
    int result = begin(token); if (result) return result;
    bool ok = true; double axes[7]{};
    for (int arm = 0; arm < 2; ++arm) if (mask & (1u << arm)) {
        TjProfile p = profiles[arm];
        if (arm == 0) {
            ok &= OnSetTool_A(p.tool_pose, p.tool_dynamics);
            ok &= OnSetJointLmt_A(p.velocity_ratio, p.acceleration_ratio);
            ok &= OnSetCartKD_A(p.k, p.d, 2);
            ok &= OnSetEefRot_A(1, axes);
        } else {
            ok &= OnSetTool_B(p.tool_pose, p.tool_dynamics);
            ok &= OnSetJointLmt_B(p.velocity_ratio, p.acceleration_ratio);
            ok &= OnSetCartKD_B(p.k, p.d, 2);
            ok &= OnSetEefRot_B(1, axes);
        }
        ok &= FX_OnSetVelEstStep(arm == 0 ? 'A' : 'B', 0);
    }
    return commit(ok);
}
int tj_engage(uint32_t mask, const double q[14], uint64_t token, uint64_t expiry) {
    std::lock_guard<std::mutex> lock(api_mutex); return target(mask, q, token, expiry, true);
}
int tj_submit(uint32_t mask, const double q[14], uint64_t token, uint64_t expiry) {
    std::lock_guard<std::mutex> lock(api_mutex);
    const int result = target(mask, q, token, expiry, false);
    if (result == 0) {
        // A connection may take over an already enabled position arm. Its first
        // target still needs an unconditional position stop if interrupted.
        std::lock_guard<std::mutex> queue_lock(queue_mutex);
        for (int arm = 0; arm < 2; ++arm)
            if ((mask & (1u << arm)) && have_latest && latest.state[arm] == 1)
                position_motion_mask |= 1u << arm;
    }
    return result;
}
int tj_confirm_cartesian(uint32_t mask) {
    std::lock_guard<std::mutex> lock(api_mutex);
    int result = ready(); if (result) return result;
    if (!mask_valid(mask)) return fail(-3, "invalid arm mask");
    DCSS feedback{};
    if (!OnGetBuf(&feedback)) return fail(-1, "no controller mode feedback");
    for (int arm = 0; arm < 2; ++arm) if (mask & (1u << arm)) {
        if (feedback.m_State[arm].m_CurState != 3 || feedback.m_In[arm].m_ImpType != 2)
            return fail(-1, "Cartesian mode is not reported");
    }
    // Only confirmed Cartesian engagement supersedes the earlier position
    // stop policy. Merely queueing an engagement (which can expire) does not.
    position_motion_mask &= ~mask;
    return 0;
}
int tj_reset_emergency(int32_t arm, uint64_t token, uint64_t expiry) {
    std::lock_guard<std::mutex> lock(api_mutex);
    int result = ready(); if (result) return result;
    if (arm < 0 || arm > 1 || !token || token <= last_token || expiry <= tj_now_ns())
        return fail(-3, "invalid emergency reset arm, token or expiry");
    DCSS feedback{};
    if (!OnGetBuf(&feedback)) return fail(-1, "no feedback for emergency reset");
    if (feedback.m_State[arm].m_CurState != 100 || feedback.m_State[arm].m_ERRCode != 13 ||
        feedback.m_Out[arm].m_LowSpdFlag != 1)
        return fail(-3, "reset requires stationary emergency fault 13 in state 100");
    char name[30]{}; std::strcpy(name, arm == 0 ? "RESET0" : "RESET1");
    context_token = token; context_expiry = expiry;
    // Same official parameter as OnClearErr_A/B, but do not hide a rejected
    // controller reply or retry an emergency reset automatically.
    const int code = OnSetIntPara(name, 0);
    last_token = token; context_token = context_expiry = 0;
    if (code != 0) {
        const std::string message = std::string(name) + " returned " + std::to_string(code);
        return fail(-1, message.c_str());
    }
    return 0;
}
int tj_move_joints(int32_t arm, const double target_q[7], int32_t velocity_ratio,
                   int32_t acceleration_ratio, uint64_t token, uint64_t expiry) {
    std::lock_guard<std::mutex> lock(api_mutex);
    if (arm < 0 || arm > 1 || !finite(target_q, 7) || expiry <= tj_now_ns() ||
        velocity_ratio < 1 || velocity_ratio > 100 || acceleration_ratio < 1 || acceleration_ratio > 100)
        return fail(-3, "invalid joint move arguments");
    std::lock_guard<std::recursive_mutex> sdk_lock(tj_sdk_mutex);
    int result = begin(token); if (result) return result;
    context_expiry = expiry;
    double q[7]; std::copy(target_q, target_q + 7, q);
    bool ok = arm == 0 ? OnSetJointLmt_A(velocity_ratio, acceleration_ratio)
                      : OnSetJointLmt_B(velocity_ratio, acceleration_ratio);
    ok &= arm == 0 ? OnSetJointCmdPos_A(q) : OnSetJointCmdPos_B(q);
    ok &= arm == 0 ? OnSetTargetState_A(1) : OnSetTargetState_B(1);
    result = commit(ok);
    if (result == 0) position_motion_mask |= 1u << arm;
    return result;
}
int tj_position_mode(int32_t arm, const double current_q[7], uint64_t token, uint64_t expiry) {
    return tj_move_joints(arm, current_q, 100, 100, token, expiry);
}
int tj_hold(uint32_t mask, uint64_t token) {
    std::lock_guard<std::mutex> lock(api_mutex);
    int result = ready(); if (result) return result;
    if (!mask_valid(mask) || !token || token <= last_token) return fail(-3, "invalid mask or token");
    context_token = token; context_expiry = 0;
    // A position target may be pending while low-speed is still set. Issue the
    // same official stop parameter without OnSetStopRunning's low-speed skip.
    bool ok;
    std::string stop_error = "SDK stop request failed; inspect feedback";
    if (mask & position_motion_mask) {
        char name[30]{}; std::strcpy(name, mask == 1 ? "RSTA0" : mask == 2 ? "RSTA1" : "RSTA01");
        const int code = OnSetIntPara(name, 0);
        ok = code == 0;
        if (!ok) stop_error = std::string(name) + " returned " + std::to_string(code) + "; inspect feedback";
        if (ok) position_motion_mask &= ~mask;
    } else {
        ok = OnSetStopRunning(mask == 1 ? "A" : mask == 2 ? "B" : "AB");
    }
    last_token = token; context_token = 0;
    return ok ? 0 : fail(-1, stop_error.c_str());
}
} // extern C
