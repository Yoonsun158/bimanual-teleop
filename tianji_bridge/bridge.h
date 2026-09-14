#ifndef BIMANUAL_TIANJI_BRIDGE_H
#define BIMANUAL_TIANJI_BRIDGE_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* ABI 2, natural alignment. All arrays are left then right. SDK units remain
 * degrees, mm, current permille, Nm. No device sample timestamp is available. */
typedef struct {
    uint64_t received_ns, packet_index;
    int32_t sequence[2], input_sequence[2], state[2], commanded_state[2];
    int32_t error[2], impedance_type[2], low_speed[2];
    double q[14], dq[14], current[14], torque[14], external_torque[14];
    double external_wrench[12], target[14], cart_k[14], cart_d[14];
    double tool_pose[12], tool_dynamics[20];
    int32_t velocity_ratio[2], acceleration_ratio[2], force_type[2];
    double impedance_rotation[14];
    int32_t pvt_run_id[2], pvt_run_state[2];
    double identification_tag[2]; /* SDK >100343007: gather channels 66/166. */
} TjFeedback;

typedef struct {
    uint64_t token, sent_ns, packet_index;
    int64_t result;
    int32_t error_number, attempted;
    uint32_t size;
    uint8_t payload[1500];
} TjSend;

typedef struct {
    uint64_t received, feedback_dropped, sends, send_dropped;
    uint64_t first_feedback_lost, last_feedback_lost;
    uint64_t first_send_lost, last_send_lost;
} TjStats;

typedef struct {
    double k[7], d[7], tool_pose[6], tool_dynamics[10];
    int32_t velocity_ratio, acceleration_ratio;
    /* Fixed base aligned impedance axes; velocity feedforward disabled. */
} TjProfile;

typedef struct {
    double q[7];
    int32_t solution_count, out_of_range, singular_mask, limit_mask;
} TjIkResult;

/* 0 success, -1 SDK/IO failure, -2 pending/busy/owned, -3 invalid argument,
 * -4 not connected. poll/latest: 1 item, 0 empty, negative error.
 * Successful submit means accepted, never executed. token > 0, increasing.
 * Sent receipts come from actual sendto, token 0 is SDK maintenance traffic.
 * Clock fields are host CLOCK_MONOTONIC ns. sent_ns is call-start when attempted
 * and the rejection/cancellation decision otherwise. Queues hold 1024 each. */
int tj_abi_version(void);
const char *tj_last_error(void);
int tj_open(const char *ipv4);
int tj_close(void); /* Connection cleanup only; caller must coordinate hold. */
int tj_versions(int64_t *sdk, int64_t *controller);
int tj_download_config(const char *local_path);
int tj_get_int(const char *name, int64_t *value);
int tj_get_float(const char *name, double *value);
int tj_poll_feedback(TjFeedback *out);
int tj_latest(TjFeedback *out);
int tj_poll_send(TjSend *out);
int tj_stats(TjStats *out);
int tj_configure(uint32_t arm_mask, const TjProfile profiles[2], uint64_t token);
int tj_engage(uint32_t arm_mask, const double q[14], uint64_t token, uint64_t expires_ns);
/* No datagram: validate reported Cartesian mode and retire prior position stop policy. */
int tj_confirm_cartesian(uint32_t arm_mask);
/* Explicit operator-authorized reset of a stationary arm with emergency error 13.
 * Does not enable servos; caller must observe fresh state=0/error=0 afterward. */
int tj_reset_emergency(int32_t arm, uint64_t token, uint64_t expires_ns);
int tj_submit(uint32_t arm_mask, const double q[14], uint64_t token, uint64_t expires_ns);
/* SDK OnSetStopRunning("A"/"B"/"AB"): result is only the SDK return, not proof of
 * stopping. All resulting physical sends are recorded with this token. */
int tj_hold(uint32_t arm_mask, uint64_t token);

/* One controller position target, joints in degrees. The controller performs
 * the motion at its velocity/acceleration percentages (1..100); no host stream
 * is required. Success means accepted for send, not motion completion.
 * On mode entry pass measured joints, wait for state=1 feedback, then submit
 * the final target separately with tj_submit; mode entry may replace a target. */
int tj_move_joints(int32_t arm, const double target_q[7], int32_t velocity_ratio,
                   int32_t acceleration_ratio, uint64_t token, uint64_t expires_ns);
/* Seed the measured pose while entering position mode at 100% velocity/acceleration.
 * Subsequent position targets use tj_submit; caller monitors motion feedback. */
int tj_position_mode(int32_t arm, const double current_q[7], uint64_t token, uint64_t expires_ns);

/* Pure local kinematics: arm 0 left/1 right. DH row major 8x4;
 * limits 7x4 [positive,negative,velocity,acceleration]; J67 4x3.
 * Tool and FK/IK poses are row major 4x4, translation mm, rotation unitless.
 * Joints degrees. IK uses ZSPType=0 with caller supplied reference. */
int tj_init_kine(int32_t arm, int32_t type, const double dh[32],
                 const double limits[28], const double j67[12],
                 const double tool[16]);
int tj_fk(int32_t arm, const double q[7], double pose[16]);
int tj_ik(int32_t arm, const double pose[16], const double reference[7],
          TjIkResult *out);
#ifdef __cplusplus
}
#endif
#endif
