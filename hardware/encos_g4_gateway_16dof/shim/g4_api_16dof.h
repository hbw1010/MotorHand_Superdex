#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
#ifndef LQS_G4_JOINT_STRUCT_DEFINED
#define LQS_G4_JOINT_STRUCT_DEFINED
typedef struct G4Joint {
    uint32_t bus, id, valid, error;
    float angle_deg, current_a, temp_c, speed_rpm, hz;
    uint64_t samples;
    int64_t received_ns;
    uint32_t board_timestamp_us, max_board_gap_us, max_host_gap_us, tx_failures;
} G4Joint;
#endif

// 标准 16-DOF 驱动 C ABI（与 g4_12dof_* 完全并列、零符号冲突）
uint32_t g4_16dof_joint_size(void);
uint32_t g4_16dof_axis_count(void);
uint32_t g4_16dof_target_hz(void* handle);
int64_t g4_16dof_now_ns(void);
void* g4_16dof_open(const char* port, int baud, unsigned hz);
void* g4_16dof_open_fresh(const char* port, int baud, unsigned hz);
void* g4_16dof_open_reader(const char* port);
int g4_16dof_transport_lost(void* handle);
void g4_16dof_close(void* handle);
int g4_16dof_snapshots(void* handle, G4Joint* out, int capacity);
int g4_16dof_snapshots_with_current_time(void* handle, G4Joint* out, int64_t* current_times, int capacity);
void g4_16dof_set_mit(void* handle, uint32_t bus, uint32_t id, float deg, float kp, float kd, float vel, float tor, int64_t deadline_ns);
int g4_16dof_set_mit_batch(void* handle, const float* commands, int count, int64_t deadline_ns);
int g4_16dof_renew_mit_batch(void* handle, const float* commands, int count, int64_t deadline_ns, int64_t previous_deadline_ns);
int g4_16dof_homing_mit_batch(void* handle, const float* commands, int count, uint32_t active_mask, int64_t deadline_ns);
void g4_16dof_set_servo(void* handle, uint32_t bus, uint32_t id, float deg, float spd, float cur, int64_t deadline_ns);
void g4_16dof_set_disabled_mask(void* handle, uint32_t mask);
void g4_16dof_release(void* handle, uint32_t bus, uint32_t id);
uint64_t g4_16dof_release_count(void* handle);
#ifdef __cplusplus
}
#endif
