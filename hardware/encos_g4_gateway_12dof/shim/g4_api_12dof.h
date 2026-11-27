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

// 标准 12-DOF 驱动 C ABI
uint32_t g4_12dof_joint_size(void);
uint32_t g4_12dof_axis_count(void);
int64_t g4_12dof_now_ns(void);
void* g4_12dof_open(const char* port, int baud, unsigned hz);
void* g4_12dof_open_fresh(const char* port, int baud, unsigned hz);
void* g4_12dof_open_reader(const char* port);
int g4_12dof_transport_lost(void* handle);
void g4_12dof_close(void* handle);
int g4_12dof_snapshots(void* handle, G4Joint* out, int capacity);
int g4_12dof_snapshots_with_current_time(void* handle, G4Joint* out, int64_t* current_times, int capacity);
void g4_12dof_set_mit(void* handle, uint32_t bus, uint32_t id, float deg, float kp, float kd, float vel, float tor, int64_t deadline_ns);
int g4_12dof_set_mit_batch(void* handle, const float* commands, int count, int64_t deadline_ns);
int g4_12dof_renew_mit_batch(void* handle, const float* commands, int count, int64_t deadline_ns, int64_t previous_deadline_ns);
int g4_12dof_homing_mit_batch(void* handle, const float* commands, int count, uint32_t active_mask, int64_t deadline_ns);
void g4_12dof_set_servo(void* handle, uint32_t bus, uint32_t id, float deg, float spd, float cur, int64_t deadline_ns);
void g4_12dof_set_disabled_mask(void* handle, uint32_t mask);
void g4_12dof_release(void* handle, uint32_t bus, uint32_t id);
uint64_t g4_12dof_release_count(void* handle);

// 向后兼容旧版 12-DOF Python / 测试符号别名
uint32_t g4_joint_size(void);
int64_t g4_now_ns(void);
void* g4_open(const char* port, int baud, unsigned hz);
void* g4_open_fresh(const char* port, int baud, unsigned hz);
void* g4_open_reader(const char* port);
int g4_transport_lost(void* handle);
void g4_close(void* handle);
int g4_snapshots(void* handle, G4Joint* out, int capacity);
int g4_snapshots_with_current_time(void* handle, G4Joint* out, int64_t* current_times, int capacity);
void g4_set_mit(void* handle, uint32_t bus, uint32_t id, float deg, float kp, float kd, float vel, float tor, int64_t deadline_ns);
int g4_set_mit_batch(void* handle, const float* commands, int count, int64_t deadline_ns);
int g4_renew_mit_batch(void* handle, const float* commands, int count, int64_t deadline_ns, int64_t previous_deadline_ns);
int g4_homing_mit_batch(void* handle, const float* commands, int count, uint32_t active_mask, int64_t deadline_ns);
void g4_set_servo(void* handle, uint32_t bus, uint32_t id, float deg, float spd, float cur, int64_t deadline_ns);
void g4_set_disabled_mask(void* handle, uint32_t mask);
void g4_release(void* handle, uint32_t bus, uint32_t id);
uint64_t g4_release_count(void* handle);
#ifdef __cplusplus
}
#endif
