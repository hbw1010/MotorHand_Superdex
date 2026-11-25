#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef struct G4Joint {
    uint32_t bus, id, valid, error;
    float angle_deg, current_a, temp_c, speed_rpm, hz;
    uint64_t samples;
    int64_t received_ns;
    uint32_t board_timestamp_us, max_board_gap_us, max_host_gap_us, tx_failures;
} G4Joint;

void* g4_open(const char* port, int baud, unsigned hz);
void* g4_open_fresh(const char* port, int baud, unsigned hz);
void* g4_open_axes(const char* port, int baud, unsigned hz, int axis_count);
void* g4_open_fresh_axes(const char* port, int baud, unsigned hz, int axis_count);
void* g4_open_reader_axes(const char* port, int axis_count);
int g4_transport_lost(void* handle);
void g4_close(void* handle);
int g4_snapshots_with_current_time(void* handle, G4Joint* out, int64_t* current_times, int capacity);
int g4_set_mit_batch(void* handle, const float* commands, int count, int64_t deadline_ns);
int g4_renew_mit_batch(void* handle, const float* commands, int count, int64_t deadline_ns, int64_t previous_deadline_ns);
int g4_homing_mit_batch(void* handle, const float* commands, int count, uint32_t active_mask, int64_t deadline_ns);
void g4_set_disabled_mask(void* handle, uint32_t mask);
void g4_release(void* handle, uint32_t bus, uint32_t id);
#ifdef __cplusplus
}
#endif
