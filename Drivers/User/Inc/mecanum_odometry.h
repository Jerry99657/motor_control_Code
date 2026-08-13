#ifndef MECANUM_ODOMETRY_H
#define MECANUM_ODOMETRY_H

#include <stdint.h>

/* Encoder-only chassis odometry.
 *
 * Coordinate convention follows mecanum.h:
 *   +X       chassis forward
 *   +Y       chassis right
 *   +heading clockwise
 *
 * The pose origin is arbitrary and can be reset while the chassis is idle.
 * No IMU measurement or per-wheel calibration is used in this stage.
 */
typedef struct
{
    float x_mm;
    float y_mm;
    float heading_deg;
    float heading_total_deg;
    float body_forward_total_mm;
    float body_right_total_mm;
    float body_vx_mm_s;
    float body_vy_mm_s;
    float body_wz_deg_s;
    float travel_distance_mm;
    int32_t wheel_delta_counts[4];
    uint32_t update_count;
    uint32_t rejected_count;
    uint32_t timing_gap_count;
    uint32_t reset_sequence;
    uint32_t last_update_tick;
    uint8_t encoder_ready_mask;
    uint8_t initialized;
    uint8_t last_sample_accepted;
} MecanumOdometrySnapshot;

void MecanumOdometry_Init(void);
void MecanumOdometry_Update10ms(void);
void MecanumOdometry_RequestReset(void);
void MecanumOdometry_GetSnapshot(MecanumOdometrySnapshot *snapshot);

#endif /* MECANUM_ODOMETRY_H */
