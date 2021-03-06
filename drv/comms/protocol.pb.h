/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.9.2 at Thu Jun 11 15:31:08 2020. */

#ifndef PB_PROTOCOL_PB_H_INCLUDED
#define PB_PROTOCOL_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _RequestId {
    RequestId_MSG_NONE = 0,
    RequestId_READ_CONFIG = 1,
    RequestId_WRITE_CONFIG = 2,
    RequestId_GET_STATS = 3,
    RequestId_CALLIBRATE_ACC = 4,
    RequestId_SAVE_CONFIG = 5,
    RequestId_GET_DEBUG_BUFFER = 6,
    RequestId_SET_DEBUG_STREAM_ID = 7,
    RequestId_TOGGLE_PASSTHROUGH = 8
} RequestId;
#define _RequestId_MIN RequestId_MSG_NONE
#define _RequestId_MAX RequestId_TOGGLE_PASSTHROUGH
#define _RequestId_ARRAYSIZE ((RequestId)(RequestId_TOGGLE_PASSTHROUGH+1))

typedef enum _ReplyId {
    ReplyId_NO_REPLY = 0,
    ReplyId_GENERIC_OK = 1,
    ReplyId_GENERIC_FAIL = 2,
    ReplyId_STATS = 3,
    ReplyId_CONFIG = 4,
    ReplyId_CRC_MISMATCH = 5,
    ReplyId_DEBUG_BUFFER = 6
} ReplyId;
#define _ReplyId_MIN ReplyId_NO_REPLY
#define _ReplyId_MAX ReplyId_DEBUG_BUFFER
#define _ReplyId_ARRAYSIZE ((ReplyId)(ReplyId_DEBUG_BUFFER+1))

/* Struct definitions */
typedef struct _Config_BalancingConfig {
    float balance_expo;
    float balance_angle_scaling;
    int32_t max_start_angle_steer;
    int32_t shutoff_angle_steer;
    int32_t shutoff_angle_drive;
    int32_t max_update_limiter;
    float output_lpf_hz;
    int32_t balance_d_param_limiter;
    float balance_d_param_lpf_hz;
    uint32_t global_gyro_lpf;
    bool has_imu_beta;
    float imu_beta;
    bool has_expo_type;
    int32_t expo_type;
    bool has_usart_control_scaling;
    float usart_control_scaling;
/* @@protoc_insertion_point(struct:Config_BalancingConfig) */
} Config_BalancingConfig;

typedef struct _Config_Callibration {
    int32_t acc_x;
    int32_t acc_y;
    int32_t acc_z;
    bool has_x_offset;
    float x_offset;
    bool has_y_offset;
    float y_offset;
    bool has_z_offset;
    float z_offset;
/* @@protoc_insertion_point(struct:Config_Callibration) */
} Config_Callibration;

typedef struct _Config_FootPadSettings {
    float filter_rc;
    int32_t min_level_to_start;
    int32_t min_level_to_continue;
    int32_t shutoff_delay_ms;
/* @@protoc_insertion_point(struct:Config_FootPadSettings) */
} Config_FootPadSettings;

typedef struct _Config_Misc {
    bool has_log_type;
    int32_t log_type;
    float throttle_rc;
    float throttle_threshold;
    bool has_duty_threshold;
    float duty_threshold;
    bool has_duty_rc;
    float duty_rc;
    bool has_erpm_threshold;
    int32_t erpm_threshold;
    bool has_erpm_rc;
    float erpm_rc;
    bool has_low_volt_threshold;
    float low_volt_threshold;
    bool has_volt_rc;
    float volt_rc;
    bool has_speed_input_mixin;
    float speed_input_mixin;
    bool has_erpm_to_dist_const;
    float erpm_to_dist_const;
/* @@protoc_insertion_point(struct:Config_Misc) */
} Config_Misc;

typedef struct _Config_PidConfig {
    float p;
    float d;
    float i;
    float max_i;
    bool has_i_expo;
    float i_expo;
/* @@protoc_insertion_point(struct:Config_PidConfig) */
} Config_PidConfig;

typedef struct _Config_PusbackSettings {
    int32_t min_speed_erpm;
    int32_t push_angle;
    float push_raise_speed_deg_sec;
    float push_release_speed_deg_sec;
/* @@protoc_insertion_point(struct:Config_PusbackSettings) */
} Config_PusbackSettings;

typedef struct _Stats {
    float batt_voltage;
    float batt_current;
    float motor_current;
    float speed;
    float distance_traveled;
    float drive_angle;
    float stear_angle;
    uint32_t pad_pressure1;
    uint32_t pad_pressure2;
    float motor_duty;
    float esc_temp;
    float motor_temp;
/* @@protoc_insertion_point(struct:Stats) */
} Stats;

typedef struct _Config {
    bool has_callibration;
    Config_Callibration callibration;
    Config_PidConfig balance_pid;
    Config_FootPadSettings foot_pad;
    Config_BalancingConfig balance_settings;
    Config_Misc misc;
    bool has_pushback;
    Config_PusbackSettings pushback;
/* @@protoc_insertion_point(struct:Config) */
} Config;

/* Default values for struct fields */
extern const int32_t Config_Callibration_acc_x_default;
extern const int32_t Config_Callibration_acc_y_default;
extern const int32_t Config_Callibration_acc_z_default;
extern const float Config_Callibration_x_offset_default;
extern const float Config_Callibration_y_offset_default;
extern const float Config_Callibration_z_offset_default;
extern const float Config_PidConfig_p_default;
extern const float Config_PidConfig_d_default;
extern const float Config_PidConfig_i_default;
extern const float Config_PidConfig_max_i_default;
extern const float Config_PidConfig_i_expo_default;
extern const float Config_FootPadSettings_filter_rc_default;
extern const int32_t Config_FootPadSettings_min_level_to_start_default;
extern const int32_t Config_FootPadSettings_min_level_to_continue_default;
extern const int32_t Config_FootPadSettings_shutoff_delay_ms_default;
extern const float Config_BalancingConfig_balance_expo_default;
extern const float Config_BalancingConfig_balance_angle_scaling_default;
extern const int32_t Config_BalancingConfig_max_start_angle_steer_default;
extern const int32_t Config_BalancingConfig_shutoff_angle_steer_default;
extern const int32_t Config_BalancingConfig_shutoff_angle_drive_default;
extern const int32_t Config_BalancingConfig_max_update_limiter_default;
extern const float Config_BalancingConfig_output_lpf_hz_default;
extern const int32_t Config_BalancingConfig_balance_d_param_limiter_default;
extern const float Config_BalancingConfig_balance_d_param_lpf_hz_default;
extern const uint32_t Config_BalancingConfig_global_gyro_lpf_default;
extern const float Config_BalancingConfig_imu_beta_default;
extern const int32_t Config_BalancingConfig_expo_type_default;
extern const float Config_BalancingConfig_usart_control_scaling_default;
extern const int32_t Config_PusbackSettings_min_speed_erpm_default;
extern const int32_t Config_PusbackSettings_push_angle_default;
extern const float Config_PusbackSettings_push_raise_speed_deg_sec_default;
extern const float Config_PusbackSettings_push_release_speed_deg_sec_default;
extern const int32_t Config_Misc_log_type_default;
extern const float Config_Misc_throttle_threshold_default;
extern const float Config_Misc_throttle_rc_default;
extern const float Config_Misc_duty_threshold_default;
extern const float Config_Misc_duty_rc_default;
extern const int32_t Config_Misc_erpm_threshold_default;
extern const float Config_Misc_erpm_rc_default;
extern const float Config_Misc_low_volt_threshold_default;
extern const float Config_Misc_volt_rc_default;
extern const float Config_Misc_speed_input_mixin_default;
extern const float Config_Misc_erpm_to_dist_const_default;

/* Initializer values for message structs */
#define Config_init_default                      {false, Config_Callibration_init_default, Config_PidConfig_init_default, Config_FootPadSettings_init_default, Config_BalancingConfig_init_default, Config_Misc_init_default, false, Config_PusbackSettings_init_default}
#define Config_Callibration_init_default         {0, 0, 0, false, 0, false, 0, false, 0}
#define Config_PidConfig_init_default            {1, 0.005, 0.001, 0.006, false, 0}
#define Config_FootPadSettings_init_default      {0.05, 3300, 2000, 100}
#define Config_BalancingConfig_init_default      {0.15, 15, 15, 40, 14, 300, 100, 100, 50, 2u, false, 0.02, false, 0, false, 0}
#define Config_PusbackSettings_init_default      {1000, 5, 0.5, 0.2}
#define Config_Misc_init_default                 {false, 0, 0.05, 0.75, false, 0.75, false, 0.25, false, 6000, false, 0.25, false, 45, false, 0.25, false, 0, false, 1}
#define Stats_init_default                       {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define Config_init_zero                         {false, Config_Callibration_init_zero, Config_PidConfig_init_zero, Config_FootPadSettings_init_zero, Config_BalancingConfig_init_zero, Config_Misc_init_zero, false, Config_PusbackSettings_init_zero}
#define Config_Callibration_init_zero            {0, 0, 0, false, 0, false, 0, false, 0}
#define Config_PidConfig_init_zero               {0, 0, 0, 0, false, 0}
#define Config_FootPadSettings_init_zero         {0, 0, 0, 0}
#define Config_BalancingConfig_init_zero         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, 0, false, 0, false, 0}
#define Config_PusbackSettings_init_zero         {0, 0, 0, 0}
#define Config_Misc_init_zero                    {false, 0, 0, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}
#define Stats_init_zero                          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define Config_BalancingConfig_balance_expo_tag  1
#define Config_BalancingConfig_balance_angle_scaling_tag 2
#define Config_BalancingConfig_max_start_angle_steer_tag 3
#define Config_BalancingConfig_shutoff_angle_steer_tag 4
#define Config_BalancingConfig_shutoff_angle_drive_tag 5
#define Config_BalancingConfig_max_update_limiter_tag 6
#define Config_BalancingConfig_output_lpf_hz_tag 7
#define Config_BalancingConfig_balance_d_param_limiter_tag 8
#define Config_BalancingConfig_balance_d_param_lpf_hz_tag 9
#define Config_BalancingConfig_global_gyro_lpf_tag 10
#define Config_BalancingConfig_imu_beta_tag      11
#define Config_BalancingConfig_expo_type_tag     12
#define Config_BalancingConfig_usart_control_scaling_tag 13
#define Config_Callibration_acc_x_tag            1
#define Config_Callibration_acc_y_tag            2
#define Config_Callibration_acc_z_tag            3
#define Config_Callibration_x_offset_tag         4
#define Config_Callibration_y_offset_tag         5
#define Config_Callibration_z_offset_tag         6
#define Config_FootPadSettings_filter_rc_tag     1
#define Config_FootPadSettings_min_level_to_start_tag 2
#define Config_FootPadSettings_min_level_to_continue_tag 3
#define Config_FootPadSettings_shutoff_delay_ms_tag 4
#define Config_Misc_log_type_tag                 1
#define Config_Misc_throttle_threshold_tag       7
#define Config_Misc_throttle_rc_tag              6
#define Config_Misc_duty_threshold_tag           8
#define Config_Misc_duty_rc_tag                  9
#define Config_Misc_erpm_threshold_tag           10
#define Config_Misc_erpm_rc_tag                  11
#define Config_Misc_low_volt_threshold_tag       12
#define Config_Misc_volt_rc_tag                  13
#define Config_Misc_speed_input_mixin_tag        14
#define Config_Misc_erpm_to_dist_const_tag       15
#define Config_PidConfig_p_tag                   1
#define Config_PidConfig_d_tag                   2
#define Config_PidConfig_i_tag                   3
#define Config_PidConfig_max_i_tag               4
#define Config_PidConfig_i_expo_tag              13
#define Config_PusbackSettings_min_speed_erpm_tag 1
#define Config_PusbackSettings_push_angle_tag    2
#define Config_PusbackSettings_push_raise_speed_deg_sec_tag 3
#define Config_PusbackSettings_push_release_speed_deg_sec_tag 4
#define Stats_batt_voltage_tag                   1
#define Stats_batt_current_tag                   2
#define Stats_motor_current_tag                  3
#define Stats_speed_tag                          5
#define Stats_distance_traveled_tag              6
#define Stats_drive_angle_tag                    7
#define Stats_stear_angle_tag                    8
#define Stats_pad_pressure1_tag                  9
#define Stats_pad_pressure2_tag                  10
#define Stats_motor_duty_tag                     11
#define Stats_esc_temp_tag                       12
#define Stats_motor_temp_tag                     13
#define Config_callibration_tag                  1
#define Config_balance_pid_tag                   2
#define Config_foot_pad_tag                      3
#define Config_balance_settings_tag              4
#define Config_misc_tag                          5
#define Config_pushback_tag                      6

/* Struct field encoding specification for nanopb */
extern const pb_field_t Config_fields[7];
extern const pb_field_t Config_Callibration_fields[7];
extern const pb_field_t Config_PidConfig_fields[6];
extern const pb_field_t Config_FootPadSettings_fields[5];
extern const pb_field_t Config_BalancingConfig_fields[14];
extern const pb_field_t Config_PusbackSettings_fields[5];
extern const pb_field_t Config_Misc_fields[12];
extern const pb_field_t Stats_fields[13];

/* Maximum encoded size of messages (where known) */
#define Config_size                              324
#define Config_Callibration_size                 48
#define Config_PidConfig_size                    25
#define Config_FootPadSettings_size              38
#define Config_BalancingConfig_size              102
#define Config_PusbackSettings_size              32
#define Config_Misc_size                         67
#define Stats_size                               62

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define PROTOCOL_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
