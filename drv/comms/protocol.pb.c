/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.9.2 at Wed Jun 30 15:45:43 2021. */

#include "protocol.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

const int32_t Config_Callibration_acc_x_default = 0;
const int32_t Config_Callibration_acc_y_default = 0;
const int32_t Config_Callibration_acc_z_default = 0;
const float Config_Callibration_x_offset_default = 0;
const float Config_Callibration_y_offset_default = 0;
const float Config_Callibration_z_offset_default = 0;
const float Config_PidConfig_p_default = 1;
const float Config_PidConfig_d_default = 0.005;
const float Config_PidConfig_i_default = 0.001;
const float Config_PidConfig_max_i_default = 0.006;
const float Config_PidConfig_i_expo_default = 0;
const float Config_FootPadSettings_filter_rc_default = 0.05;
const int32_t Config_FootPadSettings_min_level_to_start_default = 3300;
const int32_t Config_FootPadSettings_min_level_to_continue_default = 2000;
const int32_t Config_FootPadSettings_shutoff_delay_ms_default = 100;
const float Config_BalancingConfig_balance_expo_default = 0.15;
const float Config_BalancingConfig_balance_angle_scaling_default = 15;
const int32_t Config_BalancingConfig_max_start_angle_steer_default = 15;
const int32_t Config_BalancingConfig_shutoff_angle_steer_default = 40;
const int32_t Config_BalancingConfig_shutoff_angle_drive_default = 14;
const int32_t Config_BalancingConfig_max_update_limiter_default = 300;
const float Config_BalancingConfig_output_lpf_hz_default = 100;
const int32_t Config_BalancingConfig_balance_d_param_limiter_default = 100;
const float Config_BalancingConfig_balance_d_param_lpf_hz_default = 50;
const uint32_t Config_BalancingConfig_global_gyro_lpf_default = 2u;
const float Config_BalancingConfig_imu_beta_default = 0.02;
const int32_t Config_BalancingConfig_expo_type_default = 0;
const float Config_BalancingConfig_usart_control_scaling_default = 0;
const int32_t Config_PusbackSettings_min_speed_erpm_default = 100000;
const float Config_PusbackSettings_push_raise_speed_deg_sec_default = 0.5;
const float Config_PusbackSettings_push_release_speed_deg_sec_default = 0.2;
const float Config_PusbackSettings_push_angle_default = 5;
const int32_t Config_Misc_log_type_default = 0;
const float Config_Misc_throttle_threshold_default = 0.75;
const float Config_Misc_throttle_rc_default = 0.05;
const float Config_Misc_duty_threshold_default = 0.75;
const float Config_Misc_duty_rc_default = 0.25;
const int32_t Config_Misc_erpm_threshold_default = 6000;
const float Config_Misc_erpm_rc_default = 0.25;
const float Config_Misc_low_volt_threshold_default = 45;
const float Config_Misc_volt_rc_default = 0.25;
const float Config_Misc_erpm_to_dist_const_default = 1;
const int32_t Config_LoadLift_start_current_default = 10;
const float Config_LoadLift_max_angle_default = 5;
const float Config_LoadLift_filter_rc_default = 0.05;
const float Config_LoadLift_multiplier_default = 0.1;
const float Config_LoadLift_ramp_deg_sec_default = 0.5;
const float Config_KeepRoll_multiplier_default = 0;
const float Config_KeepRoll_filter_rc_default = 0.05;
const float Config_KeepRoll_max_angle_default = 5;
const float Config_KeepRoll_raise_deg_sec_default = 0.5;
const float Config_KeepRoll_drop_deg_sec_default = 0.5;


const pb_field_t Config_fields[9] = {
    PB_FIELD(  1, MESSAGE , OPTIONAL, STATIC  , FIRST, Config, callibration, callibration, &Config_Callibration_fields),
    PB_FIELD(  2, MESSAGE , REQUIRED, STATIC  , OTHER, Config, balance_pid, callibration, &Config_PidConfig_fields),
    PB_FIELD(  3, MESSAGE , REQUIRED, STATIC  , OTHER, Config, foot_pad, balance_pid, &Config_FootPadSettings_fields),
    PB_FIELD(  4, MESSAGE , REQUIRED, STATIC  , OTHER, Config, balance_settings, foot_pad, &Config_BalancingConfig_fields),
    PB_FIELD(  5, MESSAGE , REQUIRED, STATIC  , OTHER, Config, misc, balance_settings, &Config_Misc_fields),
    PB_FIELD(  6, MESSAGE , OPTIONAL, STATIC  , OTHER, Config, pushback, misc, &Config_PusbackSettings_fields),
    PB_FIELD(  7, MESSAGE , OPTIONAL, STATIC  , OTHER, Config, load_lift, pushback, &Config_LoadLift_fields),
    PB_FIELD(  8, MESSAGE , OPTIONAL, STATIC  , OTHER, Config, keep_roll, load_lift, &Config_KeepRoll_fields),
    PB_LAST_FIELD
};

const pb_field_t Config_Callibration_fields[7] = {
    PB_FIELD(  1, INT32   , REQUIRED, STATIC  , FIRST, Config_Callibration, acc_x, acc_x, &Config_Callibration_acc_x_default),
    PB_FIELD(  2, INT32   , REQUIRED, STATIC  , OTHER, Config_Callibration, acc_y, acc_x, &Config_Callibration_acc_y_default),
    PB_FIELD(  3, INT32   , REQUIRED, STATIC  , OTHER, Config_Callibration, acc_z, acc_y, &Config_Callibration_acc_z_default),
    PB_FIELD(  4, FLOAT   , OPTIONAL, STATIC  , OTHER, Config_Callibration, x_offset, acc_z, &Config_Callibration_x_offset_default),
    PB_FIELD(  5, FLOAT   , OPTIONAL, STATIC  , OTHER, Config_Callibration, y_offset, x_offset, &Config_Callibration_y_offset_default),
    PB_FIELD(  6, FLOAT   , OPTIONAL, STATIC  , OTHER, Config_Callibration, z_offset, y_offset, &Config_Callibration_z_offset_default),
    PB_LAST_FIELD
};

const pb_field_t Config_PidConfig_fields[6] = {
    PB_FIELD(  1, FLOAT   , REQUIRED, STATIC  , FIRST, Config_PidConfig, p, p, &Config_PidConfig_p_default),
    PB_FIELD(  2, FLOAT   , REQUIRED, STATIC  , OTHER, Config_PidConfig, d, p, &Config_PidConfig_d_default),
    PB_FIELD(  3, FLOAT   , REQUIRED, STATIC  , OTHER, Config_PidConfig, i, d, &Config_PidConfig_i_default),
    PB_FIELD(  4, FLOAT   , REQUIRED, STATIC  , OTHER, Config_PidConfig, max_i, i, &Config_PidConfig_max_i_default),
    PB_FIELD( 13, FLOAT   , OPTIONAL, STATIC  , OTHER, Config_PidConfig, i_expo, max_i, &Config_PidConfig_i_expo_default),
    PB_LAST_FIELD
};

const pb_field_t Config_FootPadSettings_fields[5] = {
    PB_FIELD(  1, FLOAT   , REQUIRED, STATIC  , FIRST, Config_FootPadSettings, filter_rc, filter_rc, &Config_FootPadSettings_filter_rc_default),
    PB_FIELD(  2, INT32   , REQUIRED, STATIC  , OTHER, Config_FootPadSettings, min_level_to_start, filter_rc, &Config_FootPadSettings_min_level_to_start_default),
    PB_FIELD(  3, INT32   , REQUIRED, STATIC  , OTHER, Config_FootPadSettings, min_level_to_continue, min_level_to_start, &Config_FootPadSettings_min_level_to_continue_default),
    PB_FIELD(  4, INT32   , REQUIRED, STATIC  , OTHER, Config_FootPadSettings, shutoff_delay_ms, min_level_to_continue, &Config_FootPadSettings_shutoff_delay_ms_default),
    PB_LAST_FIELD
};

const pb_field_t Config_BalancingConfig_fields[14] = {
    PB_FIELD(  1, FLOAT   , REQUIRED, STATIC  , FIRST, Config_BalancingConfig, balance_expo, balance_expo, &Config_BalancingConfig_balance_expo_default),
    PB_FIELD(  2, FLOAT   , REQUIRED, STATIC  , OTHER, Config_BalancingConfig, balance_angle_scaling, balance_expo, &Config_BalancingConfig_balance_angle_scaling_default),
    PB_FIELD(  3, INT32   , REQUIRED, STATIC  , OTHER, Config_BalancingConfig, max_start_angle_steer, balance_angle_scaling, &Config_BalancingConfig_max_start_angle_steer_default),
    PB_FIELD(  4, INT32   , REQUIRED, STATIC  , OTHER, Config_BalancingConfig, shutoff_angle_steer, max_start_angle_steer, &Config_BalancingConfig_shutoff_angle_steer_default),
    PB_FIELD(  5, INT32   , REQUIRED, STATIC  , OTHER, Config_BalancingConfig, shutoff_angle_drive, shutoff_angle_steer, &Config_BalancingConfig_shutoff_angle_drive_default),
    PB_FIELD(  6, INT32   , REQUIRED, STATIC  , OTHER, Config_BalancingConfig, max_update_limiter, shutoff_angle_drive, &Config_BalancingConfig_max_update_limiter_default),
    PB_FIELD(  7, FLOAT   , REQUIRED, STATIC  , OTHER, Config_BalancingConfig, output_lpf_hz, max_update_limiter, &Config_BalancingConfig_output_lpf_hz_default),
    PB_FIELD(  8, INT32   , REQUIRED, STATIC  , OTHER, Config_BalancingConfig, balance_d_param_limiter, output_lpf_hz, &Config_BalancingConfig_balance_d_param_limiter_default),
    PB_FIELD(  9, FLOAT   , REQUIRED, STATIC  , OTHER, Config_BalancingConfig, balance_d_param_lpf_hz, balance_d_param_limiter, &Config_BalancingConfig_balance_d_param_lpf_hz_default),
    PB_FIELD( 10, UINT32  , REQUIRED, STATIC  , OTHER, Config_BalancingConfig, global_gyro_lpf, balance_d_param_lpf_hz, &Config_BalancingConfig_global_gyro_lpf_default),
    PB_FIELD( 11, FLOAT   , OPTIONAL, STATIC  , OTHER, Config_BalancingConfig, imu_beta, global_gyro_lpf, &Config_BalancingConfig_imu_beta_default),
    PB_FIELD( 12, INT32   , OPTIONAL, STATIC  , OTHER, Config_BalancingConfig, expo_type, imu_beta, &Config_BalancingConfig_expo_type_default),
    PB_FIELD( 13, FLOAT   , OPTIONAL, STATIC  , OTHER, Config_BalancingConfig, usart_control_scaling, expo_type, &Config_BalancingConfig_usart_control_scaling_default),
    PB_LAST_FIELD
};

const pb_field_t Config_PusbackSettings_fields[5] = {
    PB_FIELD(  1, INT32   , REQUIRED, STATIC  , FIRST, Config_PusbackSettings, min_speed_erpm, min_speed_erpm, &Config_PusbackSettings_min_speed_erpm_default),
    PB_FIELD(  3, FLOAT   , REQUIRED, STATIC  , OTHER, Config_PusbackSettings, push_raise_speed_deg_sec, min_speed_erpm, &Config_PusbackSettings_push_raise_speed_deg_sec_default),
    PB_FIELD(  4, FLOAT   , REQUIRED, STATIC  , OTHER, Config_PusbackSettings, push_release_speed_deg_sec, push_raise_speed_deg_sec, &Config_PusbackSettings_push_release_speed_deg_sec_default),
    PB_FIELD(  5, FLOAT   , OPTIONAL, STATIC  , OTHER, Config_PusbackSettings, push_angle, push_release_speed_deg_sec, &Config_PusbackSettings_push_angle_default),
    PB_LAST_FIELD
};

const pb_field_t Config_Misc_fields[11] = {
    PB_FIELD(  1, INT32   , OPTIONAL, STATIC  , FIRST, Config_Misc, log_type, log_type, &Config_Misc_log_type_default),
    PB_FIELD(  6, FLOAT   , REQUIRED, STATIC  , OTHER, Config_Misc, throttle_rc, log_type, &Config_Misc_throttle_rc_default),
    PB_FIELD(  7, FLOAT   , REQUIRED, STATIC  , OTHER, Config_Misc, throttle_threshold, throttle_rc, &Config_Misc_throttle_threshold_default),
    PB_FIELD(  8, FLOAT   , OPTIONAL, STATIC  , OTHER, Config_Misc, duty_threshold, throttle_threshold, &Config_Misc_duty_threshold_default),
    PB_FIELD(  9, FLOAT   , OPTIONAL, STATIC  , OTHER, Config_Misc, duty_rc, duty_threshold, &Config_Misc_duty_rc_default),
    PB_FIELD( 10, INT32   , OPTIONAL, STATIC  , OTHER, Config_Misc, erpm_threshold, duty_rc, &Config_Misc_erpm_threshold_default),
    PB_FIELD( 11, FLOAT   , OPTIONAL, STATIC  , OTHER, Config_Misc, erpm_rc, erpm_threshold, &Config_Misc_erpm_rc_default),
    PB_FIELD( 12, FLOAT   , OPTIONAL, STATIC  , OTHER, Config_Misc, low_volt_threshold, erpm_rc, &Config_Misc_low_volt_threshold_default),
    PB_FIELD( 13, FLOAT   , OPTIONAL, STATIC  , OTHER, Config_Misc, volt_rc, low_volt_threshold, &Config_Misc_volt_rc_default),
    PB_FIELD( 15, FLOAT   , OPTIONAL, STATIC  , OTHER, Config_Misc, erpm_to_dist_const, volt_rc, &Config_Misc_erpm_to_dist_const_default),
    PB_LAST_FIELD
};

const pb_field_t Config_LoadLift_fields[6] = {
    PB_FIELD(  1, INT32   , REQUIRED, STATIC  , FIRST, Config_LoadLift, start_current, start_current, &Config_LoadLift_start_current_default),
    PB_FIELD(  2, FLOAT   , REQUIRED, STATIC  , OTHER, Config_LoadLift, max_angle, start_current, &Config_LoadLift_max_angle_default),
    PB_FIELD(  3, FLOAT   , REQUIRED, STATIC  , OTHER, Config_LoadLift, filter_rc, max_angle, &Config_LoadLift_filter_rc_default),
    PB_FIELD(  4, FLOAT   , REQUIRED, STATIC  , OTHER, Config_LoadLift, multiplier, filter_rc, &Config_LoadLift_multiplier_default),
    PB_FIELD(  5, FLOAT   , REQUIRED, STATIC  , OTHER, Config_LoadLift, ramp_deg_sec, multiplier, &Config_LoadLift_ramp_deg_sec_default),
    PB_LAST_FIELD
};

const pb_field_t Config_KeepRoll_fields[6] = {
    PB_FIELD(  1, FLOAT   , REQUIRED, STATIC  , FIRST, Config_KeepRoll, multiplier, multiplier, &Config_KeepRoll_multiplier_default),
    PB_FIELD(  2, FLOAT   , REQUIRED, STATIC  , OTHER, Config_KeepRoll, filter_rc, multiplier, &Config_KeepRoll_filter_rc_default),
    PB_FIELD(  3, FLOAT   , REQUIRED, STATIC  , OTHER, Config_KeepRoll, max_angle, filter_rc, &Config_KeepRoll_max_angle_default),
    PB_FIELD(  4, FLOAT   , REQUIRED, STATIC  , OTHER, Config_KeepRoll, raise_deg_sec, max_angle, &Config_KeepRoll_raise_deg_sec_default),
    PB_FIELD(  5, FLOAT   , REQUIRED, STATIC  , OTHER, Config_KeepRoll, drop_deg_sec, raise_deg_sec, &Config_KeepRoll_drop_deg_sec_default),
    PB_LAST_FIELD
};

const pb_field_t Stats_fields[13] = {
    PB_FIELD(  1, FLOAT   , REQUIRED, STATIC  , FIRST, Stats, batt_voltage, batt_voltage, 0),
    PB_FIELD(  2, FLOAT   , REQUIRED, STATIC  , OTHER, Stats, batt_current, batt_voltage, 0),
    PB_FIELD(  3, FLOAT   , REQUIRED, STATIC  , OTHER, Stats, motor_current, batt_current, 0),
    PB_FIELD(  5, FLOAT   , REQUIRED, STATIC  , OTHER, Stats, speed, motor_current, 0),
    PB_FIELD(  6, FLOAT   , REQUIRED, STATIC  , OTHER, Stats, distance_traveled, speed, 0),
    PB_FIELD(  7, FLOAT   , REQUIRED, STATIC  , OTHER, Stats, drive_angle, distance_traveled, 0),
    PB_FIELD(  8, FLOAT   , REQUIRED, STATIC  , OTHER, Stats, stear_angle, drive_angle, 0),
    PB_FIELD(  9, UINT32  , REQUIRED, STATIC  , OTHER, Stats, pad_pressure1, stear_angle, 0),
    PB_FIELD( 10, UINT32  , REQUIRED, STATIC  , OTHER, Stats, pad_pressure2, pad_pressure1, 0),
    PB_FIELD( 11, FLOAT   , REQUIRED, STATIC  , OTHER, Stats, motor_duty, pad_pressure2, 0),
    PB_FIELD( 12, FLOAT   , REQUIRED, STATIC  , OTHER, Stats, esc_temp, motor_duty, 0),
    PB_FIELD( 13, FLOAT   , REQUIRED, STATIC  , OTHER, Stats, motor_temp, esc_temp, 0),
    PB_LAST_FIELD
};




/* Check that field information fits in pb_field_t */
#if !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_32BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in 8 or 16 bit
 * field descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(Config, callibration) < 65536 && pb_membersize(Config, balance_pid) < 65536 && pb_membersize(Config, foot_pad) < 65536 && pb_membersize(Config, balance_settings) < 65536 && pb_membersize(Config, misc) < 65536 && pb_membersize(Config, pushback) < 65536 && pb_membersize(Config, load_lift) < 65536 && pb_membersize(Config, keep_roll) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_Config_Config_Callibration_Config_PidConfig_Config_FootPadSettings_Config_BalancingConfig_Config_PusbackSettings_Config_Misc_Config_LoadLift_Config_KeepRoll_Stats)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_16BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in the default
 * 8 bit descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(Config, callibration) < 256 && pb_membersize(Config, balance_pid) < 256 && pb_membersize(Config, foot_pad) < 256 && pb_membersize(Config, balance_settings) < 256 && pb_membersize(Config, misc) < 256 && pb_membersize(Config, pushback) < 256 && pb_membersize(Config, load_lift) < 256 && pb_membersize(Config, keep_roll) < 256), YOU_MUST_DEFINE_PB_FIELD_16BIT_FOR_MESSAGES_Config_Config_Callibration_Config_PidConfig_Config_FootPadSettings_Config_BalancingConfig_Config_PusbackSettings_Config_Misc_Config_LoadLift_Config_KeepRoll_Stats)
#endif


/* @@protoc_insertion_point(eof) */
