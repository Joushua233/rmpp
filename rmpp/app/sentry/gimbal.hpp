#pragma once

#include "motor/MF9025.hpp"
#include "motor/DMJ6006.hpp"
#include "motor/GM6020.hpp"
#include "motor/DM4310.hpp"
#include "motor/EL05.hpp"
#include "module/gimbal/Gimbal_DualYaw.hpp"
#include "imu.hpp"

// 用于Ozone调参
// #define yaw1_pid angle_pid
// #define yaw1 motor
//
// #define yaw2_speed_pid speed_pid
// #define yaw2_angle_pid angle_pid
// #define yaw2 motor
//
// #define pitch_speed_pid speed_pid
// #define pitch_angle_pid angle_pid
// #define pitch motor

// 大yaw电机：保留主控角度 PD 和速度前馈，DMJ6006 使用 MIT 纯扭矩模式。
// 沿用原 MF9025 的增益表达方式，乘扭矩系数后作为扭矩 PD 参数。
// DMJ6006 初调：降低角度、速度增益，限制输出为 1.44 Nm，实机稳定后再逐步调整。
inline PID::config_t yaw1_pid = {
    .kp = (1.5f * A) * MF9025::Kt / (1.8 * deg),
    .kd = (4.5f * A) * MF9025::Kt / (18 * deg_s),
    .max_out = (4.5f * A) * MF9025::Kt,
    .fc = 10 * Hz,
};
// 与调试助手一致：CAN1 1 Mbps、MIT 模式，电调 CAN Timeout = 2000（100 ms）。
// 电调通讯保护在上位机写入，独立于主控的反馈断联检测。
// 更换电机后需重新执行原有大 yaw 零位标定，Flash 中的旧 yaw_offset 不再适用。
inline DMJ6006 yaw1({
    .can_port = 1,
    .master_id = 0x10,
    .slave_id = 0x11, // MIT 模式，无 ID 偏移
    .reduction = 1.0f, // 协议反馈输出轴位置，不能再除以内部 6:1 减速比
    .is_invert = true,
    .control_mode = Motor::ANGLE_MODE,
    .pid_out_type = Motor::TORQUE_OUTPUT,
    .angle_pid_config = &yaw1_pid,
}, {
    .p_max = 3.14159f, // 与调试助手 PMAX 一致，单位 rad
    .v_max = 45.0f, // 与调试助手 VMAX 一致，单位 rad/s
    .t_max = 12.0f, // 与调试助手 TMAX 一致，单位 Nm；实际输出仍由 PID 限幅
});

// 小yaw电机
inline PID::config_t yaw2_speed_pid = {
    .kp = (25 * V) / (300 * deg_s),
    .kd = (25 * V) / (30000 * deg_ss),
    .max_out = 3 * A,
    .fc = 10 * Hz,
};
inline PID::config_t yaw2_angle_pid = {
    .kp = 30 * default_unit,
    .max_out = 720 * deg_s,
};
inline GM6020 yaw2({
    .can_port = 1,
    .master_id = 0x205,
    .slave_id = 0x1FF,
    .offset = 0 * deg,
    .is_limit = true,
    .limit_min = -40 * deg,
    .limit_max = 40 * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &yaw2_speed_pid,
    .angle_pid_config = &yaw2_angle_pid,
});

// pitch电机
inline PID::config_t pitch_speed_pid = {
    .kp = (3 * Nm) / (150 * deg_s),
    .max_out = 0.5f * Nm, // EL05 首次联调限扭，确认方向后再逐步提高
    .fc = 10 * Hz,
};
inline PID::config_t pitch_angle_pid = {
    .kp = 20 * default_unit,
    .max_out = 720 * deg_s,
};
// EL05 出厂私有协议 ID：电机 0x7F，主机 0xFD；CAN2 1 Mbps 扩展帧。
// 主控保留原角度-速度串级 PID，EL05 运控模式内部 Kp/Kd 置 0，只接收扭矩。
inline EL05 pitch({
    .can_port = 2,
    .master_id = EL05::DEFAULT_HOST_ID,
    .slave_id = EL05::DEFAULT_MOTOR_ID,
    .is_invert = false, // EL05 反馈方向与云台 IMU pitch 方向一致
    .offset = 0 * deg, // 新电机绝对编码器需实机确认 pitch 机械零位
    .is_limit = true,
    .limit_min = -22 * deg,
    .limit_max = 15 * deg,
    .control_mode = Motor::ANGLE_SPEED_MODE,
    .pid_out_type = Motor::TORQUE_OUTPUT,
    .speed_pid_config = &pitch_speed_pid,
    .angle_pid_config = &pitch_angle_pid,
}, {
    .is_torque_invert = true,
});

// 原 DM4310 pitch 配置保留；需切回时注释上面 EL05，再取消下面这段注释。
// inline DM4310 pitch({
//     .can_port = 2,
//     .master_id = 0x10,
//     .slave_id = 0x11,
//     .is_invert = true,
//     .offset = -72 * deg,
//     .is_limit = true,
//     .limit_min = -22 * deg,
//     .limit_max = 15 * deg,
//     .control_mode = Motor::ANGLE_SPEED_MODE,
//     .pid_out_type = Motor::TORQUE_OUTPUT,
//     .speed_pid_config = &pitch_speed_pid,
//     .angle_pid_config = &pitch_angle_pid,
// });

// 云台
inline Gimbal_DualYaw gimbal(imu, {yaw1, yaw2, pitch});
