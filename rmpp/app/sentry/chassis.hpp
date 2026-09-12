#pragma once

#include "motor/M3508.hpp"
#include "motor/GM6020.hpp"
#include "module/chassis/Chassis_DualSteer.hpp"

// 电机参数
static constexpr float REDUCTION = 14.0f; // 轮电机总减速比暂用 14:1
static constexpr UnitFloat<Nm_A> Kt = M3508::Kt / M3508::REDUCTION * REDUCTION;
static constexpr bool IS_WHEEL_INVERT = false;
static constexpr bool IS_SERVO_INVERT = true;

// 125 Hz 回传周期为 8 ms，允许连续丢失约 5 帧。控制循环仍为 1 ms。
static constexpr UnitFloat CHASSIS_FEEDBACK_TIMEOUT = 40 * ms;

// 底盘参数：前后、左右舵轴间距均为 360 mm，中心到舵轴距离为 180 * sqrt(2) mm。
static constexpr UnitFloat CHASSIS_RADIUS = 180.0f * 1.41421356237f * mm;
static constexpr UnitFloat WHEEL_RADIUS = 60.0f * mm; // 驱动轮直径 120 mm
static constexpr UnitFloat<N> MAX_F = M3508::MAX_CURRENT * Kt / WHEEL_RADIUS * 4;

// 轮电机PID参数
inline PID::config_t wheel_pid = {
    .kp = (20 * A) / (360 * rpm),
    .ki = (20 * A) / (15 * deg),
    .max_out = 20 * A,
};

// 舵电机PID参数
inline PID::config_t servo_pid = {
    .kp = (7 * V) / (40 * deg), // 四舵共用，比例增益减半以减轻转舵振荡
    .max_out = 25 * V,
};

// 底盘PID参数
inline PID::config_t vxyz_pid = {
    .kp = MAX_F / (1 * (m / s)),
    .ki = MAX_F / (10 * cm),
    .max_i = MAX_F,
    .max_out = MAX_F,
};

// 轮电机（左前、左后、右后、右前）
inline M3508 w1({
    .can_port = 1,
    .master_id = 0x201,
    .slave_id = 0x200,
    .timeout = CHASSIS_FEEDBACK_TIMEOUT,
    .reduction = REDUCTION,
    .Kt = Kt,
    .is_invert = IS_WHEEL_INVERT,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &wheel_pid,
});
inline M3508 w2({
    .can_port = 1,
    .master_id = 0x202,
    .slave_id = 0x200,
    .timeout = CHASSIS_FEEDBACK_TIMEOUT,
    .reduction = REDUCTION,
    .Kt = Kt,
    .is_invert = IS_WHEEL_INVERT,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &wheel_pid,
});
inline M3508 w3({
    .can_port = 1,
    .master_id = 0x203,
    .slave_id = 0x200,
    .timeout = CHASSIS_FEEDBACK_TIMEOUT,
    .reduction = REDUCTION,
    .Kt = Kt,
    .is_invert = IS_WHEEL_INVERT,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &wheel_pid,
});
inline M3508 w4({
    .can_port = 1,
    .master_id = 0x204,
    .slave_id = 0x200,
    .timeout = CHASSIS_FEEDBACK_TIMEOUT,
    .reduction = REDUCTION,
    .Kt = Kt,
    .is_invert = IS_WHEEL_INVERT,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &wheel_pid,
});

// 舵电机（左前、左后、右后、右前）
// 舵电机直驱，传动比 1:1；offset 为实测轮子朝前时的 raw.angle，无需随 is_invert 取反。
inline GM6020 s1({
    .can_port = 1,
    .master_id = 0x207,
    .slave_id = 0x1FF,
    .timeout = CHASSIS_FEEDBACK_TIMEOUT,
    .reduction = 1.0f,
    .is_invert = IS_SERVO_INVERT,
    .offset = -133.5f * deg, // 左前零位
    .control_mode = Motor::ANGLE_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .angle_pid_config = &servo_pid,
});
inline GM6020 s2({
    .can_port = 1,
    .master_id = 0x208,
    .slave_id = 0x1FF,
    .timeout = CHASSIS_FEEDBACK_TIMEOUT,
    .reduction = 1.0f,
    .is_invert = IS_SERVO_INVERT,
    .offset = 105.7f * deg, // 左后零位
    .control_mode = Motor::ANGLE_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .angle_pid_config = &servo_pid,
});
inline GM6020 s3({
    .can_port = 1,
    .master_id = 0x209,
    .slave_id = 0x2FF,
    .timeout = CHASSIS_FEEDBACK_TIMEOUT,
    .reduction = 1.0f,
    .is_invert = IS_SERVO_INVERT,
    .offset = -168.5f * deg, // 右后零位偏移 180 度，修正轮子驱动方向
    .control_mode = Motor::ANGLE_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .angle_pid_config = &servo_pid,
});
inline GM6020 s4({
    .can_port = 1,
    .master_id = 0x20A,
    .slave_id = 0x2FF,
    .timeout = CHASSIS_FEEDBACK_TIMEOUT,
    .reduction = 1.0f,
    .is_invert = IS_SERVO_INVERT,
    .offset = -18.4f * deg, // 右前零位
    .control_mode = Motor::ANGLE_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .angle_pid_config = &servo_pid,
});

// 底盘
inline Chassis_DualSteer chassis({
                                     .chassis_radius = CHASSIS_RADIUS,
                                     .wheel_radius = WHEEL_RADIUS,
                                     .vxyz_pid_config = &vxyz_pid,
                                 },
                                 {w1, w2, w3, w4, s1, s2, s3, s4});
