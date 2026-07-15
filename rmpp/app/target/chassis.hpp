#pragma once

#include "motor/M3508.hpp"
#include "module/chassis/Chassis_Omni.hpp"

// ==========================
// N630(C620 帧格式) + M3508 底盘配置
// ==========================
//
// 当前四个 N630 电调已经改成 DJI C620/M3508 兼容 CAN 协议：
// - 电机反馈帧：0x201、0x202、0x203、0x204，分别对应 w1~w4。
// - 电流控制帧：0x200，一帧 8 字节同时下发四个电机电流。
//
// 因此这里直接复用框架里的 M3508 电机类，而不是 VESC 类。
// M3508 类负责：
// 1. 接收并解析 0x201~0x204 的速度/电流/温度反馈。
// 2. 在 Motor::OnLoop() 中执行速度 PID。
// 3. 把速度 PID 输出转换成 int16 电流命令，供 app.cpp 打包进 0x200。

// 底盘 CAN 口。DJI C 板上 CAN1 对应数值 1。
static constexpr uint8_t CHASSIS_CAN_PORT = 1;

// 四个轮子的反馈 CAN ID。
// 轮序约定：
// w1 = 左前，w2 = 左后，w3 = 右后，w4 = 右前。
static constexpr uint32_t W1_ID = 0x201;
static constexpr uint32_t W2_ID = 0x202;
static constexpr uint32_t W3_ID = 0x203;
static constexpr uint32_t W4_ID = 0x204;

// C620/M3508 一拖四电流控制帧 ID。
static constexpr uint32_t CHASSIS_CONTROL_ID = 0x200;

// M3508 原厂减速箱参数和力矩常数。
// REDUCTION 用于电机转子速度和轮端速度换算。
// Kt 用于底盘模块估算电流能产生的牵引力。
static constexpr float REDUCTION = M3508::REDUCTION;
static constexpr UnitFloat<Nm_A> Kt = M3508::Kt;

// 电机方向统一反向标志。
// 如果后续发现某个轮子单独方向相反，优先只改对应 M3508 对象的 .is_invert。
static constexpr bool IS_INVERT = false;

// 机械尺寸。
// CHASSIS_RADIUS 是底盘中心到轮子接地点的等效旋转半径。
// WHEEL_RADIUS 是全向轮半径。
static constexpr UnitFloat CHASSIS_RADIUS = 230.0f * mm;
static constexpr UnitFloat WHEEL_RADIUS = 60.0f * mm;

// 估算底盘最大合力，用作通用底盘速度 PID 的输出上限。
static constexpr UnitFloat<N> MAX_F = M3508::MAX_CURRENT * Kt / WHEEL_RADIUS * 4;

// ==========================
// 通用底盘模块 PID
// ==========================
//
// vxyz_pid 属于 Chassis_Omni 通用模块：
// 它根据底盘 vx/vy/wr 速度误差计算期望牵引力。
// 当前 target/app.cpp 不再使用 Chassis_Omni 控制链，这组参数只保留作框架兼容。
inline PID::config_t vxyz_pid = {
    .kp = MAX_F / (1 * (m / s)),
    .ki = MAX_F / (10 * cm),
    .max_i = MAX_F,
    .max_out = MAX_F,
    .fc = 10 * Hz,
};

// 跟随模式 PID。
// 如果后续接入云台 yaw 并使用 Chassis::FOLLOW_MODE，这组 PID 用于控制底盘跟随云台。
// 当前 target 程序设置为 DETACH_MODE，主要保留兼容性。
inline PID::config_t follow_pid = {
    .kp = 5 * default_unit,
    .max_out = 360 * deg,
};

// ==========================
// 单个轮子的速度 PID
// ==========================
//
// N630 只接收电流命令，速度闭环放在 C 板上执行：
// wheel_speed_ref - wheel_speed_measure -> w*_speed_pid -> current.ref -> 0x200 CAN 帧。
//
// Ozone 调底盘速度环时主要看/改这里：
// - w1_speed_pid.*：左前轮
// - w2_speed_pid.*：左后轮
// - w3_speed_pid.*：右后轮
// - w4_speed_pid.*：右前轮
// 每个 PID 都有 kp/ki/kd/ff/max_i/max_out/fc，可单独调整。
inline PID::config_t w1_speed_pid = {
    .kp = (10.0f * A) / (125 * rpm),
    .ki = (10.0f * A) / (1 * rev),
    .kd = 0.0f * default_unit,
    .ff = 0.0f * A,
    .max_i = 3.0f * A,
    .max_out = 19.9f * A,
    .fc = 30 * Hz,
};

inline PID::config_t w2_speed_pid = {
    .kp = (10.0f * A) / (125 * rpm),
    .ki = (10.0f * A) / (1 * rev),
    .kd = 0.0f * default_unit,
    .ff = 0.0f * A,
    .max_i = 3.0f * A,
    .max_out = 19.9f * A,
    .fc = 30 * Hz,
};

inline PID::config_t w3_speed_pid = {
    .kp = (10.0f * A) / (125 * rpm),
    .ki = (10.0f * A) / (1 * rev),
    .kd = 0.0f * default_unit,
    .ff = 0.0f * A,
    .max_i = 3.0f * A,
    .max_out = 19.9f * A,
    .fc = 30 * Hz,
};

inline PID::config_t w4_speed_pid = {
    .kp = (10.0f * A) / (125 * rpm),
    .ki = (10.0f * A) / (1 * rev),
    .kd = 0.0f * default_unit,
    .ff = 0.0f * A,
    .max_i = 3.0f * A,
    .max_out = 19.9f * A,
    .fc = 30 * Hz,
};

// 注意：app.cpp 现在提供两套可切换控制链路。
// 四个底盘电机对象。
// master_id：本电机自己的反馈 ID。
// slave_id：下发控制用的 0x200 ID，app.cpp 会手动把四个 GetCanCmd() 打包。
// control_mode：速度模式。
// pid_out_type：PID 输出为电流。
inline M3508 w1({
    .can_port = CHASSIS_CAN_PORT,
    .master_id = W1_ID,
    .slave_id = CHASSIS_CONTROL_ID,
    .reduction = REDUCTION,
    .Kt = Kt,
    .is_invert = IS_INVERT,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &w1_speed_pid,
});

inline M3508 w2({
    .can_port = CHASSIS_CAN_PORT,
    .master_id = W2_ID,
    .slave_id = CHASSIS_CONTROL_ID,
    .reduction = REDUCTION,
    .Kt = Kt,
    .is_invert = IS_INVERT,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &w2_speed_pid,
});

inline M3508 w3({
    .can_port = CHASSIS_CAN_PORT,
    .master_id = W3_ID,
    .slave_id = CHASSIS_CONTROL_ID,
    .reduction = REDUCTION,
    .Kt = Kt,
    .is_invert = IS_INVERT,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &w3_speed_pid,
});

inline M3508 w4({
    .can_port = CHASSIS_CAN_PORT,
    .master_id = W4_ID,
    .slave_id = CHASSIS_CONTROL_ID,
    .reduction = REDUCTION,
    .Kt = Kt,
    .is_invert = IS_INVERT,
    .control_mode = Motor::SPEED_MODE,
    .pid_out_type = Motor::CURRENT_OUTPUT,
    .speed_pid_config = &w4_speed_pid,
});

// 通用四轮全向底盘模块。
// 当前 target/app.cpp 只保留该对象作框架兼容，不参与实际底盘控制。
inline Chassis_Omni chassis({
                                .chassis_radius = CHASSIS_RADIUS,
                                .wheel_radius = WHEEL_RADIUS,
                                .vxyz_pid_config = &vxyz_pid,
                                .follow_pid_config = &follow_pid,
                            },
                            {w1, w2, w3, w4});
