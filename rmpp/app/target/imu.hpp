#pragma once

#include "imu/IMU.hpp"

// ==========================
// C 板板载 IMU 配置
// ==========================
//
// DJI C 板使用 BMI088 IMU。这里先复用步兵工程中的安装方向和零偏参数。
//
// imu_dir：
// - 描述 IMU 坐标系相对车身坐标系的安装方向。
// - 当前 yaw/pitch/roll 都为 0，表示暂时认为板子安装方向和程序默认方向一致。
//
// imu_calib：
// - gx/gy/gz_offset 是陀螺仪三轴零偏。
// - g_norm 是当地重力加速度标定值。
//
// 如果车静止时 target_imu_yaw_deg 仍然明显漂移，需要在本车上重新执行 IMU 标定，
// 再把新的 offset 和 g_norm 写回这里。
inline IMU::dir_t imu_dir = {.yaw = 0 * deg, .pitch = 0 * deg, .roll = 0 * deg};
inline IMU::calib_t imu_calib = {.gx_offset = 0.00159558014, .gy_offset = -0.00282915123, .gz_offset = -0.000537958229, .g_norm = 9.70215511};

// 全局 IMU 对象。
// app.cpp 每 1 ms 调用 imu.OnLoop() 更新姿态，之后通过 imu.yaw 参与世界坐标解算和 yaw 补偿。
inline IMU imu(imu_dir, imu_calib);
