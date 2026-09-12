#pragma once

#include "Chassis.hpp"
#include "motor/Motor.hpp"

// 沿用原类名和接口，当前扩展为四舵轮底盘。
class Chassis_DualSteer : public Chassis {
public:
    // 底盘电机
    struct motor_t {
        Motor &w1, &w2, &w3, &w4; // 轮电机（左前，左后，右后，右前）
        Motor &s1, &s2, &s3, &s4; // 舵电机（左前，左后，右后，右前）
    } motor;

    // 舵轮角度
    struct {
        Angle<deg> measure, ref_v, ref_f;
        bool is_initialized = false; // 失能或断联后，先用当前舵角初始化目标
    } s1, s2, s3, s4;

    Chassis_DualSteer(const config_t& config, const motor_t& motor);

    // 底盘使能/失能
    void SetEnable(bool is_enable) override;

    // 在自定义UI上显示电机连接状态
    void UpdateUI(UI& ui) override;

    // 需要在循环中调用
    void OnLoop() override;

private:
    static constexpr UnitFloat<> MIN_V = 1 * (cm / s); // 最小转舵速度
    static constexpr UnitFloat<> MIN_F = 1 * N;        // 最小转舵力

    bool is_invert_v1 = false, is_invert_v2 = false, is_invert_v3 = false, is_invert_v4 = false;
    bool is_invert_f1 = false, is_invert_f2 = false, is_invert_f3 = false, is_invert_f4 = false;

    // 速度和力学正解
    void forward() override;

    // 速度和力学逆解
    void backward() override;

    // 功率控制，必须放在【更新电机之后】
    void powerControlAfterMotorUpdate();
};