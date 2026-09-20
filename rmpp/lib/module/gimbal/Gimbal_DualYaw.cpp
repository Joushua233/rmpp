#include "Gimbal_DualYaw.hpp"

Gimbal_DualYaw::Gimbal_DualYaw(IMU& imu, const motor_t& motor) : Gimbal(imu), motor(motor) {}

void Gimbal_DualYaw::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    motor.yaw1.SetEnable(is_enable);
    motor.yaw2.SetEnable(is_enable);
    motor.pitch.SetEnable(is_enable);

    if (!is_enable) {
        is_yaw_allocator_init = false;
        yaw2_speed_ff = 0 * deg_s;
    }
}

void Gimbal_DualYaw::LoadYawOffset(FlashDB& flashdb) {
    motor.yaw1.config.offset = flashdb.Read("yaw_offset") * deg;
}

void Gimbal_DualYaw::SaveYawOffset(FlashDB& flashdb) {
    flashdb.Write("yaw_offset", motor.yaw1.config.offset.toFloat(deg));
}

void Gimbal_DualYaw::SetYawZero() {
    motor.yaw1.config.offset = motor.yaw1.raw.angle;
}

void Gimbal_DualYaw::UpdateUI(UI& ui) {
    ui.robot.yaw1 = motor.yaw1.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.yaw2 = motor.yaw2.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.pitch = motor.pitch.is_connect ? UI::GREEN : UI::PINK;
}

void Gimbal_DualYaw::OnLoop() {
    Gimbal::OnLoop();

    // 更新电机
    motor.yaw1.OnLoop();
    motor.yaw2.OnLoop();
    motor.pitch.OnLoop();
}

void Gimbal_DualYaw::forward() {
    // 从电机读取
    yaw1.measure = motor.yaw1.angle.measure;
    yaw2.measure = motor.yaw2.angle.measure;
    pitch.ecd.measure = motor.pitch.angle.measure;

    // 合并大小yaw
    yaw.ecd.measure = yaw1.measure + yaw2.measure;

    // 读取imu
    yaw.imu.measure = imu.yaw;
    pitch.imu.measure = imu.pitch;
}

void Gimbal_DualYaw::backward() {
    // 用于参考系转换
    yaw.imu_minus_ecd = yaw.imu.measure - yaw.ecd.measure;
    pitch.imu_minus_ecd = pitch.imu.measure - pitch.ecd.measure;

    if (mode == ECD_MODE) {
        // 换算到imu参考系
        yaw.imu.ref = yaw.ecd.ref + yaw.imu_minus_ecd;
        pitch.imu.ref = pitch.ecd.ref + pitch.imu_minus_ecd;
    } else if (mode == IMU_MODE) {
        // 换算到ecd参考系
        yaw.ecd.ref = yaw.imu.ref - yaw.imu_minus_ecd;
        pitch.ecd.ref = pitch.imu.ref - pitch.imu_minus_ecd;
    }

    // 分配大小 yaw：
    // 1. 大 yaw 对总目标做一阶低频跟随，避免和小 yaw 同时吃满完整指令。
    // 2. 小 yaw 使用大 yaw 实际角度计算残差，仍能补偿大 yaw 的真实滞后。
    const UnitFloat dt = dwt_yaw_allocator.UpdateDT();
    UnitFloat<deg_s> yaw1_ref_speed = 0 * deg_s;
    if (!is_yaw_allocator_init || !is_enable) {
        yaw1.ref = yaw1.measure;
        yaw2_speed_ff = 0 * deg_s;
        is_yaw_allocator_init = is_enable;
    } else {
        const Angle yaw1_error = yaw.ecd.ref - yaw1.ref;
        const UnitFloat alpha = dt / (YAW1_FOLLOW_TAU + dt);
        const Angle yaw1_delta = yaw1_error * alpha;
        yaw1.ref += yaw1_delta;
        if (dt > 0 * s) yaw1_ref_speed = yaw1_delta / dt;
    }
    yaw2.ref = yaw.ecd.ref - yaw1.measure;

    // 松杆时原始前馈会瞬间跳到 -yaw1.speed，一阶平滑可减少小 yaw 反向猛甩。
    // 角度残差闭环仍保留，因此不影响最终跟踪精度。
    const UnitFloat<deg_s> yaw2_speed_ff_raw = -chassis_wr + yaw_speed - motor.yaw1.speed.measure;
    const UnitFloat yaw2_ff_alpha = dt / (YAW2_SPEED_FF_TAU + dt);
    yaw2_speed_ff += (yaw2_speed_ff_raw - yaw2_speed_ff) * yaw2_ff_alpha;

    // 设置电机角度
    yaw1.ref = motor.yaw1.SetAngle(yaw1.ref, yaw1_ref_speed);
    yaw2.ref = motor.yaw2.SetAngle(yaw2.ref, yaw2_speed_ff);
    pitch.ecd.ref = motor.pitch.SetAngle(pitch.ecd.ref, pitch_speed);

    // 传递软件限位到imu参考系
    yaw.imu.ref = yaw.ecd.ref + yaw.imu_minus_ecd;
    pitch.imu.ref = pitch.ecd.ref + pitch.imu_minus_ecd;
}
