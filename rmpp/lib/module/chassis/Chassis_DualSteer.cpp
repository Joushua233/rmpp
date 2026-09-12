#include "Chassis_DualSteer.hpp"

// 正方形布置：x 向前、y 向左，舵角零度向前，逆时针旋转为正。
// 前后、左右半轴距与底盘半径的比值均为 sqrt(2) / 2。
static const float sqrt2div2 = std::sqrt(2.0f) / 2.0f;

Chassis_DualSteer::Chassis_DualSteer(const config_t& config, const motor_t& motor) : Chassis(config), motor(motor) {}

void Chassis_DualSteer::SetEnable(const bool is_enable) {
    if (this->is_enable == is_enable) return;
    this->is_enable = is_enable;

    SetMode(DETACH_MODE); // 失能/使能后默认分离模式，防止车突然转动伤人

    motor.w1.SetEnable(is_enable);
    motor.w2.SetEnable(is_enable);
    motor.w3.SetEnable(is_enable);
    motor.w4.SetEnable(is_enable);
    motor.s1.SetEnable(is_enable);
    motor.s2.SetEnable(is_enable);
    motor.s3.SetEnable(is_enable);
    motor.s4.SetEnable(is_enable);

    // 清除旧轮速、电流；再次使能时重新读取舵角，避免沿用上次目标。
    motor.w1.SetSpeed(0 * rpm);
    motor.w1.SetCurrent(0 * A);
    s1.is_initialized = false;
    motor.w2.SetSpeed(0 * rpm);
    motor.w2.SetCurrent(0 * A);
    s2.is_initialized = false;
    motor.w3.SetSpeed(0 * rpm);
    motor.w3.SetCurrent(0 * A);
    s3.is_initialized = false;
    motor.w4.SetSpeed(0 * rpm);
    motor.w4.SetCurrent(0 * A);
    s4.is_initialized = false;
}

void Chassis_DualSteer::UpdateUI(UI& ui) {
    ui.robot.w1 = motor.w1.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.w2 = motor.w2.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.w3 = motor.w3.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.w4 = motor.w4.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.s1 = motor.s1.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.s2 = motor.s2.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.s3 = motor.s3.is_connect ? UI::GREEN : UI::PINK;
    ui.robot.s4 = motor.s4.is_connect ? UI::GREEN : UI::PINK;
}

void Chassis_DualSteer::OnLoop() {
    Chassis::OnLoop();

    // 先更新舵电机断联状态，舵电机断联时禁止对应轮电机驱动。
    motor.s1.OnLoop();
    motor.s2.OnLoop();
    motor.s3.OnLoop();
    motor.s4.OnLoop();
    motor.w1.SetEnable(is_enable && motor.s1.is_connect);
    motor.w1.OnLoop();
    motor.w2.SetEnable(is_enable && motor.s2.is_connect);
    motor.w2.OnLoop();
    motor.w3.SetEnable(is_enable && motor.s3.is_connect);
    motor.w3.OnLoop();
    motor.w4.SetEnable(is_enable && motor.s4.is_connect);
    motor.w4.OnLoop();

    // 轮电机断联时也必须清零电流，防止发送上次闭环留下的输出。
    if (!is_enable || !motor.w1.is_connect || !motor.s1.is_connect) {
        motor.w1.SetCurrent(0 * A);
        motor.w1.SetSpeed(0 * rpm);
        s1.is_initialized = false;
    }
    if (!is_enable || !motor.w2.is_connect || !motor.s2.is_connect) {
        motor.w2.SetCurrent(0 * A);
        motor.w2.SetSpeed(0 * rpm);
        s2.is_initialized = false;
    }
    if (!is_enable || !motor.w3.is_connect || !motor.s3.is_connect) {
        motor.w3.SetCurrent(0 * A);
        motor.w3.SetSpeed(0 * rpm);
        s3.is_initialized = false;
    }
    if (!is_enable || !motor.w4.is_connect || !motor.s4.is_connect) {
        motor.w4.SetCurrent(0 * A);
        motor.w4.SetSpeed(0 * rpm);
        s4.is_initialized = false;
    }

    powerControlAfterMotorUpdate();
}

void Chassis_DualSteer::forward() {
    // 读取舵向
    s1.measure = motor.s1.angle.measure;
    s2.measure = motor.s2.angle.measure;
    s3.measure = motor.s3.angle.measure;
    s4.measure = motor.s4.angle.measure;

    // 四舵轮速度正解
    const UnitFloat<m_s> v1 = motor.w1.speed.measure * config.wheel_radius;
    const UnitFloat<m_s> v2 = motor.w2.speed.measure * config.wheel_radius;
    const UnitFloat<m_s> v3 = motor.w3.speed.measure * config.wheel_radius;
    const UnitFloat<m_s> v4 = motor.w4.speed.measure * config.wheel_radius;
    const UnitFloat<m_s> vx1 = v1 * unit::cos(s1.measure);
    const UnitFloat<m_s> vy1 = v1 * unit::sin(s1.measure);
    const UnitFloat<m_s> vz1 = v1 * unit::cos(s1.measure - (135 * deg));
    const UnitFloat<m_s> vx2 = v2 * unit::cos(s2.measure);
    const UnitFloat<m_s> vy2 = v2 * unit::sin(s2.measure);
    const UnitFloat<m_s> vz2 = v2 * unit::cos(s2.measure - (-135 * deg));
    const UnitFloat<m_s> vx3 = v3 * unit::cos(s3.measure);
    const UnitFloat<m_s> vy3 = v3 * unit::sin(s3.measure);
    const UnitFloat<m_s> vz3 = v3 * unit::cos(s3.measure - (-45 * deg));
    const UnitFloat<m_s> vx4 = v4 * unit::cos(s4.measure);
    const UnitFloat<m_s> vy4 = v4 * unit::sin(s4.measure);
    const UnitFloat<m_s> vz4 = v4 * unit::cos(s4.measure - (45 * deg));
    const UnitFloat<m_s> vx = (vx1 + vx2 + vx3 + vx4) / 4;
    const UnitFloat<m_s> vy = (vy1 + vy2 + vy3 + vy4) / 4;
    const UnitFloat<m_s> vz = (vz1 + vz2 + vz3 + vz4) / 4;
    std::tie(this->vx.measure, this->vy.measure) = unit::rotate(vx, vy, -gimbal_yaw);
    this->vz.measure = vz;
    wr.measure = vz / config.chassis_radius;

    // 四舵轮力学正解
    const UnitFloat<N> f1 = motor.w1.torque.measure / config.wheel_radius;
    const UnitFloat<N> f2 = motor.w2.torque.measure / config.wheel_radius;
    const UnitFloat<N> f3 = motor.w3.torque.measure / config.wheel_radius;
    const UnitFloat<N> f4 = motor.w4.torque.measure / config.wheel_radius;
    const UnitFloat<N> fx1 = f1 * unit::cos(s1.measure);
    const UnitFloat<N> fy1 = f1 * unit::sin(s1.measure);
    const UnitFloat<N> fz1 = f1 * unit::cos(s1.measure - (135 * deg));
    const UnitFloat<N> fx2 = f2 * unit::cos(s2.measure);
    const UnitFloat<N> fy2 = f2 * unit::sin(s2.measure);
    const UnitFloat<N> fz2 = f2 * unit::cos(s2.measure - (-135 * deg));
    const UnitFloat<N> fx3 = f3 * unit::cos(s3.measure);
    const UnitFloat<N> fy3 = f3 * unit::sin(s3.measure);
    const UnitFloat<N> fz3 = f3 * unit::cos(s3.measure - (-45 * deg));
    const UnitFloat<N> fx4 = f4 * unit::cos(s4.measure);
    const UnitFloat<N> fy4 = f4 * unit::sin(s4.measure);
    const UnitFloat<N> fz4 = f4 * unit::cos(s4.measure - (45 * deg));
    const UnitFloat<N> fx = (fx1 + fx2 + fx3 + fx4);
    const UnitFloat<N> fy = (fy1 + fy2 + fy3 + fy4);
    const UnitFloat<N> fz = (fz1 + fz2 + fz3 + fz4);
    std::tie(this->fx.measure, this->fy.measure) = unit::rotate(fx, fy, -gimbal_yaw);
    this->fz.measure = fz;
}

void Chassis_DualSteer::backward() {
    // 上电、失能或断联后，静止时保持当前舵向。
    if (!s1.is_initialized) {
        s1.ref_v = s1.ref_f = s1.measure;
        s1.is_initialized = true;
    }
    if (!s2.is_initialized) {
        s2.ref_v = s2.ref_f = s2.measure;
        s2.is_initialized = true;
    }
    if (!s3.is_initialized) {
        s3.ref_v = s3.ref_f = s3.measure;
        s3.is_initialized = true;
    }
    if (!s4.is_initialized) {
        s4.ref_v = s4.ref_f = s4.measure;
        s4.is_initialized = true;
    }

    // 四舵轮速度逆解
    auto [vx, vy] = unit::rotate(this->vx.ref, this->vy.ref, gimbal_yaw);
    auto vz = this->vz.ref;
    // 左前舵轮
    const UnitFloat<m_s> vx1 = (vx - sqrt2div2 * vz);
    const UnitFloat<m_s> vy1 = (vy + sqrt2div2 * vz);
    UnitFloat<m_s> v1 = unit::sqrt(vx1 * vx1 + vy1 * vy1);
    if (v1 > MIN_V) {
        s1.ref_v = unit::atan2(vy1, vx1);
        if (unit::abs(Angle(s1.ref_v - s1.measure)) > 90 * deg) { // 就近转舵
            s1.ref_v += 180 * deg;
            is_invert_v1 = true;
        } else {
            is_invert_v1 = false;
        }
    } else {
        v1 = 0 * (m / s); // 停车保持上次舵向，避免零速时转舵抖动
    }
    // 左后舵轮
    const UnitFloat<m_s> vx2 = (vx - sqrt2div2 * vz);
    const UnitFloat<m_s> vy2 = (vy - sqrt2div2 * vz);
    UnitFloat<m_s> v2 = unit::sqrt(vx2 * vx2 + vy2 * vy2);
    if (v2 > MIN_V) {
        s2.ref_v = unit::atan2(vy2, vx2);
        if (unit::abs(Angle(s2.ref_v - s2.measure)) > 90 * deg) { // 就近转舵
            s2.ref_v += 180 * deg;
            is_invert_v2 = true;
        } else {
            is_invert_v2 = false;
        }
    } else {
        v2 = 0 * (m / s); // 停车保持上次舵向，避免零速时转舵抖动
    }
    // 右后舵轮
    const UnitFloat<m_s> vx3 = (vx + sqrt2div2 * vz);
    const UnitFloat<m_s> vy3 = (vy - sqrt2div2 * vz);
    UnitFloat<m_s> v3 = unit::sqrt(vx3 * vx3 + vy3 * vy3);
    if (v3 > MIN_V) {
        s3.ref_v = unit::atan2(vy3, vx3);
        if (unit::abs(Angle(s3.ref_v - s3.measure)) > 90 * deg) { // 就近转舵
            s3.ref_v += 180 * deg;
            is_invert_v3 = true;
        } else {
            is_invert_v3 = false;
        }
    } else {
        v3 = 0 * (m / s); // 停车保持上次舵向，避免零速时转舵抖动
    }
    // 右前舵轮
    const UnitFloat<m_s> vx4 = (vx + sqrt2div2 * vz);
    const UnitFloat<m_s> vy4 = (vy + sqrt2div2 * vz);
    UnitFloat<m_s> v4 = unit::sqrt(vx4 * vx4 + vy4 * vy4);
    if (v4 > MIN_V) {
        s4.ref_v = unit::atan2(vy4, vx4);
        if (unit::abs(Angle(s4.ref_v - s4.measure)) > 90 * deg) { // 就近转舵
            s4.ref_v += 180 * deg;
            is_invert_v4 = true;
        } else {
            is_invert_v4 = false;
        }
    } else {
        v4 = 0 * (m / s); // 停车保持上次舵向，避免零速时转舵抖动
    }

    if (is_invert_v1) v1 = -v1;
    if (is_invert_v2) v2 = -v2;
    if (is_invert_v3) v3 = -v3;
    if (is_invert_v4) v4 = -v4;

    // 轮速投影到当前舵向，转舵误差为 90 度时不驱动。
    v1 *= unit::clamp(unit::cos(Angle(s1.ref_v - s1.measure)), 0 * ratio, 1 * ratio);
    motor.w1.SetSpeed(v1 / config.wheel_radius);
    v2 *= unit::clamp(unit::cos(Angle(s2.ref_v - s2.measure)), 0 * ratio, 1 * ratio);
    motor.w2.SetSpeed(v2 / config.wheel_radius);
    v3 *= unit::clamp(unit::cos(Angle(s3.ref_v - s3.measure)), 0 * ratio, 1 * ratio);
    motor.w3.SetSpeed(v3 / config.wheel_radius);
    v4 *= unit::clamp(unit::cos(Angle(s4.ref_v - s4.measure)), 0 * ratio, 1 * ratio);
    motor.w4.SetSpeed(v4 / config.wheel_radius);

    // 四舵轮力学逆解
    auto [fx, fy] = unit::rotate(this->fx.ref, this->fy.ref, gimbal_yaw);
    auto fz = this->fz.ref;
    // 左前舵轮
    const UnitFloat<N> fx1 = (fx - sqrt2div2 * fz) / 4;
    const UnitFloat<N> fy1 = (fy + sqrt2div2 * fz) / 4;
    UnitFloat<N> f1 = unit::sqrt(fx1 * fx1 + fy1 * fy1);
    if (f1 > MIN_F) {
        s1.ref_f = unit::atan2(fy1, fx1);
        if (unit::abs(Angle(s1.ref_f - s1.measure)) > 90 * deg) { // 就近转舵
            s1.ref_f += 180 * deg;
            is_invert_f1 = true;
        } else {
            is_invert_f1 = false;
        }
    }
    // 左后舵轮
    const UnitFloat<N> fx2 = (fx - sqrt2div2 * fz) / 4;
    const UnitFloat<N> fy2 = (fy - sqrt2div2 * fz) / 4;
    UnitFloat<N> f2 = unit::sqrt(fx2 * fx2 + fy2 * fy2);
    if (f2 > MIN_F) {
        s2.ref_f = unit::atan2(fy2, fx2);
        if (unit::abs(Angle(s2.ref_f - s2.measure)) > 90 * deg) { // 就近转舵
            s2.ref_f += 180 * deg;
            is_invert_f2 = true;
        } else {
            is_invert_f2 = false;
        }
    }
    // 右后舵轮
    const UnitFloat<N> fx3 = (fx + sqrt2div2 * fz) / 4;
    const UnitFloat<N> fy3 = (fy - sqrt2div2 * fz) / 4;
    UnitFloat<N> f3 = unit::sqrt(fx3 * fx3 + fy3 * fy3);
    if (f3 > MIN_F) {
        s3.ref_f = unit::atan2(fy3, fx3);
        if (unit::abs(Angle(s3.ref_f - s3.measure)) > 90 * deg) { // 就近转舵
            s3.ref_f += 180 * deg;
            is_invert_f3 = true;
        } else {
            is_invert_f3 = false;
        }
    }
    // 右前舵轮
    const UnitFloat<N> fx4 = (fx + sqrt2div2 * fz) / 4;
    const UnitFloat<N> fy4 = (fy + sqrt2div2 * fz) / 4;
    UnitFloat<N> f4 = unit::sqrt(fx4 * fx4 + fy4 * fy4);
    if (f4 > MIN_F) {
        s4.ref_f = unit::atan2(fy4, fx4);
        if (unit::abs(Angle(s4.ref_f - s4.measure)) > 90 * deg) { // 就近转舵
            s4.ref_f += 180 * deg;
            is_invert_f4 = true;
        } else {
            is_invert_f4 = false;
        }
    }

    if (is_invert_f1) f1 = -f1;
    if (is_invert_f2) f2 = -f2;
    if (is_invert_f3) f3 = -f3;
    if (is_invert_f4) f4 = -f4;

    // 保留原力矩接口；当前速度闭环会在 Motor::OnLoop 中更新轮电机力矩。
    motor.w1.SetTorque(f1 * config.wheel_radius);
    motor.w2.SetTorque(f2 * config.wheel_radius);
    motor.w3.SetTorque(f3 * config.wheel_radius);
    motor.w4.SetTorque(f4 * config.wheel_radius);

    // 当前使用速度逆解得到的舵向。
    motor.s1.SetAngle(s1.ref_v);
    motor.s2.SetAngle(s2.ref_v);
    motor.s3.SetAngle(s3.ref_v);
    motor.s4.SetAngle(s4.ref_v);
}

void Chassis_DualSteer::powerControlAfterMotorUpdate() {
    // 四个轮电机的机械功率与铜耗；不包含舵电机功耗。
    power.estimate = motor.w1.torque.measure * motor.w1.speed.measure
        + 3 * unit::square(motor.w1.current.measure) * motor.w1.config.R
        + motor.w2.torque.measure * motor.w2.speed.measure
        + 3 * unit::square(motor.w2.current.measure) * motor.w2.config.R
        + motor.w3.torque.measure * motor.w3.speed.measure
        + 3 * unit::square(motor.w3.current.measure) * motor.w3.config.R
        + motor.w4.torque.measure * motor.w4.speed.measure
        + 3 * unit::square(motor.w4.current.measure) * motor.w4.config.R;

    power.gain = 100 * pct;
    // 裁判系统设置无限功率时，不进行功率和缓冲能量衰减。
    if (power.limit == 65535 * W) return;

    // 电流统一乘 gain 后，预测功率为 a * gain^2 + b * gain。
    const UnitFloat<> a = 3 * unit::square(motor.w1.current.ref) * motor.w1.config.R
        + 3 * unit::square(motor.w2.current.ref) * motor.w2.config.R
        + 3 * unit::square(motor.w3.current.ref) * motor.w3.config.R
        + 3 * unit::square(motor.w4.current.ref) * motor.w4.config.R;
    const UnitFloat<> b = motor.w1.torque.ref * motor.w1.speed.measure
        + motor.w2.torque.ref * motor.w2.speed.measure
        + motor.w3.torque.ref * motor.w3.speed.measure
        + motor.w4.torque.ref * motor.w4.speed.measure;
    const UnitFloat<> c = -unit::clamp(power.limit, 0 * W, 65535 * W);
    if (a + b + c > 0 * W) {
        if (a > 0 * W) {
            power.gain = (-b + unit::sqrt(b * b - 4 * a * c)) / (2 * a);
        } else if (b > 0 * W) { // 零电流时避免二次公式除零
            power.gain = -c / b;
        }
    }
    power.gain = unit::clamp(power.gain, 0 * pct, 100 * pct);

    // 缓冲能量低于 30 J 时停止驱动，高于 60 J 时不再增加输出。
    power.gain *= unit::clamp((power.buffer_energy - 30 * J) / (30 * J), 0 * ratio, 1 * ratio);

    // 在轮电机 PID 更新后衰减，避免本周期被速度闭环覆盖。
    motor.w1.SetTorque(motor.w1.torque.ref * power.gain);
    motor.w2.SetTorque(motor.w2.torque.ref * power.gain);
    motor.w3.SetTorque(motor.w3.torque.ref * power.gain);
    motor.w4.SetTorque(motor.w4.torque.ref * power.gain);
}
