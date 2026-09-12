#pragma once

#include "Motor.hpp"

// DM-J6006-2EC 双编码器减速电机，MIT 纯扭矩控制。
class DMJ6006 : public Motor {
public:
    // CAN 反馈已经是输出轴位置、速度；内部 6:1 减速比不再用于软件角度换算。
    static constexpr float REDUCTION = 1.0f;
    // 按额定 4 Nm / 4 A 估算，仅用于 Motor 电流/扭矩接口换算。
    // 协议反馈的是扭矩，current.measure 是估算值，不是独立电流反馈。
    static constexpr UnitFloat<> Kt = 1.0f * Nm_A;
    static constexpr UnitFloat<> R = 0.33f * Ohm;
    static constexpr UnitFloat<> MAX_TORQUE = 11.0f * Nm;

    // 必须与调试助手中的 MIT 映射量程一致；零值表示尚未配置。
    struct protocol_t {
        float p_max = 0.0f; // rad
        float v_max = 0.0f; // rad/s
        float t_max = 0.0f; // Nm，映射量程不等于实际允许输出上限
    } protocol;

    // 反馈高四位同时包含使能状态和错误代码。
    enum err_code_e {
        NONE      = 0x00, // 失能，无故障
        ENABLED   = 0x01, // 使能，无故障
        OVP       = 0x08, // 超压
        UVP       = 0x09, // 欠压
        OCP       = 0x0A, // 过电流
        MOS_OTP   = 0x0B, // MOS 过温
        MOTOR_OTP = 0x0C, // 电机过温
        COMM_LOST = 0x0D, // 通讯丢失
        OVERLOAD  = 0x0E  // 过载
    } err_code = NONE;

    DMJ6006(const config_t& config, const protocol_t& protocol);

    void SendCanCmd();

private:
    uint32_t send_cnt = 0; // 用于定期发送使能命令

    bool isConfigValid() const;

    // CAN 接收回调
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);

    static float uint_to_float(int x_int, float x_min, float x_max, int bits);
    static uint16_t float_to_uint(float x, float x_min, float x_max, int bits);

    void sendEnable() const;
    void sendDisable() const;
    void sendTorque(const UnitFloat<>& torque) const;
};
