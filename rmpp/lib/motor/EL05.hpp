#pragma once

#include "Motor.hpp"

// RobStride EL05 integrated actuator, private CAN 2.0 extended-frame protocol.
//
// The motor-side controller stays in remote-control mode, while Motor performs
// the position/speed loops on the main controller and sends torque feedforward.
class EL05 : public Motor {
public:
    static constexpr float REDUCTION = 1.0f;
    static constexpr UnitFloat<> Kt = 0.94f * Nm_A;
    static constexpr UnitFloat<> MAX_TORQUE = 6.0f * Nm;
    static constexpr UnitFloat<A> MAX_CURRENT = 11.0f * A;

    // Factory IDs used by the EL05 manual.
    static constexpr uint8_t DEFAULT_MOTOR_ID = 0x7F;
    static constexpr uint8_t DEFAULT_HOST_ID = 0xFD;

    enum mode_e : uint8_t {
        RESET_MODE = 0,
        CALIBRATION_MODE = 1,
        MOTOR_MODE = 2,
    } mode = RESET_MODE;

    uint8_t fault = 0;

    struct protocol_t {
        // Some installations report positive position for the opposite torque
        // direction. Keep this independent from Motor::config.is_invert so the
        // feedback coordinate and command polarity can be corrected separately.
        bool is_torque_invert = false;
    } protocol;

    EL05(const config_t& config, const protocol_t& protocol);

    // Call at the same rate as the main control loop.
    void SendCanCmd();

private:
    static constexpr float P_MIN = -12.57f;
    static constexpr float P_MAX = 12.57f;
    static constexpr float V_MIN = -50.0f;
    static constexpr float V_MAX = 50.0f;
    static constexpr float T_MIN = -6.0f;
    static constexpr float T_MAX = 6.0f;

    enum communication_type_e : uint8_t {
        REMOTE_CONTROL = 1,
        FEEDBACK = 2,
        ENABLE = 3,
        STOP = 4,
    };

    uint32_t send_cnt = 0;

    bool isConfigValid() const;

    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);

    static float uintToFloat(uint16_t value, float min, float max);
    static uint16_t floatToUint(float value, float min, float max);
    static uint32_t makeExtId(uint8_t type, uint16_t data, uint8_t target_id);

    void sendEnable() const;
    void sendStop(bool clear_fault = false) const;
    void sendTorque(const UnitFloat<>& torque) const;
};
