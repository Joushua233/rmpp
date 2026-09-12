#include "EL05.hpp"

#include <algorithm>
#include <cmath>

EL05::EL05(const config_t& config, const protocol_t& protocol) : Motor(config), protocol(protocol) {
    if (this->config.reduction == 0.0f) this->config.reduction = REDUCTION;
    if (this->config.Kt == 0.0f) this->config.Kt = Kt;

    auto callback = [this](const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
        this->callback(port, id, data, dlc);
    };
    BSP::CAN::RegisterCallback(callback);
}

bool EL05::isConfigValid() const {
    return (config.can_port == 1 || config.can_port == 2)
        && config.master_id <= 0xFF
        && config.slave_id <= 0xFF;
}

void EL05::SendCanCmd() {
    if (!isConfigValid()) return;

    if (dwt_connect.GetDT() > config.timeout) is_connect = false;

    if (!is_enable || fault != 0) {
        send_cnt = 0;
        sendStop();
        return;
    }

    // Enable immediately, then refresh every 100 ms. Until the first valid
    // feedback arrives, keep the remote-control torque command at zero.
    if (send_cnt++ % 100 == 0) {
        sendEnable();
    } else if (is_connect && mode == MOTOR_MODE) {
        sendTorque(torque.ref);
    } else {
        sendTorque(0 * Nm);
    }
}

void EL05::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    if (!isConfigValid()) return;
    if (port != config.can_port || dlc != 8) return;

    const uint8_t type = (id >> 24) & 0x1F;
    if (type != FEEDBACK) return;

    const uint8_t motor_id = (id >> 8) & 0xFF;
    const uint8_t host_id = id & 0xFF;
    if (motor_id != config.slave_id || host_id != config.master_id) return;

    fault = (id >> 16) & 0x3F;
    mode = static_cast<mode_e>((id >> 22) & 0x03);

    const uint16_t angle_u16 = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    const uint16_t speed_u16 = (static_cast<uint16_t>(data[2]) << 8) | data[3];
    const uint16_t torque_u16 = (static_cast<uint16_t>(data[4]) << 8) | data[5];
    const uint16_t temperature_u16 = (static_cast<uint16_t>(data[6]) << 8) | data[7];

    const UnitFloat torque = uintToFloat(torque_u16, T_MIN, T_MAX) * Nm;
    temperature.motor = static_cast<float>(temperature_u16) / 10.0f * C;

    const raw_t raw = {
        .current = torque / config.Kt,
        .torque = torque,
        .speed = uintToFloat(speed_u16, V_MIN, V_MAX) * rad_s,
        .angle = uintToFloat(angle_u16, P_MIN, P_MAX) * rad,
    };
    Motor::callback(raw);
}

float EL05::uintToFloat(const uint16_t value, const float min, const float max) {
    return static_cast<float>(value) * (max - min) / 65535.0f + min;
}

uint16_t EL05::floatToUint(const float value, const float min, const float max) {
    const float clamped = std::clamp(value, min, max);
    return static_cast<uint16_t>((clamped - min) * 65535.0f / (max - min));
}

uint32_t EL05::makeExtId(const uint8_t type, const uint16_t data, const uint8_t target_id) {
    return (static_cast<uint32_t>(type & 0x1F) << 24)
        | (static_cast<uint32_t>(data) << 8)
        | target_id;
}

void EL05::sendEnable() const {
    uint8_t data[8]{};
    const uint32_t id = makeExtId(ENABLE, static_cast<uint16_t>(config.master_id),
                                  static_cast<uint8_t>(config.slave_id));
    BSP::CAN::Transmit(config.can_port, id, data, 8, true);
}

void EL05::sendStop(const bool clear_fault) const {
    uint8_t data[8]{};
    data[0] = clear_fault ? 1 : 0;
    const uint32_t id = makeExtId(STOP, static_cast<uint16_t>(config.master_id),
                                  static_cast<uint8_t>(config.slave_id));
    BSP::CAN::Transmit(config.can_port, id, data, 8, true);
}

void EL05::sendTorque(const UnitFloat<>& torque) const {
    if (!std::isfinite(torque.toFloat(Nm))) {
        sendStop();
        return;
    }

    const float torque_limit = std::clamp(torque.toFloat(Nm), T_MIN, T_MAX);
    const bool invert_command = config.is_invert != protocol.is_torque_invert;
    const float command = invert_command ? -torque_limit : torque_limit;
    const uint16_t torque_u16 = floatToUint(command, T_MIN, T_MAX);

    // Pure torque remote-control command: p/v/kp/kd are all zero.
    const uint16_t angle_u16 = floatToUint(0.0f, P_MIN, P_MAX);
    const uint16_t speed_u16 = floatToUint(0.0f, V_MIN, V_MAX);
    uint8_t data[8] = {
        static_cast<uint8_t>(angle_u16 >> 8), static_cast<uint8_t>(angle_u16),
        static_cast<uint8_t>(speed_u16 >> 8), static_cast<uint8_t>(speed_u16),
        0, 0, 0, 0,
    };

    const uint32_t id = makeExtId(REMOTE_CONTROL, torque_u16,
                                  static_cast<uint8_t>(config.slave_id));
    BSP::CAN::Transmit(config.can_port, id, data, 8, true);
}
