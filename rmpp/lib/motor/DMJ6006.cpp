#include "DMJ6006.hpp"
#include <algorithm>

DMJ6006::DMJ6006(const config_t& config, const protocol_t& protocol) : Motor(config), protocol(protocol) {
    // 设置电机默认参数
    if (this->config.reduction == 0.0f) this->config.reduction = REDUCTION;
    if (this->config.Kt == 0.0f) this->config.Kt = Kt;
    if (this->config.R == 0.0f) this->config.R = R;

    // 注册 CAN 回调
    auto callback = [this](const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
        this->callback(port, id, data, dlc);
    };
    BSP::CAN::RegisterCallback(callback);
}

bool DMJ6006::isConfigValid() const {
    return (config.can_port == 1 || config.can_port == 2)
        && config.slave_id > 0 && config.slave_id <= 0x7FF && config.master_id <= 0x7FF
        && std::isfinite(protocol.p_max) && protocol.p_max > 0.0f
        && std::isfinite(protocol.v_max) && protocol.v_max > 0.0f
        && std::isfinite(protocol.t_max) && protocol.t_max > 0.0f;
}

void DMJ6006::SendCanCmd() {
    // ID、量程尚未确认时不发送，避免占位 ID 和错误量程产生误控制。
    if (!isConfigValid()) return;

    if (dwt_connect.GetDT() > config.timeout) is_connect = false;
    if (!is_enable) {
        send_cnt = 0;
        sendDisable();
        return;
    }
    if (err_code >= OVP) { // 故障时失能，不自动清除过流、过温等保护。
        send_cnt = 0;
        sendDisable();
        return;
    }

    // 第一次调用即发使能，不依赖先收到反馈，避免启动时互相等待。
    // 1 kHz 调用时每 100 ms 重发一次使能，重连期间只发送零扭矩。
    if (send_cnt++ % 100 == 0) {
        sendEnable();
    } else if (is_connect && err_code == ENABLED) {
        sendTorque(torque.ref);
    } else {
        sendTorque(0 * Nm);
    }
}

void DMJ6006::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    // 端口、ID、长度校验
    if (!isConfigValid()) return;
    if (port != config.can_port) return;
    if (id != config.master_id) return;
    if (dlc != 8) return;

    // D[0] 高四位为状态，低四位为电机 ID；完整反馈 ID 由上面的 master_id 校验。
    if ((data[0] & 0x0F) != (config.slave_id & 0x0F)) return;
    const auto state = (data[0] >> 4) & 0x0F;
    if (state != NONE && state != ENABLED && (state < OVP || state > OVERLOAD)) return;
    err_code = err_code_e(state);

    const uint16_t angle_u16 = (data[1] << 8) | data[2];
    const uint16_t speed_u12 = (data[3] << 4) | (data[4] >> 4);
    const uint16_t torque_u12 = ((data[4] & 0x0F) << 8) | data[5];
    temperature.mos = data[6] * C;
    temperature.motor = data[7] * C;

    // 双编码器反馈输出轴绝对位置，不像转子编码器那样再次除以内部减速比。
    const UnitFloat torque = uint_to_float(torque_u12, -protocol.t_max, protocol.t_max, 12) * Nm;
    const raw_t raw = {
        .current = torque / config.Kt,
        .torque = torque,
        .speed = uint_to_float(speed_u12, -protocol.v_max, protocol.v_max, 12) * rad_s,
        .angle = uint_to_float(angle_u16, -protocol.p_max, protocol.p_max, 16) * rad,
    };
    Motor::callback(raw);
}

float DMJ6006::uint_to_float(const int x_int, const float x_min, const float x_max, const int bits) {
    return (float)x_int * (x_max - x_min) / (float)((1 << bits) - 1) + x_min;
}

uint16_t DMJ6006::float_to_uint(const float x, const float x_min, const float x_max, const int bits) {
    return (uint16_t)((std::clamp(x, x_min, x_max) - x_min) * (float)((1 << bits) - 1) / (x_max - x_min));
}

void DMJ6006::sendEnable() const {
    uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
    BSP::CAN::Transmit(config.can_port, config.slave_id, data, 8);
}

void DMJ6006::sendDisable() const {
    uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
    BSP::CAN::Transmit(config.can_port, config.slave_id, data, 8);
}

void DMJ6006::sendTorque(const UnitFloat<>& torque) const {
    if (!std::isfinite(torque.toFloat(Nm))) {
        sendDisable();
        return;
    }

    // 保持主控角度闭环：MIT 内部 Kp、Kd 均为 0，仅使用扭矩前馈。
    // 这不同于 MIT 内部位置闭环，不把云台角度再交给电调计算一次。
    const uint16_t angle_u16 = float_to_uint(0, -protocol.p_max, protocol.p_max, 16);
    const uint16_t speed_u12 = float_to_uint(0, -protocol.v_max, protocol.v_max, 12);
    const uint16_t kp_u12 = 0;
    const uint16_t kd_u12 = 0;
    const float torque_max = std::min(protocol.t_max, MAX_TORQUE.toFloat(Nm));
    const float torque_limit = std::clamp(torque.toFloat(Nm), -torque_max, torque_max);
    const uint16_t torque_u12 = float_to_uint(config.is_invert ? -torque_limit : torque_limit,
                                             -protocol.t_max, protocol.t_max, 12);

    uint8_t data[8];
    data[0] = angle_u16 >> 8;
    data[1] = angle_u16;
    data[2] = speed_u12 >> 4;
    data[3] = ((speed_u12 & 0x0F) << 4) | (kp_u12 >> 8);
    data[4] = kp_u12;
    data[5] = kd_u12 >> 4;
    data[6] = ((kd_u12 & 0x0F) << 4) | (torque_u12 >> 8);
    data[7] = torque_u12;
    BSP::CAN::Transmit(config.can_port, config.slave_id, data, 8);
}
