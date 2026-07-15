#include <cmath>

#include "misc.hpp"
#include "chassis.hpp"
#include "imu.hpp"

// 四轮全向底盘 45 度安装时常用的 sqrt(2)/2 系数。
// 这里用静态变量保存，避免在 1 kHz 主循环中重复计算。
static const float sqrt2div2 = std::sqrt(2.0f) / 2.0f;
static const float sqrt2 = std::sqrt(2.0f);

// ==========================
// 整车主要限幅参数
// ==========================
//
// VXY_MAX：遥控器和 Ozone 给出的最大平移速度，单位 m/s。
// WR_MAX：遥控器和 Ozone 给出的最大自转速度，单位 rpm。
// WHEEL_TEST_SPEED_MAX：单轮测试模式下的轮端目标速度限幅。
static constexpr UnitFloat VXY_MAX = 4 * m_s;
static constexpr UnitFloat WR_MAX = 90 * rpm;
static constexpr UnitFloat WHEEL_TEST_SPEED_MAX = 400 * rpm;
static constexpr UnitFloat RC_SPIN_MID_SPEED = 30 * rpm;
static constexpr UnitFloat RC_SPIN_DOWN_SPEED = 60 * rpm;

// ==========================
// 云台转接板 CAN 协议
// ==========================
//
// - 控制帧 ID：0x1FF，当前只使用 data[0..1] 下发电流。
// - 反馈帧 ID：0x205~0x208，格式按转接板源码解析。
// - CAN 口：当前和底盘共用 CAN1。
// - 超时：100 ms 内没有收到反馈，则认为云台转接板断联。
static constexpr uint32_t GIMBAL_CONTROL_ID = 0x1FF;
static constexpr uint32_t GIMBAL_FEEDBACK_ID_MIN = 0x205;
static constexpr uint32_t GIMBAL_FEEDBACK_ID_MAX = 0x208;
static constexpr uint8_t GIMBAL_CAN_PORT = 1;
static constexpr UnitFloat GIMBAL_FEEDBACK_TIMEOUT = 100 * ms;

static void update_chassis_speed_observer();
static void update_gimbal_feedback_state();

// ==========================
// Ozone 可观察/可修改变量
// ==========================
//
// 1. Ozone 能直接在 Watched Data 中读取和修改。
// 2. volatile 防止编译器把调试变量优化掉。
// 3. 变量名都带 target_ 前缀，便于 Ozone 搜索。

// 通用测试输入。
// target_test_mode:
//   0      -> Ozone 直接给底盘 vx/vy/wr。
//   1~4    -> 单轮速度测试，对应 w1~w4。
//   11~14  -> 单轮开环电流测试，对应 w1~w4。
//   20     -> 遥控器控制，当前上电默认模式。
//   30     -> 云台开环电流测试。
volatile uint8_t target_is_enable = 0;
volatile uint8_t target_test_mode = 20;
volatile float target_vx_m_s = 0.0f;
volatile float target_vy_m_s = 0.0f;
volatile float target_wr_rpm = 0.0f;
volatile float target_wheel_rpm = 0.0f;
volatile float target_low_speed_disable_rpm = 5.0f;
volatile float target_speed_measure_deadband_rpm = 5.0f;
volatile float target_current_a = 0.0f;
volatile uint8_t target_is_world_frame_mode = 0;
// IMU 和 yaw 坐标。
// target_yaw_zero_deg 是上电时记录的车头方向，之后所有 yaw 都减去这个零点。
volatile float target_head_yaw_deg = 0.0f;
volatile float target_imu_yaw_deg = 0.0f;
volatile float target_yaw_zero_deg = 0.0f;
volatile float target_chassis_yaw_mount_offset_deg = 0.0f;
volatile float target_yaw_zero_lock_delay_ms = 1000.0f;
volatile uint8_t target_yaw_zero_is_locked = 0;
volatile uint8_t target_yaw_zero_reset_request = 0;

// 直线防偏航补偿。
// 当车正在平移且驾驶员没有主动打 yaw 时，锁住进入平移瞬间的 yaw，
// 用 yaw 误差额外叠加一个 wr，抵消打滑或受力不均带来的车头偏转。
volatile uint8_t target_yaw_comp_is_enable = 1;
volatile uint8_t target_yaw_comp_is_active = 0;
volatile float target_yaw_comp_ref_deg = 0.0f;
volatile float target_yaw_comp_err_deg = 0.0f;
volatile float target_yaw_comp_wr_rpm = 0.0f;
volatile float target_yaw_comp_kp_rpm_per_deg = -1.5f;
volatile float target_yaw_comp_max_rpm = 45.0f;
volatile float target_yaw_comp_move_deadband_m_s = 0.05f;
volatile float target_yaw_comp_manual_deadband_rpm = 3.0f;

// 底盘速度观测量。
// body_* 是车身坐标系速度；world_* 是用 IMU yaw 转到世界坐标系后的速度。
volatile float target_body_vx_measure_m_s = 0.0f;
volatile float target_body_vy_measure_m_s = 0.0f;
volatile float target_world_vx_measure_m_s = 0.0f;
volatile float target_world_vy_measure_m_s = 0.0f;
volatile float target_wr_measure_rpm = 0.0f;

// 四个轮子的目标速度，单位 rpm。用于 Ozone 对照 w*.speed.measure。
volatile float target_w1_rpm = 0.0f;
volatile float target_w2_rpm = 0.0f;
volatile float target_w3_rpm = 0.0f;
volatile float target_w4_rpm = 0.0f;

// 遥控器输入映射后的目标速度。
volatile float target_rc_vx_m_s = 0.0f;
volatile float target_rc_vy_m_s = 0.0f;
volatile float target_rc_wr_rpm = 0.0f;
volatile uint8_t target_rc_is_enable = 0;
volatile uint8_t target_rc_is_world_frame_mode = 0;
volatile uint8_t target_rc_spin_mode = 0;
volatile float target_rc_auto_wr_rpm = 0.0f;
volatile float target_rc_yaw_deadband_ratio = 0.08f;
volatile uint8_t target_is_gimbal_mode = 0;

// target 专用 SBUS 原始通道和拨杆状态。只用于 target 调试，不改 lib/rc 公用代码。
volatile uint16_t target_sbus_raw0 = 0;
volatile uint16_t target_sbus_raw1 = 0;
volatile uint16_t target_sbus_raw2 = 0;
volatile uint16_t target_sbus_raw3 = 0;
volatile uint16_t target_sbus_raw4 = 0;
volatile uint16_t target_sbus_raw5 = 0;
volatile uint16_t target_sbus_raw6 = 0;
volatile uint16_t target_sbus_raw7 = 0;
volatile uint16_t target_sbus_raw8 = 0;
volatile uint16_t target_sbus_raw9 = 0;
volatile uint16_t target_sbus_raw10 = 0;
volatile uint16_t target_sbus_raw11 = 0;
volatile uint16_t target_sbus_raw12 = 0;
volatile uint16_t target_sbus_raw13 = 0;
volatile uint16_t target_sbus_raw14 = 0;
volatile uint16_t target_sbus_raw15 = 0;
volatile uint8_t target_sbus_swa = FSi6X::ERR;
volatile uint8_t target_sbus_swb = FSi6X::ERR;
volatile uint8_t target_sbus_swc = FSi6X::ERR;
volatile uint8_t target_sbus_swd = FSi6X::ERR;

// 云台电机速度环和电流输出调试量。
volatile float target_gimbal_current_max_a = 5.0f;
volatile float target_gimbal_current_a = 0.0f;
volatile float target_gimbal_speed_ref_rpm = 0.0f;
volatile float target_gimbal_speed_measure_rpm = 0.0f;
volatile float target_gimbal_speed_raw_rpm = 0.0f;
volatile float target_gimbal_speed_filter_fc_hz = 20.0f;
volatile float target_gimbal_speed_pid_out_a = 0.0f;
volatile uint32_t target_gimbal_speed_pid_update_cnt = 0;
volatile float target_gimbal_speed_kp_a_per_rpm = 0.01f;
volatile float target_gimbal_speed_ki_a_per_rpm_s = 0.0f;
volatile float target_gimbal_speed_kd_a_per_rpm = 0.0f;
volatile float target_gimbal_speed_max_i_a = 0.5f;
volatile float target_gimbal_speed_max_out_a = 3.0f;
volatile float target_gimbal_speed_max_rpm = 100.0f;
volatile float target_gimbal_speed_err_rpm = 0.0f;
volatile int16_t target_gimbal_current_cmd_raw = 0;
volatile uint8_t target_gimbal_tx_data0 = 0;
volatile uint8_t target_gimbal_tx_data1 = 0;
volatile uint8_t target_gimbal_tx_data2 = 0;
volatile uint8_t target_gimbal_tx_data3 = 0;
volatile uint8_t target_gimbal_tx_data4 = 0;
volatile uint8_t target_gimbal_tx_data5 = 0;
volatile uint8_t target_gimbal_tx_data6 = 0;
volatile uint8_t target_gimbal_tx_data7 = 0;

// 云台转接板反馈量。
volatile float target_gimbal_angle_deg = 0.0f;
volatile float target_gimbal_speed_deg_s = 0.0f;
volatile float target_gimbal_current_measure_a = 0.0f;
volatile float target_gimbal_temperature_c = 0.0f;
volatile uint8_t target_gimbal_is_connect = 0;
volatile uint32_t target_gimbal_rx_cnt = 0;
volatile uint8_t target_gimbal_rx_data0 = 0;
volatile uint8_t target_gimbal_rx_data1 = 0;
volatile uint8_t target_gimbal_rx_data2 = 0;
volatile uint8_t target_gimbal_rx_data3 = 0;
volatile uint8_t target_gimbal_rx_data4 = 0;
volatile uint8_t target_gimbal_rx_data5 = 0;
volatile uint8_t target_gimbal_rx_data6 = 0;
volatile uint8_t target_gimbal_rx_data7 = 0;
volatile uint32_t target_gimbal_feedback_id = 0;
volatile float target_gimbal_rx_dt_ms = 0.0f;
volatile float target_gimbal_last_rx_interval_ms = 0.0f;

// CAN1 调试量：用来确认 C 板有没有收到某个 ID、最后一帧数据是什么。
volatile uint32_t target_can1_rx_cnt = 0;
volatile uint32_t target_can1_last_id = 0;
volatile uint8_t target_can1_last_dlc = 0;
volatile uint8_t target_can1_data0 = 0;
volatile uint8_t target_can1_data1 = 0;
volatile uint8_t target_can1_data2 = 0;
volatile uint8_t target_can1_data3 = 0;
volatile uint8_t target_can1_data4 = 0;
volatile uint8_t target_can1_data5 = 0;
volatile uint8_t target_can1_data6 = 0;
volatile uint8_t target_can1_data7 = 0;

// CAN 发送节奏。
// 底盘和云台共用 CAN1，为了避免邮箱拥塞，发送前会保证任意两包之间至少间隔
// target_can_packet_gap_us；两个 interval 分别控制云台包和底盘包的周期。
volatile float target_gimbal_can_interval_ms = 2.0f;
volatile float target_chassis_can_interval_ms = 1.0f;
volatile float target_can_packet_gap_us = 200.0f;
volatile uint32_t target_gimbal_can_tx_cnt = 0;
volatile uint32_t target_chassis_can_tx_cnt = 0;

// 对称限幅：把 value 限制在 [-limit_abs, +limit_abs]。
static float limit(const float value, const float limit_abs) {
    if (value > limit_abs) return limit_abs;
    if (value < -limit_abs) return -limit_abs;
    return value;
}

// 区间限幅：把 value 限制在 [min_value, max_value]。
static float limit_range(const float value, const float min_value, const float max_value) {
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

// 遥控器左摇杆水平死区。
// 普通模式下左摇杆控制底盘自转，但摇杆中位会有轻微漂移；
// 小于该死区时当成 0，避免手碰摇杆或中位误差导致车头慢慢偏。
static float apply_rc_yaw_deadband(const float yaw_ratio) {
    const float deadband = limit_range(target_rc_yaw_deadband_ratio, 0.0f, 0.5f);
    return std::abs(yaw_ratio) < deadband ? 0.0f : yaw_ratio;
}

class TargetFSi6X {
public:
    bool is_connect = false;
    UnitFloat<ratio> x = 0 * ratio;
    UnitFloat<ratio> y = 0 * ratio;
    UnitFloat<ratio> pitch = 0 * ratio;
    UnitFloat<ratio> yaw = 0 * ratio;
    FSi6X::switch_e swa = FSi6X::ERR;
    FSi6X::switch_e swb = FSi6X::ERR;
    FSi6X::switch_e swc = FSi6X::ERR;
    FSi6X::switch_e swd = FSi6X::ERR;
    uint16_t raw[16]{};

    TargetFSi6X() {
        auto callback = [this](const uint8_t data[], const uint16_t size) {
            this->callback(data, size);
        };
        BSP::UART3::RegisterCallback(callback);
    }

    void OnLoop() {
        if (dwt_connect.GetDT() > 100 * ms) {
            resetData();
        }
        updateDebugVariables();
    }

private:
    BSP::Dwt dwt_connect;

    void callback(const uint8_t data[], const uint16_t size) {
        if (size != 25) return;
        if (data[0] != 0x0F) return;

        raw[0] = ((data[1] | data[2] << 8) & 0x07FF);
        raw[1] = ((data[2] >> 3 | data[3] << 5) & 0x07FF);
        raw[2] = ((data[3] >> 6 | data[4] << 2 | data[5] << 10) & 0x07FF);
        raw[3] = ((data[5] >> 1 | data[6] << 7) & 0x07FF);
        raw[4] = ((data[6] >> 4 | data[7] << 4) & 0x07FF);
        raw[5] = ((data[7] >> 7 | data[8] << 1 | data[9] << 9) & 0x07FF);
        raw[6] = ((data[9] >> 2 | data[10] << 6) & 0x07FF);
        raw[7] = ((data[10] >> 5 | data[11] << 3) & 0x07FF);
        raw[8] = ((data[12] | data[13] << 8) & 0x07FF);
        raw[9] = ((data[13] >> 3 | data[14] << 5) & 0x07FF);
        raw[10] = ((data[14] >> 6 | data[15] << 2 | data[16] << 10) & 0x07FF);
        raw[11] = ((data[16] >> 1 | data[17] << 7) & 0x07FF);
        raw[12] = ((data[17] >> 4 | data[18] << 4) & 0x07FF);
        raw[13] = ((data[18] >> 7 | data[19] << 1 | data[20] << 9) & 0x07FF);
        raw[14] = ((data[20] >> 2 | data[21] << 6) & 0x07FF);
        raw[15] = ((data[21] >> 5 | data[22] << 3) & 0x07FF);

        const bool failsafe = (data[23] & 0x08) != 0;
        if (failsafe) {
            resetData();
            return;
        }

        y = -getJoystick(raw[0]);
        x = getJoystick(raw[1]);
        pitch = getJoystick(raw[2]);
        yaw = -getJoystick(raw[3]);
        swa = getSwitch(raw[4]);
        swb = getSwitch(raw[5]);
        swc = getSwitch(raw[6]);
        swd = getSwitch(raw[7]);

        is_connect = true;
        dwt_connect.UpdateDT();
        updateDebugVariables();
    }

    UnitFloat<> getJoystick(const uint16_t value) const {
        const float mid = 1024.0f;
        const float min_span = mid - 240.0f;
        const float max_span = 1807.0f - mid;
        const float span = value >= 1024 ? max_span : min_span;
        UnitFloat<> ret = span > 1.0f ? ((float)value - mid) / span * ratio : 0 * ratio;
        ret = unit::clamp(ret, -1 * ratio, 1 * ratio);
        if (unit::abs(ret) < 5 * pct) ret = 0 * ratio;
        return ret;
    }

    static FSi6X::switch_e getSwitch(const uint16_t value) {
        if (value < 650) return FSi6X::UP;
        if (value < 1350) return FSi6X::MID;
        if (value <= 1900) return FSi6X::DOWN;
        return FSi6X::ERR;
    }

    void resetData() {
        is_connect = false;
        x = y = pitch = yaw = 0 * ratio;
        swa = swb = swc = swd = FSi6X::ERR;
        for (uint8_t i = 0; i < 16; i++) raw[i] = 0;
        updateDebugVariables();
    }

    void updateDebugVariables() const {
        target_sbus_raw0 = raw[0];
        target_sbus_raw1 = raw[1];
        target_sbus_raw2 = raw[2];
        target_sbus_raw3 = raw[3];
        target_sbus_raw4 = raw[4];
        target_sbus_raw5 = raw[5];
        target_sbus_raw6 = raw[6];
        target_sbus_raw7 = raw[7];
        target_sbus_raw8 = raw[8];
        target_sbus_raw9 = raw[9];
        target_sbus_raw10 = raw[10];
        target_sbus_raw11 = raw[11];
        target_sbus_raw12 = raw[12];
        target_sbus_raw13 = raw[13];
        target_sbus_raw14 = raw[14];
        target_sbus_raw15 = raw[15];
        target_sbus_swa = swa;
        target_sbus_swb = swb;
        target_sbus_swc = swc;
        target_sbus_swd = swd;
    }
};

// DWT 计时器。PollTimeout() 用于固定周期执行，UpdateDT()/GetDT() 用于测量时间间隔。
static TargetFSi6X target_rc;
static BSP::Dwt gimbal_feedback_dwt;
static BSP::Dwt gimbal_can_send_dwt;
static BSP::Dwt chassis_can_send_dwt;
static BSP::Dwt can_packet_gap_dwt;
static BSP::Dwt gimbal_speed_pid_dwt;
static BSP::Dwt yaw_zero_lock_dwt;
static bool yaw_zero_lock_timer_is_started = false;

// 云台速度估计和 PID 内部状态。
static bool gimbal_angle_is_ready = false;
static float gimbal_last_angle_deg = 0.0f;
static float gimbal_speed_i_out_a = 0.0f;
static float gimbal_speed_last_err_rpm = 0.0f;
static uint32_t gimbal_speed_pid_last_rx_cnt = 0;
static UnitFloat<> gimbal_speed_measure_filtered = 0 * rpm;

// target 使用 FS-i6X SBUS，串口参数只在本应用里重配，避免改动 Core/Src/usart.c 影响其他车辆。
static void configure_target_sbus_uart3() {
    HAL_UART_AbortReceive(&huart3);
    HAL_UART_DeInit(&huart3);
    huart3.Init.BaudRate = 100000;
    huart3.Init.WordLength = UART_WORDLENGTH_9B;
    huart3.Init.StopBits = UART_STOPBITS_2;
    huart3.Init.Parity = UART_PARITY_EVEN;
    huart3.Init.Mode = UART_MODE_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
}

// 保证任意两帧 CAN 发送之间有最小间隔。
// 这是为了给底盘 0x200 帧和云台 0x1FF 帧留出邮箱/总线余量。
static void wait_can_packet_gap() {
    const float gap_us = limit_range(target_can_packet_gap_us, 0.0f, 1000.0f);
    if (gap_us <= 0.0f) {
        can_packet_gap_dwt.UpdateDT();
        return;
    }

    const UnitFloat gap = (gap_us / 1000000.0f) * s;
    while (can_packet_gap_dwt.GetDT() < gap) {}
    can_packet_gap_dwt.UpdateDT();
}

// 写云台目标电流，并做最大电流限幅。
static void set_gimbal_current(const UnitFloat<>& current) {
    target_gimbal_current_a = limit(current.toFloat(A), target_gimbal_current_max_a);
}

// 云台速度环。
// speed_ratio 来自摇杆，范围约为 [-1, 1]，再乘 target_gimbal_speed_max_rpm。
// 速度反馈来自 0x205~0x208 位置差分，而 PID 输出是转接板接收的电流命令。
static void set_gimbal_speed(const bool is_enable, const float speed_ratio) {
    const float speed_ref_rpm = limit(speed_ratio, 1.0f) * limit_range(target_gimbal_speed_max_rpm, 0.0f, 500.0f);
    const float speed_measure_rpm = gimbal_speed_measure_filtered.toFloat(rpm);

    target_gimbal_speed_ref_rpm = speed_ref_rpm;
    target_gimbal_speed_measure_rpm = speed_measure_rpm;

    // 未使能或断联时立即清积分、清误差、输出 0 电流，避免恢复连接后突然冲一下。
    if (!is_enable || !target_gimbal_is_connect) {
        gimbal_speed_i_out_a = 0.0f;
        gimbal_speed_last_err_rpm = 0.0f;
        gimbal_speed_pid_last_rx_cnt = target_gimbal_rx_cnt;
        target_gimbal_speed_err_rpm = 0.0f;
        target_gimbal_speed_pid_out_a = 0.0f;
        set_gimbal_current(0 * A);
        return;
    }

    // 速度环只在收到新的云台反馈后更新一次。
    // 这样不会在 1 kHz 主循环里反复拿旧反馈积分，降低卡顿和过冲风险。
    if (gimbal_speed_pid_last_rx_cnt == target_gimbal_rx_cnt) return;
    gimbal_speed_pid_last_rx_cnt = target_gimbal_rx_cnt;

    const float dt_s = limit_range(gimbal_speed_pid_dwt.UpdateDT().toFloat(s), 0.0001f, 0.05f);
    const float err_rpm = speed_ref_rpm - speed_measure_rpm;
    const float derr_rpm = err_rpm - gimbal_speed_last_err_rpm;
    gimbal_speed_last_err_rpm = err_rpm;

    // I 项以“电流”为单位保存，max_i 也是电流限幅。
    gimbal_speed_i_out_a += target_gimbal_speed_ki_a_per_rpm_s * err_rpm * dt_s;
    gimbal_speed_i_out_a = limit(gimbal_speed_i_out_a, target_gimbal_speed_max_i_a);

    const float current_out_a = target_gimbal_speed_kp_a_per_rpm * err_rpm
                                + gimbal_speed_i_out_a
                                + target_gimbal_speed_kd_a_per_rpm * derr_rpm;

    target_gimbal_speed_err_rpm = err_rpm;
    target_gimbal_speed_pid_out_a = limit(current_out_a, target_gimbal_speed_max_out_a);
    target_gimbal_speed_pid_update_cnt++;
    set_gimbal_current(target_gimbal_speed_pid_out_a * A);
}

// 发送云台电流命令。
// 当前转接板只接收 0x1FF，但会按自身反馈 ID 选择电流槽位：
// 0x205 -> data[0..1]，0x206 -> data[2..3]，0x207 -> data[4..5]，0x208 -> data[6..7]。
// 现在云台地址未知，所以四个槽位都填同一个电流，哪块转接板在线都能转。
static void send_gimbal_can_cmd() {
    const int16_t current_cmd = (int16_t)(limit(target_gimbal_current_a, target_gimbal_current_max_a) * 100.0f);
    uint8_t data[8] = {};
    data[0] = current_cmd >> 8;
    data[1] = current_cmd;
    data[2] = current_cmd >> 8;
    data[3] = current_cmd;
    data[4] = current_cmd >> 8;
    data[5] = current_cmd;
    data[6] = current_cmd >> 8;
    data[7] = current_cmd;
    target_gimbal_current_cmd_raw = current_cmd;
    target_gimbal_tx_data0 = data[0];
    target_gimbal_tx_data1 = data[1];
    target_gimbal_tx_data2 = data[2];
    target_gimbal_tx_data3 = data[3];
    target_gimbal_tx_data4 = data[4];
    target_gimbal_tx_data5 = data[5];
    target_gimbal_tx_data6 = data[6];
    target_gimbal_tx_data7 = data[7];
    wait_can_packet_gap();
    BSP::CAN::Transmit(GIMBAL_CAN_PORT, GIMBAL_CONTROL_ID, data);
    target_gimbal_can_tx_cnt++;
}

// CAN 接收回调。
// 所有 CAN1 数据都会记录到 target_can1_*，便于 Ozone 判断总线上是否有数据。
// 只有 ID=0x205~0x208 且 dlc=8 的帧会继续当成云台反馈解析。
static void gimbal_feedback_callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    if (port == GIMBAL_CAN_PORT) {
        target_can1_rx_cnt++;
        target_can1_last_id = id;
        target_can1_last_dlc = dlc;
        target_can1_data0 = data[0];
        target_can1_data1 = data[1];
        target_can1_data2 = data[2];
        target_can1_data3 = data[3];
        target_can1_data4 = data[4];
        target_can1_data5 = data[5];
        target_can1_data6 = data[6];
        target_can1_data7 = data[7];
    }

    if (port != GIMBAL_CAN_PORT) return;
    if (id < GIMBAL_FEEDBACK_ID_MIN || id > GIMBAL_FEEDBACK_ID_MAX) return;
    if (dlc != 8) return;

    target_gimbal_rx_cnt++;
    target_gimbal_feedback_id = id;
    target_gimbal_rx_data0 = data[0];
    target_gimbal_rx_data1 = data[1];
    target_gimbal_rx_data2 = data[2];
    target_gimbal_rx_data3 = data[3];
    target_gimbal_rx_data4 = data[4];
    target_gimbal_rx_data5 = data[5];
    target_gimbal_rx_data6 = data[6];
    target_gimbal_rx_data7 = data[7];
    target_gimbal_rx_dt_ms = 0.0f;
    const UnitFloat rx_dt = gimbal_feedback_dwt.UpdateDT();
    target_gimbal_last_rx_interval_ms = rx_dt.toFloat(ms);

    // 转接板反馈格式：
    // data[0..1]：uint16 角度，单位 0.01 deg。
    // data[2..3]：int16 速度，单位 deg/s，当前只记录，不直接作为速度环反馈。
    // data[4..5]：int16 电流，单位 0.01 A。
    // data[6]：int8 温度，单位摄氏度。
    const uint16_t angle_u16 = (uint16_t)((data[0] << 8) | data[1]);
    const int16_t speed_i16 = (int16_t)((data[2] << 8) | data[3]);
    const int16_t current_i16 = (int16_t)((data[4] << 8) | data[5]);
    const int8_t temperature_i8 = (int8_t)data[6];

    target_gimbal_angle_deg = (float)angle_u16 * 0.01f;
    target_gimbal_speed_deg_s = (float)speed_i16;
    target_gimbal_current_measure_a = (float)current_i16 * 0.01f;
    target_gimbal_temperature_c = (float)temperature_i8;

    // 用角度差分估算云台速度。
    // 第一次收到反馈、dt 异常或超时后重新收到反馈时，不用旧角度算速度，避免跳变。
    if (!gimbal_angle_is_ready || rx_dt <= 0 * ms || rx_dt > GIMBAL_FEEDBACK_TIMEOUT) {
        gimbal_angle_is_ready = true;
        gimbal_last_angle_deg = target_gimbal_angle_deg;
        gimbal_speed_measure_filtered = 0 * rpm;
        target_gimbal_speed_raw_rpm = 0.0f;
    } else {
        float delta_angle_deg = target_gimbal_angle_deg - gimbal_last_angle_deg;
        if (delta_angle_deg > 180.0f) delta_angle_deg -= 360.0f;
        if (delta_angle_deg < -180.0f) delta_angle_deg += 360.0f;
        gimbal_last_angle_deg = target_gimbal_angle_deg;

        const UnitFloat speed_raw = (delta_angle_deg * deg) / rx_dt;
        target_gimbal_speed_raw_rpm = speed_raw.toFloat(rpm);

        // 一阶低通滤波。fc 越小越平滑但延迟越大；fc=0 时关闭滤波。
        const float fc_hz = limit_range(target_gimbal_speed_filter_fc_hz, 0.0f, 100.0f);
        if (fc_hz <= 0.0f) {
            gimbal_speed_measure_filtered = speed_raw;
        } else {
            const float dt_s = rx_dt.toFloat(s);
            const float rc_s = 1.0f / (6.28318530718f * fc_hz);
            const float alpha = dt_s / (rc_s + dt_s);
            gimbal_speed_measure_filtered = gimbal_speed_measure_filtered * (1.0f - alpha) + speed_raw * alpha;
        }
    }

    target_gimbal_is_connect = 1;
}

// 主循环中周期检查云台反馈超时。
static void update_gimbal_feedback_state() {
    target_gimbal_rx_dt_ms = gimbal_feedback_dwt.GetDT().toFloat(ms);
    if (gimbal_feedback_dwt.GetDT() > GIMBAL_FEEDBACK_TIMEOUT) {
        target_gimbal_is_connect = 0;
    }
}

// 设置单个底盘轮速度，同时保留低速失能策略。
// 目标速度和测量速度都低于阈值时才失能，避免 N630 HFI 在静止低速附近发出高频噪音。
// 如果目标为 0 但轮子还在转，则仍然使能并给 0 rpm，让速度环提供刹车能力。
static void set_wheel_speed_with_low_speed_brake(M3508& motor, const bool chassis_enable, const UnitFloat<>& wheel_speed) {
    const UnitFloat low_speed_disable = target_low_speed_disable_rpm * rpm;
    const bool is_target_low_speed = unit::abs(wheel_speed) < low_speed_disable;
    const bool is_measure_low_speed = unit::abs(motor.speed.measure) < low_speed_disable;
    const bool motor_enable = chassis_enable && !(is_target_low_speed && is_measure_low_speed);
    const UnitFloat speed_ref = is_target_low_speed ? 0 * rpm : wheel_speed;

    motor.SetEnable(motor_enable);
    motor.SetSpeed(motor_enable ? speed_ref : 0 * rpm);
}

// N630 在 0 rpm 附近可能有轻微反馈抖动。
// 进入速度 PID 前做测量死区，避免目标 0 rpm 时被测量噪声反复触发小电流。
static void apply_speed_measure_deadband(M3508& motor) {
    const UnitFloat deadband = target_speed_measure_deadband_rpm * rpm;
    if (unit::abs(motor.speed.measure) < deadband) {
        motor.speed.measure = 0 * rpm;
    }
}

static void apply_chassis_speed_measure_deadband() {
    apply_speed_measure_deadband(w1);
    apply_speed_measure_deadband(w2);
    apply_speed_measure_deadband(w3);
    apply_speed_measure_deadband(w4);
}

// 在两套底盘控制链路之间切换电机工作模式。
// 底盘急停/失能。
// 这里不仅清目标速度和电流，也会跑一次电机 OnLoop，让 0 输出尽快进入 CAN 命令缓存。
static void stop_chassis() {
    chassis.SetEnable(false);
    w1.SetEnable(false);
    w2.SetEnable(false);
    w3.SetEnable(false);
    w4.SetEnable(false);

    w1.SetSpeed(0 * rpm);
    w2.SetSpeed(0 * rpm);
    w3.SetSpeed(0 * rpm);
    w4.SetSpeed(0 * rpm);
    w1.SetCurrent(0 * A);
    w2.SetCurrent(0 * A);
    w3.SetCurrent(0 * A);
    w4.SetCurrent(0 * A);

    apply_chassis_speed_measure_deadband();
    w1.OnLoop();
    w2.OnLoop();
    w3.OnLoop();
    w4.OnLoop();

    target_w1_rpm = target_w2_rpm = target_w3_rpm = target_w4_rpm = 0.0f;
    target_rc_vx_m_s = target_rc_vy_m_s = target_rc_wr_rpm = 0.0f;
    target_yaw_comp_is_active = 0;
    target_yaw_comp_ref_deg = (imu.yaw + target_chassis_yaw_mount_offset_deg * deg - target_yaw_zero_deg * deg).toFloat(deg);
    target_yaw_comp_err_deg = 0.0f;
    target_yaw_comp_wr_rpm = 0.0f;
    update_chassis_speed_observer();
}

// 当前底盘 yaw：IMU 原始 yaw 减去上电零点，再补偿 IMU/车头之间的安装偏角。
// target_chassis_yaw_mount_offset_deg 默认按实车测试设置为 -45。
// 如果后续重新安装 C 板，优先在 Ozone 中微调这个值。
static Angle<deg> get_chassis_yaw() {
    return imu.yaw + target_chassis_yaw_mount_offset_deg * deg - target_yaw_zero_deg * deg;
}

// 上电后 IMU 姿态解算需要一点时间稳定。
// 在底盘未使能且未超过锁定延时前，持续把当前 yaw 作为零点；
// 一旦底盘使能或延时结束，就锁住零点，保证世界系前方等于稳定后的上电车头方向。
static void update_yaw_zero_capture(const bool is_chassis_enable_requested) {
    if (target_yaw_zero_reset_request != 0) {
        target_yaw_zero_reset_request = 0;
        target_yaw_zero_is_locked = 0;
        yaw_zero_lock_timer_is_started = false;
        yaw_zero_lock_dwt.Reset();
    }

    if (!yaw_zero_lock_timer_is_started) {
        yaw_zero_lock_timer_is_started = true;
        yaw_zero_lock_dwt.Reset();
        target_yaw_zero_is_locked = 0;
    }

    const float lock_delay_ms = limit_range(target_yaw_zero_lock_delay_ms, 0.0f, 5000.0f);
    const bool is_timeout = yaw_zero_lock_dwt.GetDT() >= lock_delay_ms * ms;

    if (target_yaw_zero_is_locked == 0 && !is_chassis_enable_requested && !is_timeout) {
        target_yaw_zero_deg = imu.yaw.toFloat(deg);
        target_yaw_comp_is_active = 0;
        target_yaw_comp_ref_deg = 0.0f;
        target_yaw_comp_err_deg = 0.0f;
        target_yaw_comp_wr_rpm = 0.0f;
        return;
    }

    target_yaw_zero_is_locked = 1;
}

// 把角度压回 (-180, 180]，用于 yaw 误差计算。
static float wrap_angle_deg(float angle_deg) {
    while (angle_deg <= -180.0f) angle_deg += 360.0f;
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    return angle_deg;
}

// 直线 yaw 补偿。
// 只有同时满足以下条件才工作：
// 1. target_yaw_comp_is_enable 打开。
// 2. 当前正在平移，速度超过 target_yaw_comp_move_deadband_m_s。
// 3. 驾驶员没有主动给自转，wr 小于 target_yaw_comp_manual_deadband_rpm。
// 4. 外部允许补偿，例如遥控器连接且底盘使能。
static UnitFloat<> apply_yaw_compensation(const UnitFloat<>& vx,
                                          const UnitFloat<>& vy,
                                          const UnitFloat<>& wr,
                                          const bool allow_compensation) {
    const float move_speed_m_s = std::sqrt(vx.toFloat(m_s) * vx.toFloat(m_s) + vy.toFloat(m_s) * vy.toFloat(m_s));
    const bool is_moving = move_speed_m_s > limit_range(target_yaw_comp_move_deadband_m_s, 0.0f, 1.0f);
    const bool is_manual_rotate = std::abs(wr.toFloat(rpm)) > limit_range(target_yaw_comp_manual_deadband_rpm, 0.0f, 30.0f);
    const bool is_enable = (target_yaw_comp_is_enable != 0) && allow_compensation && is_moving && !is_manual_rotate;
    const float yaw_deg = get_chassis_yaw().toFloat(deg);

    // 不满足补偿条件时，重置锁定角。下次重新开始平移时会用新的车头方向做参考。
    if (!is_enable) {
        target_yaw_comp_is_active = 0;
        target_yaw_comp_ref_deg = yaw_deg;
        target_yaw_comp_err_deg = 0.0f;
        target_yaw_comp_wr_rpm = 0.0f;
        return wr;
    }

    if (target_yaw_comp_is_active == 0) {
        target_yaw_comp_ref_deg = yaw_deg;
        target_yaw_comp_is_active = 1;
    }

    target_yaw_comp_err_deg = wrap_angle_deg(target_yaw_comp_ref_deg - yaw_deg);
    target_yaw_comp_wr_rpm = limit(target_yaw_comp_err_deg * target_yaw_comp_kp_rpm_per_deg,
                                   limit_range(target_yaw_comp_max_rpm, 0.0f, WR_MAX.toFloat(rpm)));
    return wr + target_yaw_comp_wr_rpm * rpm;
}

// 底盘速度观测。
// 根据四个轮子的测量速度反解出车身坐标系 vx/vy/wr，
// 再使用 IMU yaw 转成世界坐标系速度
static void update_chassis_speed_observer() {
    const UnitFloat<m_s> v1 = w1.speed.measure * WHEEL_RADIUS;
    const UnitFloat<m_s> v2 = w2.speed.measure * WHEEL_RADIUS;
    const UnitFloat<m_s> v3 = w3.speed.measure * WHEEL_RADIUS;
    const UnitFloat<m_s> v4 = w4.speed.measure * WHEEL_RADIUS;

    const UnitFloat<m_s> body_vx = sqrt2 * (-v1 - v2 + v3 + v4) / 4.0f;
    const UnitFloat<m_s> body_vy = sqrt2 * (+v1 - v2 - v3 + v4) / 4.0f;
    const UnitFloat<m_s> body_vz = (+v1 + v2 + v3 + v4) / 4.0f;
    const UnitFloat<rpm> wr = body_vz / CHASSIS_RADIUS;
    auto [world_vx, world_vy] = unit::rotate(body_vx, body_vy, get_chassis_yaw());

    target_body_vx_measure_m_s = body_vx.toFloat(m_s);
    target_body_vy_measure_m_s = body_vy.toFloat(m_s);
    target_world_vx_measure_m_s = world_vx.toFloat(m_s);
    target_world_vy_measure_m_s = world_vy.toFloat(m_s);
    target_wr_measure_rpm = wr.toFloat(rpm);
}

// 统一发送 CAN 命令。
// 云台包和底盘包都在这里发，避免不同位置同时抢 CAN 邮箱。
void send_can_cmd() {
    const float gimbal_interval_ms = limit_range(target_gimbal_can_interval_ms, 1.0f, 100.0f);
    const float chassis_interval_ms = limit_range(target_chassis_can_interval_ms, 1.0f, 100.0f);
    const bool need_send_gimbal = gimbal_can_send_dwt.PollTimeout(gimbal_interval_ms * ms);
    const bool need_send_chassis = chassis_can_send_dwt.PollTimeout(chassis_interval_ms * ms);

    if (need_send_gimbal) {
        send_gimbal_can_cmd();
    }
    if (!need_send_chassis) return;

    // N630 已改成 C620/M3508 帧格式：
    // 反馈来自 0x201~0x204，控制用 0x200 一帧打包四个 int16 电流。
    const int16_t cmd1 = w1.GetCanCmd();
    const int16_t cmd2 = w2.GetCanCmd();
    const int16_t cmd3 = w3.GetCanCmd();
    const int16_t cmd4 = w4.GetCanCmd();

    uint8_t data[8];
    data[0] = cmd1 >> 8;
    data[1] = cmd1;
    data[2] = cmd2 >> 8;
    data[3] = cmd2;
    data[4] = cmd3 >> 8;
    data[5] = cmd3;
    data[6] = cmd4 >> 8;
    data[7] = cmd4;

    wait_can_packet_gap();
    BSP::CAN::Transmit(CHASSIS_CAN_PORT, CHASSIS_CONTROL_ID, data, 8);
    target_chassis_can_tx_cnt++;
}

// 底盘直接速度控制。
// 输入：
// - vx/vy：平移速度。车身坐标模式下表示车头方向的前后/左右；
//          世界坐标模式下会先用 IMU yaw 转回车身坐标，用于实现边转边沿固定方向平移。
// - wr：底盘自转速度。
//
// 输出：
// - 解算成四个轮子的轮端目标转速。
// - 每个 M3508 对象在 OnLoop() 内执行速度 PID，输出电流。
void set_chassis_speed_direct(const bool is_enable,
                              const bool is_world_frame_mode,
                              const UnitFloat<>& vx,
                              const UnitFloat<>& vy,
                              const UnitFloat<>& wr) {
    auto [body_vx, body_vy] = is_world_frame_mode ? unit::rotate(vx, vy, -get_chassis_yaw()) : std::make_pair(vx, vy);

    const UnitFloat vz = wr * CHASSIS_RADIUS;
    const UnitFloat<m_s> v1 = sqrt2div2 * (-body_vx + body_vy) + vz;
    const UnitFloat<m_s> v2 = sqrt2div2 * (-body_vx - body_vy) + vz;
    const UnitFloat<m_s> v3 = sqrt2div2 * (+body_vx - body_vy) + vz;
    const UnitFloat<m_s> v4 = sqrt2div2 * (+body_vx + body_vy) + vz;
    const UnitFloat w1_speed = v1 / WHEEL_RADIUS;
    const UnitFloat w2_speed = v2 / WHEEL_RADIUS;
    const UnitFloat w3_speed = v3 / WHEEL_RADIUS;
    const UnitFloat w4_speed = v4 / WHEEL_RADIUS;

    target_w1_rpm = w1_speed.toFloat(rpm);
    target_w2_rpm = w2_speed.toFloat(rpm);
    target_w3_rpm = w3_speed.toFloat(rpm);
    target_w4_rpm = w4_speed.toFloat(rpm);

    set_wheel_speed_with_low_speed_brake(w1, is_enable, w1_speed);
    set_wheel_speed_with_low_speed_brake(w2, is_enable, w2_speed);
    set_wheel_speed_with_low_speed_brake(w3, is_enable, w3_speed);
    set_wheel_speed_with_low_speed_brake(w4, is_enable, w4_speed);

    apply_chassis_speed_measure_deadband();
    w1.OnLoop();
    w2.OnLoop();
    w3.OnLoop();
    w4.OnLoop();
    update_chassis_speed_observer();
}

// 单轮测试模式。
// 速度测试：target_test_mode=1~4，target_wheel_rpm 给目标速度。
// 电流测试：target_test_mode=11~14，target_current_a 给开环电流。
void set_single_wheel_speed() {
    const UnitFloat wheel_speed = limit(target_wheel_rpm, WHEEL_TEST_SPEED_MAX.toFloat(rpm)) * rpm;
    const UnitFloat current = limit(target_current_a, M3508::MAX_CURRENT.toFloat(A)) * A;
    const bool is_enable = target_is_enable != 0;

    if (target_test_mode >= 11 && target_test_mode <= 14) {
        w1.SetEnable(is_enable && target_test_mode == 11);
        w2.SetEnable(is_enable && target_test_mode == 12);
        w3.SetEnable(is_enable && target_test_mode == 13);
        w4.SetEnable(is_enable && target_test_mode == 14);

        w1.SetCurrent(target_test_mode == 11 ? current : 0 * A);
        w2.SetCurrent(target_test_mode == 12 ? current : 0 * A);
        w3.SetCurrent(target_test_mode == 13 ? current : 0 * A);
        w4.SetCurrent(target_test_mode == 14 ? current : 0 * A);
    } else {
        set_wheel_speed_with_low_speed_brake(w1, is_enable && target_test_mode == 1, wheel_speed);
        set_wheel_speed_with_low_speed_brake(w2, is_enable && target_test_mode == 2, wheel_speed);
        set_wheel_speed_with_low_speed_brake(w3, is_enable && target_test_mode == 3, wheel_speed);
        set_wheel_speed_with_low_speed_brake(w4, is_enable && target_test_mode == 4, wheel_speed);

        apply_chassis_speed_measure_deadband();
        w1.OnLoop();
        w2.OnLoop();
        w3.OnLoop();
        w4.OnLoop();
        update_chassis_speed_observer();
    }
}

void setup() {
    BSP::Init();
    configure_target_sbus_uart3();
    BSP::UART3::Init(); // target 当前使用 FS-i6X SBUS，接在 DJI C 板 UART3。
    BSP::CAN::RegisterCallback(gimbal_feedback_callback);

    // 上电时把当前车头方向记为世界坐标零点。
    // 这样世界坐标模式的“前进”默认和刚上电时车身前方一致。
    imu.OnLoop();
    target_yaw_zero_deg = imu.yaw.toFloat(deg);
    target_yaw_zero_is_locked = 0;
    target_yaw_zero_reset_request = 0;
    yaw_zero_lock_timer_is_started = false;
    yaw_zero_lock_dwt.Reset();

    // 当前 target 只使用本文件的手写四轮解算。
    // chassis 对象保留在 chassis.hpp 中，方便以后需要时再接回框架。
}

void loop() {
    imu.OnLoop();
    target_rc.OnLoop();
    const bool is_chassis_enable_requested = target_test_mode == 20 ? target_rc.is_connect && target_rc.swa == FSi6X::DOWN : target_is_enable != 0;
    update_yaw_zero_capture(is_chassis_enable_requested);
    target_imu_yaw_deg = get_chassis_yaw().toFloat(deg);
    update_gimbal_feedback_state();

    if (target_test_mode == 0) {
        // Ozone 直接速度模式。适合不接遥控器时测试底盘解算和 PID。
        const UnitFloat vx = limit(target_vx_m_s, VXY_MAX.toFloat(m_s)) * m_s;
        const UnitFloat vy = limit(target_vy_m_s, VXY_MAX.toFloat(m_s)) * m_s;
        const UnitFloat wr = limit(target_wr_rpm, WR_MAX.toFloat(rpm)) * rpm;
        const UnitFloat wr_with_compensation = apply_yaw_compensation(vx, vy, wr, target_is_enable != 0);
        set_chassis_speed_direct(target_is_enable != 0, target_is_world_frame_mode != 0, vx, vy, wr_with_compensation);
    } else if (target_test_mode == 20) {
        // 遥控器模式：
        // - SWA：下档使能，上档失能。
        // - SWC：上档正常模式，中档 30rpm 自动旋转，下档 60rpm 自动旋转。
        // - SWB：下档进入云台速度模式，左摇杆水平控制云台，不再控制底盘自转。
        // - 右摇杆：普通模式按当前车身方向平移；自动旋转时按上电车头方向平移。
        // - 左摇杆水平：SWC 上档时控制底盘自转；SWC 自动旋转时无效。
        const bool is_rc_enable = target_rc.is_connect && target_rc.swa == FSi6X::DOWN;
        target_rc_is_enable = is_rc_enable ? 1 : 0;
        target_rc_is_world_frame_mode = 0;
        target_is_gimbal_mode = target_rc.swb == FSi6X::DOWN ? 1 : 0;
        if (!is_rc_enable) {
            target_rc_spin_mode = 0;
            target_rc_auto_wr_rpm = 0.0f;
            set_gimbal_current(0 * A);
            stop_chassis();
            send_can_cmd();
            return;
        }

        const UnitFloat vx = target_rc.x * VXY_MAX;
        const UnitFloat vy = target_rc.y * VXY_MAX;
        const bool is_auto_spin = target_rc.swc == FSi6X::MID || target_rc.swc == FSi6X::DOWN;
        const bool is_gimbal_mode = target_rc.swb == FSi6X::DOWN;
        target_rc_is_world_frame_mode = is_auto_spin ? 1 : 0;
        const float rc_yaw_ratio = apply_rc_yaw_deadband(target_rc.yaw.toFloat(ratio));
        UnitFloat wr = is_gimbal_mode ? 0 * rpm : -rc_yaw_ratio * WR_MAX;
        target_rc_spin_mode = 0;
        target_rc_auto_wr_rpm = 0.0f;
        if (is_auto_spin && target_rc.swc == FSi6X::MID) {
            wr = RC_SPIN_MID_SPEED;
            target_rc_spin_mode = 1;
            target_rc_auto_wr_rpm = wr.toFloat(rpm);
        } else if (is_auto_spin && target_rc.swc == FSi6X::DOWN) {
            wr = RC_SPIN_DOWN_SPEED;
            target_rc_spin_mode = 2;
            target_rc_auto_wr_rpm = wr.toFloat(rpm);
        }
        const UnitFloat wr_with_compensation = apply_yaw_compensation(vx, vy, wr, true);

        target_rc_vx_m_s = vx.toFloat(m_s);
        target_rc_vy_m_s = vy.toFloat(m_s);
        target_rc_wr_rpm = wr_with_compensation.toFloat(rpm);
        set_gimbal_speed(true, is_gimbal_mode ? rc_yaw_ratio : 0.0f);
        set_chassis_speed_direct(true, is_auto_spin, vx, vy, wr_with_compensation);
    } else if (target_test_mode == 30) {
        // 云台开环电流测试。底盘保持停止，target_current_a 直接映射为云台电流。
        set_gimbal_current((target_is_enable != 0) ? target_current_a * A : 0 * A);
        stop_chassis();
    } else {
        // 其它模式默认进入单轮测试；如果 test_mode 不在测试范围内，四轮都会失能。
        chassis.SetEnable(false);
        set_single_wheel_speed();
    }
    send_can_cmd();
}

extern "C" void rmpp_main() {
    setup();

    static UnitFloat<pct> cpu_usage;
    BSP::Dwt dwt;
    while (true) {
        // 主循环目标周期 1 ms。
        // running_time 是 loop() 实际耗时，除以 1 ms 后得到粗略 CPU 占用率。
        if (dwt.PollTimeout(1 * ms)) {
            loop();
            const UnitFloat<ms> running_time = dwt.GetDT();
            cpu_usage = running_time / (1 * ms);
        }
    }
}
