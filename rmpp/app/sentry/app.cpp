#include "misc.hpp"
#include "robot.hpp"

void send_can_cmd() {
    // CAN1
    ui.SendCanCmd();

    const int16_t cmd1 = w1.GetCanCmd();
    const int16_t cmd2 = w2.GetCanCmd();
    const int16_t cmd3 = w3.GetCanCmd();
    const int16_t cmd4 = w4.GetCanCmd();
    const int16_t cmd5 = yaw2.GetVoltageCmd();
    const int16_t cmd6 = shoot.GetCanCmd();
    // 四个舵电机的转速补偿减半，测试转舵稳定性。
    const int16_t cmd7 = s1.GetVoltageCmd(0.5f);
    const int16_t cmd8 = s2.GetVoltageCmd(0.5f);

    uint8_t data[8];
    data[0] = cmd1 >> 8;
    data[1] = cmd1;
    data[2] = cmd2 >> 8;
    data[3] = cmd2;
    data[4] = cmd3 >> 8;
    data[5] = cmd3;
    data[6] = cmd4 >> 8;
    data[7] = cmd4;
    BSP::CAN::Transmit(1, 0x200, data);
    data[0] = cmd5 >> 8;
    data[1] = cmd5;
    data[2] = cmd6 >> 8;
    data[3] = cmd6;
    data[4] = cmd7 >> 8;
    data[5] = cmd7;
    data[6] = cmd8 >> 8;
    data[7] = cmd8;
    BSP::CAN::Transmit(1, 0x1FF, data);

    // GM6020 ID 5/6：0x2FF slot1/2；未用 slot3 和保留 slot4 必须清零。
    const int16_t cmd9 = s3.GetVoltageCmd(0.5f);
    const int16_t cmd10 = s4.GetVoltageCmd(0.5f);
    data[0] = cmd9 >> 8;
    data[1] = cmd9;
    data[2] = cmd10 >> 8;
    data[3] = cmd10;
    data[4] = data[5] = data[6] = data[7] = 0;
    BSP::CAN::Transmit(1, 0x2FF, data);

    yaw1.SendCanCmd();

    // CAN2
    pitch.SendCanCmd();
    rub1.SendCanCmd();
    rub2.SendCanCmd();
}

void setup() {
    BSP::Init();
    // imu.Calibrate();
    robot.Init();
}

void loop() {
    robot.OnLoop();
    send_can_cmd();
}

extern "C" void rmpp_main() {
    setup();

    static UnitFloat<pct> cpu_usage;
    BSP::Dwt dwt;
    while (true) {
        if (dwt.PollTimeout(1 * ms)) {
            loop();
            const UnitFloat<ms> running_time = dwt.GetDT();
            cpu_usage = running_time / (1 * ms);
        }
    }
}
