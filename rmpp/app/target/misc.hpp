#pragma once

#include "misc/LED.hpp"
#include "misc/Buzzer.hpp"
#include "rc/RC.hpp"

// ==========================
// target app 的杂项外设
// ==========================
//
// target 是一个尽量小的底盘/云台调试程序。
// 这里不引入发射机构、裁判系统等步兵完整逻辑，只保留当前调车需要的设备：
// - LED：板载灯，后续可用于指示状态。
// - Buzzer：蜂鸣器，后续可用于提示错误或连接状态。
// - RC：遥控器输入，当前使用 FS-i6X SBUS，底层 UART 在 app.cpp 里初始化。

inline LED led({});
inline Buzzer buzzer;

// 遥控器对象。
// timeout = 100 ms：超过 100 ms 没有新遥控器数据，rc.is_connect 会变为 false，
// 上层 RC.cpp 会进一步影响 rc.is_enable，最终 app.cpp 会 stop_chassis() 防失控。
inline RC rc({.timeout = 100 * ms}, {});
