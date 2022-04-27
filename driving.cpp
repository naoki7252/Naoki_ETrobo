#include "driving.h"

#include <math.h>

WheelsControl::WheelsControl(MotorIo* motor_io) : motor_io_(motor_io) {
}

void WheelsControl::Exec(int8_t target_power_l, int8_t target_power_r) {
  // int8_t curr_power_l = motor_io_->power_l_;
}

void WheelsControl::GoStraight(int8_t power) {
  motor_io_->SetWheelsPower(power, power);
}

void WheelsControl::GoBackStraight(int8_t power) {
  motor_io_->SetWheelsPower(-power, -power);
}

void WheelsControl::TurnLeft() {
  motor_io_->TurnLeft();
}

void WheelsControl::Stop() {
  bool is_break = true;
  motor_io_->StopWheels(is_break);
}
