#include "driving.h"

#include <math.h>

WheelsControl::WheelsControl(MotorIo* motor_io) : motor_io_(motor_io) {
}

void WheelsControl::Exec(int8_t target_power_l, int8_t target_power_r) {
  // int8_t curr_power_l = motor_io_->power_l_;
}

void WheelsControl::LineTrace(Hsv curr_hsv) {
  int8_t base_power = 50;
  float target_v = 40;

  //float mv = calcMv(curr_hsv.v);
  float kp = 0.45;
  float mv = (target_v - curr_hsv.v) * kp;

  int8_t right_power = static_cast<int8_t>(base_power + mv);
  int8_t left_power = static_cast<int8_t>(base_power - mv);

  motor_io_->SetWheelsPower(right_power, left_power);

  /*
  char str[264];

  sprintf(str, "H: %f, S: %f, V: %f\n ", curr_hsv.h, curr_hsv.s, curr_hsv.v);
  syslog(LOG_NOTICE, str);
  */
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
