#include "driving_type.h"

#include <math.h>


WheelsControl::WheelsControl(MotorIo* motor_io) : motor_io_(motor_io) {
}

void WheelsControl::Exec(int8_t target_power_l, int8_t target_power_r) {
  int8_t curr_power_l = motor_io_->power_l_;
  if (target_power_l > curr_power_l) {
    curr_power_l += 1;
  } else if (target_power_l < curr_power_l) {
    curr_power_l -= 1;
  }

  int8_t curr_power_r = motor_io_->power_r_;
  if (target_power_r > curr_power_r) {
    curr_power_r += 1;
  } else if (target_power_r < curr_power_r) {
    curr_power_r -= 1;
  }

  if (target_power_l == 0 && target_power_r == 0) {
    motor_io_->StopWheels(true);
  } else {
    motor_io_->SetWheelsPower(curr_power_l, curr_power_r);
  }
}

BasicDriver::BasicDriver(WheelsControl* wheels_control)
    : wheels_control_(wheels_control),
      move_type_(kInvalidMove), ref_power_(0) {
}

BasicDriver::~BasicDriver() {
}

void BasicDriver::SetParam(Move move_type, int8_t ref_power) {
  move_type_ = move_type;
  ref_power_ = ref_power;
}

void BasicDriver::Run() {
  int8_t power_l;
  int8_t power_r;

  if (move_type_ == kGoForward) {
    power_l = power_r = ref_power_;
  } else if (move_type_ == kGoBackward) {
    power_l = power_r = -ref_power_;
  } else if (move_type_ == kRotateLeft) {
    power_l = -ref_power_;
    power_r = ref_power_;
  } else if (move_type_ == kRotateRight) {
    power_l = ref_power_;
    power_r = -ref_power_;
  } else {
    power_l = power_r = 0;
  }

  wheels_control_->Exec(power_l, power_r);
}

void BasicDriver::Stop() {
  wheels_control_->Exec(0, 0);
}

LineTracer::LineTracer(WheelsControl* wheels_control, Luminous* luminous)
    : wheels_control_(wheels_control), luminous_(luminous),
      move_type_(kInvalidMove), ref_power_(0), ref_value_(0) {
  pid_control_ = new PidControl();
}

LineTracer::~LineTracer() {
  delete pid_control_;
}

void LineTracer::SetParam(Move move_type, int8_t ref_power, float ref_value, Gain gain) {
  move_type_ = move_type;
  ref_power_ = ref_power;
  ref_value_ = ref_value;
  pid_control_->SetGain(gain.kp, gain.ki, gain.kd);
}

void LineTracer::Run() {
  // float mv = pid_control_->GetMv(ref_value_, luminous_->hsv_.v);
  float mv = 1.0f;
  if (move_type_ == kTraceLeftEdge) {
    mv *= -1;
  }

  int8_t power_l = static_cast<int8_t>(ref_power_ + mv);
  int8_t power_r = static_cast<int8_t>(ref_power_ - mv);
  wheels_control_->Exec(power_l, power_r);
}

void LineTracer::Stop() {
  wheels_control_->Exec(0, 0);
}

/*
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


  //char str[264];

  //sprintf(str, "H: %f, S: %f, V: %f\n ", curr_hsv.h, curr_hsv.s, curr_hsv.v);
  //syslog(LOG_NOTICE, str);

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
*/
