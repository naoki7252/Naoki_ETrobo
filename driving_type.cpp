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
      move_type_(kInvalidMove), base_power_(0) {
}

BasicDriver::~BasicDriver() {
}

void BasicDriver::SetParam(Move move_type, int8_t base_power) {
  move_type_ = move_type;
  base_power_ = base_power;
}

void BasicDriver::Run() {
  int8_t power_l;
  int8_t power_r;

  if (move_type_ == kGoForward) {
    power_l = power_r = base_power_;
  } else if (move_type_ == kGoBackward) {
    power_l = power_r = -base_power_;
  } else if (move_type_ == kRotateLeft) {
    power_l = -base_power_;
    power_r = base_power_;
  } else if (move_type_ == kRotateRight) {
    power_l = base_power_;
    power_r = -base_power_;
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
      move_type_(kInvalidMove), base_power_(0) {
  pid_control_ = new PidControl();
}

LineTracer::~LineTracer() {
  delete pid_control_;
}

void LineTracer::SetParam(Move move_type, int8_t base_power, Gain gain) {
  move_type_ = move_type;
  base_power_ = base_power;
  pid_control_->SetGain(gain.kp, gain.ki, gain.kd);
}

void LineTracer::Run() {
  float curr_hsv = luminous_->hsv_.v;
  float mv = pid_control_->CalcMv(line_trace_threshold, curr_hsv);

  if (move_type_ == kTraceLeftEdge) {
    mv *= -1;
  }

  int8_t power_l = static_cast<int8_t>(base_power_ + mv);
  int8_t power_r = static_cast<int8_t>(base_power_ - mv);
  wheels_control_->Exec(power_l, power_r);
}

void LineTracer::Stop() {
  wheels_control_->Exec(0, 0);
}
