#ifndef ETRC22_DRIVING_TYPE_H_
#define ETRC22_DRIVING_TYPE_H_

#include "etrc_info.h"
#include "utils.h"

class WheelsControl {
 public:
  WheelsControl(MotorIo* motor_io);
  void Exec(int8_t target_power_l, int8_t target_power_r);

 private:
  MotorIo* motor_io_;
};

class BasicDriver {
 public:
  BasicDriver(WheelsControl* wheels_control);
  ~BasicDriver();
  void SetParam(Move move_type, int8_t ref_power);
  void Run();
  void Stop();

 private:
  WheelsControl* wheels_control_;
  Move move_type_;
  int8_t ref_power_;
};

class LineTracer {
 public:
  LineTracer(WheelsControl* wheels_control, Luminous* luminous);
  ~LineTracer();
  void SetParam(Move move_type, int8_t ref_power, float ref_value, Gain gain);
  void Run();
  void Stop();

 private:
  WheelsControl* wheels_control_;
  Luminous* luminous_;
  Move move_type_;
  int8_t ref_power_;
  float ref_value_;
  PidControl* pid_control_;
};

#endif  // ETRC22_DRIVING_TYPE_H_