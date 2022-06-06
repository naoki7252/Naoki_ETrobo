#ifndef ETRC22_ETRC_INFO_H_
#define ETRC22_ETRC_INFO_H_

#include "ev3api.h"
#include "info_type.h"
#include "device_io.h"

class Luminous {
 public:
  Luminous(SensorIo* sensor_io, Camera* camera);
  void Update();
  Color color_;
  Rgb rgb_;
  Hsv hsv_;

 private:
  void SetColorReference(Color c, Hsv hsv);
  void UpdateRgb();
  void UpdateHsv();
  void UpdateColor();
  SensorIo* sensor_io_;
  Camera* camera_;
  Hsv color_ref_[kColorNum];
};

class Localize {
 public:
  Localize(MotorIo* motor_io);
  void Update();
  void SaveOdometri();
  double distance_ = 0;
  double distance_right = 0;
  int32_t theta = 0;

 private:
  MotorIo* motor_io_;
  int32_t counts_rs[100000] = {};
  int32_t counts_ls[100000] = {};
  int curr_index = 0;
  const float R = 49.75;
  const int8_t D = 128;
  double theta_wa = 0;
  double x = 0;
  double y = 0;
};

#endif  // ETRC22_ETRC_INFO_H_
