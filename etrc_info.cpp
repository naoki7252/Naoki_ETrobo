#include "etrc_info.h"

#include <math.h>

#include "app.h"

Luminous::Luminous(SensorIo* sensor_io)
    : color_(kInvalidColor), hsv_({0, 0, 0}), sensor_io_(sensor_io) {
}

void Luminous::Update() {
  UpdateHsv();
  UpdateColor();
}

void Luminous::SetColorReference(Color c, Hsv hsv) {
  color_ref_[c] = hsv;
}

void Luminous::UpdateHsv() {
  // rgb_raw_t val = sensor_io_->color_rgb_raw_;
}

void Luminous::UpdateColor() {
}

Localize::Localize(MotorIo* motor_io)
    : motor_io_(motor_io) {
}

void Localize::Update() {
}