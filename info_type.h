#ifndef ETRC22_INFO_TYPE_H_
#define ETRC22_INFO_TYPE_H_

enum Color {
  kGreen = 0,
  kBlack,
  kRed,
  kYellow,
  kBlue,
  kWhite,
  kInvalidColor,
  kColorNum
};

struct Rgb {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

struct Hsv {
  float h;
  float s;
  float v;
};

struct Gain {
  float kp;
  float ki;
  float kd;
};

#endif  // ETRC22_INFO_TYPE_H_
