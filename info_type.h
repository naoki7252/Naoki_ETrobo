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