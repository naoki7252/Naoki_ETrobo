#include "etrc_info.h"

#include <math.h>
#include <time.h>

#include "app.h"

Luminous::Luminous(SensorIo* sensor_io, Camera* camera)
    : color_(kInvalidColor), hsv_({0, 0, 0}), sensor_io_(sensor_io), camera_(camera) {
}

void Luminous::Update() {
  UpdateRgb();
  UpdateHsv();
  UpdateColor();
}

void Luminous::SetColorReference(Color c, Hsv hsv) {
  color_ref_[c] = hsv;
}

void Luminous::UpdateRgb() {
  rgb_raw_t val = sensor_io_->color_rgb_raw_;
  
  rgb_.r = val.r;
  rgb_.g = val.g;
  rgb_.b = val.b;
}

void Luminous::UpdateHsv() {
  float r = static_cast<float>(rgb_.r);
  float g = static_cast<float>(rgb_.g);
  float b = static_cast<float>(rgb_.b);

  float max = r > g ? r : g;
  max = max > b ? max : b;
  float min = r < g ? r : g;
  min = min < b ? min : b;
  float c = max - min;

  float h;
  if (c == 0) {
    h = -1;
  } else if (max == r) {
    h = fmodf(((g - b) / c), 6);
  } else if (max == g) {
    h = ((b - r) / c) + 2;
  } else if (max == b) {
    h = ((r - g) / c) + 4;
  } else {
    h = -1;
  }

  if (h != -1) {
    h = 60 * h;
  }

  float s;
  if (max == 0) {
    s = 0;
  } else {
    s = c / max;
  }

  float v = max;
  if (v > 100) {
    v = 100;
  }

  hsv_.h = h;
  hsv_.s = s * 100;
  hsv_.v = v;
}

void Luminous::UpdateColor() {
}

Localize::Localize(MotorIo* motor_io)
    : motor_io_(motor_io) {
}

void Localize::Update() {
  
    // unsigned long sec;
    // clock_gettime(CLOCK_REALTIME, &now_time);
    // sec = now_time.tv_sec;
    // secs[curr_index] = sec;
    // curr_index += 1;

    unsigned long nsec;
    clock_gettime(CLOCK_REALTIME, &now_time);
    nsec = now_time.tv_nsec;
    secs[curr_index] = nsec;
    curr_index += 1;
    
    // nsec = now_time.tv_nsec;
  // clock_t now = clock();
  // // double a = (static_cast<double>(now-before_time))/CLOCKS_PER_SEC;
  // // double keep += a;  
  // sprintf(str, "time: %f sum: %f\n", (static_cast<double>(now-before_time))/CLOCKS_PER_SEC);
  // syslog(LOG_NOTICE, str);
  // before_time = now;
  
  int32_t counts_r_ = motor_io_->counts_r_;
  int32_t counts_l_ = motor_io_->counts_l_;

  // if(counts_l_ > 50 && counts_r_ > 50) {
  //   counts_l_=0;
  //   counts_r_=0;
  // }

  // curr_index += 1;
  counts_rs[curr_index] = counts_r_;
  counts_ls[curr_index] = counts_l_;
  locate_x[curr_index] = x;
  locate_y[curr_index] = y;

  double Ll = R * (counts_ls[curr_index] - counts_ls[curr_index - 1]) * M_PI / 180;
  double Lr = R * (counts_rs[curr_index] - counts_rs[curr_index - 1]) * M_PI / 180;

  double micro_theta = (Lr - Ll) / D;
  theta_wa += micro_theta;
  theta = counts_r_;
  double A = (Lr + Ll) / 2 * (1 - 0);
  double dx = A * cos(theta_wa + micro_theta / 2);
  double dy = A * sin(theta_wa + micro_theta / 2);
  double dd = sqrt(dx * dx + dy * dy);

  x += dx;
  y += dy;
  distance_ += dd;
  distance_right += A;

   char str[264];
  sprintf(str, "x: %f y: %f distance: %f distance_right: %f theta_wa:%f\n", x, y, distance_, distance_right, theta_wa);
  syslog(LOG_NOTICE, str); 
}

 void Localize::SaveOdometri() {
  char str [256];
  FILE* fp = fopen("Odome.csv", "w");

  // for (int i=0; i<curr_index; i++) {
  //   sprintf(str, "%f, %f\n", locate_x[i], locate_y[i]);
  //   fprintf(fp, str);
  // }

  // for (int i=0; i<curr_index; i++) {
  //   sprintf(str, "%f, %f\n", counts_rs[i], counts_ls[i]);
  //   fprintf(fp, str);
  // }

  for (int i=0; i<curr_index; i++) {
    sprintf(str, "%u\n", secs[i]);
    fprintf(fp, str);
  }

  fclose(fp);
}
 