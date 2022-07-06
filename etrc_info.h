#ifndef ETRC22_ETRC_INFO_H_
#define ETRC22_ETRC_INFO_H_

#include "ev3api.h"
#include "info_type.h"
#include "device_io.h"
#include "time.h"
#include <vector>

// #include "pursuit.h"

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
  // clock_t before_time = 0;
};

class Pursuit {
  public:
    Pursuit(double initx, double inity, double inityaw, double innitv);
    void Update(double v_,double delta);
    double Calc_distance(double point_x, double point_y, int a);
    double x, y, yaw, v;
    double rear_x, rear_y;
    std::vector<double>tx;
    std::vector<double>ty;

  private:
    const int8_t lfc = 100; //先見る距離
    const int8_t k = 1;
    const float kp = 0.02; 
    double  dt = 0.1;

};

class TargetCourse{
  public:
    TargetCourse(std::vector<double>x,std::vector<double>y);
    std::tuple<int,double>search_target_index(Pursuit pursuit);
    std::vector<double>cx;
    std::vector<double>cy;
    double distance_this_index;
    double distance_next_index;  

  private:    
    const int8_t lfc = 100; 
    const int8_t k = 1;
    const float kp = 0.02; 
    double  dt = 0.1;
    int old_point_index;
    int ind = 0;
};

class Localize {
 public:
  Localize(MotorIo* motor_io);
  void Update();
  void SaveOdometri();
  void Trajectory();
  double distance_ = 0;
  double distance_right = 0;
  int32_t theta = 0;

 private:
  MotorIo* motor_io_;
  int32_t counts_rs[100000] = {};
  int32_t counts_ls[100000] = {};
  float locate_x[100000] = {};
  float locate_y[100000] = {};
  unsigned long secs[100000] = {};
  float theta_[100000] = {};
  int curr_index = 0;
  const float R = 49.75;
  const int8_t D = 128;
  double theta_wa = 0;
  double x = 0;
  double y = 0;
  clock_t before_time = 0;
  char str[264];
  struct timespec now_time;
};

#endif  // ETRC22_ETRC_INFO_H_
