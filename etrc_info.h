#ifndef ETRC22_ETRC_INFO_H_
#define ETRC22_ETRC_INFO_H_


#include <vector>
#include "ev3api.h"
#include "info_type.h"
#include "device_io.h"
#include "time.h"
#include<math.h>
// #include<iostream>
#include<stdio.h>
#include<algorithm>
#include<tuple>
#include<climits>


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

class Pstate{//機体の状態
    public:
        Pstate(double initx, double inity, double inityaw);
        void update(double x_, double y_);
        double calc_distance(double point_x, double point_y);
        double x, y, yaw;
        double rear_x, rear_y;
        std::vector<double>tx; //現在のx座標
        std::vector<double>ty; //現在のy座標
};

class TargetCourse{//次の目標点の算出
    public:
        TargetCourse(std::vector<double>x, std::vector<double>y);
        std::tuple<int, double>search_target_index(Pstate pstate);
        std::vector<double>cx; //目標点のx座標
        std::vector<double>cy; //目標点のy座標
        double distance_this_index;
        double distance_next_index;    
        double lf = 50;//先見る距離
    private:    
        int old_point_index;
        int ind = 0;

};

class Pursuit{
    public:
        Pursuit();
        // std::vector<double>rx;//仮想軌道のx座標 
        // std::vector<double>ry;//仮想軌道のy座標 
        void pursuit_course(std::vector<double> course, std::string input_csv_file_path);
        std::tuple<int, double> pursuit_control(Pstate pstate, TargetCourse& trajectory, int pind);
        void pursuit_update(std::vector<double>rx, std::vector<double>ry, double init[3]);
        
        double lastindex;

    private:
        int  Ind;
        double lf;
        double tx, ty;
        double T = 1300;//実機
        double Time = 0;//実機
        double lf_m;//先見る距離
        int target_ind;
        double speed, delta;
        double dt = 0.1;
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
