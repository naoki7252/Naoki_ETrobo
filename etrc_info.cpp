#include "etrc_info.h"

#include <math.h>
#include <time.h>
#include <tuple>
#include <vector>
#include <algorithm>
#include <climits>

#include "app.h"

// #include"pursuit.h"
// #include"CubicSpline.h"
// #include<iostream>
// #include<string>
// #include<fstream>
// #include<sstream>

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

Pursuit::Pursuit(double initx, double inity, double inityaw, double initv) {
    x = initx;
    y = inity;
    v = initv;
    yaw = inityaw;
}

void Pursuit::Update(double v_, double alpha) {
    x += v*cos(yaw)*dt;
    y += v*sin(yaw)*dt;
    v += v_*dt;
    yaw = alpha;
    tx.push_back(x);
    ty.push_back(y);
}

double Pursuit::Calc_distance(double point_x, double point_y, int a){
    double dx = x-point_x;
    double dy = y-point_y;
/*
    std::cout<<"X:"<<point_x<<std::endl;
    std::cout<<"Y:"<<point_y<<std::endl;
    std::cout<<"a:"<<a<<std::endl;
    std::cout<<" "<<std::endl;
    */
    return hypot(dx, dy);
}

TargetCourse::TargetCourse(std::vector<double>x, std::vector<double>y){
    for(int i=0; i<y.size(); i++){
        cx.push_back(x[i]);
        cy.push_back(y[i]);
    }
    old_point_index = INT_MAX;
    //std::cout<<"constractor";
}

std::tuple<int,double>TargetCourse::search_target_index(Pursuit pursuit){
    //最短点を探す
   if(old_point_index==INT_MAX){
        double dx;
        double dy;
        std::vector<int>d;

        for(int i=0;i<cx.size();i++){
            dx = pursuit.x-cx[i];
            dy = pursuit.y-cy[i];
            d.push_back(hypot(dx, dy));
        }
        std::vector<int>::iterator minIt=std::min_element(d.begin(), d.end());
        ind=std::distance(d.begin(), minIt);

        old_point_index=ind;

    } 

   
    else{
       ind=old_point_index;
       distance_this_index=pursuit.Calc_distance(cx[ind], cy[ind], 1); 

       while(true){

            distance_next_index=pursuit.Calc_distance(cx[ind+1], cy[ind+1], 2);
            if(distance_this_index<distance_next_index)break;
            
            if(ind+1<cx.size()){
                ind++;
            }
            else{
                ind=ind;
            }
            distance_this_index = distance_next_index;
        }
        old_point_index=ind;
    }
    

    double lf=k*pursuit.v+lfc;
    
    while(lf>pursuit.Calc_distance(cx[ind], cy[ind], 4)){
        if(ind+1>=cx.size())break;
        ind+=1;
    }
    
    return std::forward_as_tuple(ind, lf);
}

double speed_control(double target, double current){
    // double speed=kp*(target-current);
    // return speed;
}

std::tuple<int, double> pursuit_control(Pursuit pursuit, TargetCourse& trajectory, int pind){

    int  Ind;
    double lf;
    std::tie(Ind, lf)=trajectory.search_target_index(pursuit);


    double tx,ty;

    if(pind>=Ind){
        Ind=pind;

    }
    
    if(Ind<trajectory.cx.size()){
        tx=trajectory.cx[Ind];    
        ty=trajectory.cy[Ind];    
    }
    if(Ind>trajectory.cx.size()){
        tx=trajectory.cx[trajectory.cx.size()-1];
        ty=trajectory.cy[trajectory.cy.size()-1];
        Ind=trajectory.cx.size()-1;
    }

    double alpha=atan2(ty-pursuit.y, tx-pursuit.x);
  //  double delta=atan2(2.0*WB*sin(alpha)/lf,1.0);
  //  double dist=hypot(tx,ty);
    
   // double relx=dist*cos(alpha);
   // double rely=dist*sin(alpha);
    return std::forward_as_tuple(Ind, alpha);
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

    // unsigned long nsec;
    // clock_gettime(CLOCK_REALTIME, &now_time);
    // nsec = now_time.tv_nsec;
    // secs[curr_index] = nsec;
    // curr_index += 1;
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
  theta_[curr_index] = theta_wa;

  // theta_[curr_index] = micro_theta;

  // char str[264];
  // sprintf(str, "x: %f y: %f distance: %f distance_right: %f theta_wa:%f\n", x, y, distance_, distance_right, theta_wa);
  // syslog(LOG_NOTICE, str);

  // char str[264];
  // sprintf(str, "theta: %f\n", micro_theta*180/M_PI);
  // syslog(LOG_NOTICE, str);

  char str[264];
  sprintf(str, "theta: %f\n", theta_wa*180/M_PI);
  syslog(LOG_NOTICE, str); 
}

void Localize::Trajectory(){


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

  //  for (int i=0; i<curr_index; i++) {
  //    sprintf(str, "%u\n", secs[i]);
  //    fprintf(fp, str);
  //  }

  //for (int i=0; i<curr_index; i++) {
  //  sprintf(str, "%f\n", theta_[i]*180/M_PI);
  //  fprintf(fp, str);
  //}


  fclose(fp);
}
 