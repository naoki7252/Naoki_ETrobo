#include "etrc_info.h"

#include<vector>
// #include<math.h>
// #include<iostream>
#include<string>
#include<fstream>
#include<sstream>
#include<stdio.h>
// #include<algorithm>
// #include<tuple>
// #include<climits>
#include "app.h"

// #include"CubicSpline.h"

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

Pstate::Pstate(double initx, double inity, double inityaw){
    x = initx;
    y = inity;
    yaw = inityaw;
}

void Pstate::update(double x_, double y_){//現在の推定位置
    x = x_;
    y = y_;
    tx.push_back(x);
    ty.push_back(y);
}

double Pstate::calc_distance(double point_x, double point_y){
    double dx = x - point_x;
    double dy = y - point_y;
    return hypot(dx,dy);
}

// State state(300.0, 1100.0, M_PI/2 );//初期位置 x, y, 姿勢


TargetCourse::TargetCourse(std::vector<double>x, std::vector<double>y){
    for(int i=0; i<y.size(); i++){
        cx.push_back(x[i]);
        cy.push_back(y[i]);
    }
    old_point_index = INT_MAX; //最初と次の目標点を識別するため
}

std::tuple<int, double>TargetCourse::search_target_index(Pstate pstate){
    //最短点を探す
    if(old_point_index == INT_MAX){//最初 初期位置から最も近い座標の探索
        double dx;
        double dy;
        std::vector<int>d;

        for(int i=0; i<cx.size(); i++){
            dx = pstate.x-cx[i];//最初の座標と目標点の差
            dy = pstate.y-cy[i];
            d.push_back(hypot(dx, dy));
        }
        std::vector<int>::iterator minIt = std::min_element(d.begin(), d.end());//1番近い距離を探索
        ind = std::distance(d.begin(), minIt); //最小値の場所
        old_point_index = ind;
    } 
    
    else{
        ind = old_point_index;
        distance_this_index = pstate.calc_distance(cx[ind], cy[ind]);//現在の座標と目標点との距離 

        while(true){
            distance_next_index = pstate.calc_distance(cx[ind+1], cy[ind+1]);//次の目標点との距離
            if(distance_this_index < distance_next_index)break;
            
            if(ind+1 < cx.size()) ind++;
            else ind=ind;
            
            distance_this_index = distance_next_index;
        }
        old_point_index=ind;
    }

    while(lf > pstate.calc_distance(cx[ind], cy[ind])){//目標点がlfより小さければ次の目標点
        if(ind+1 >= cx.size()) break;
        ind += 1;
    }
    
    return std::forward_as_tuple(ind, lf);
}

Pursuit::Pursuit(){}

// void Pursuit::pursuit_course(std::vector<double> course, std::string input_csv_file_path){
//   std::string str_buf;
//   std::string str_conma_buf;
//   std::ifstream input_csv_file(input_csv_file_path);
//   while (getline(ifs_csv_file, str_buf)) { 
//     std::istringstream i_stream(str_buf);
//     while (getline(i_stream, str_conma_buf, ',')) {
//     double pre = stod(str_conma_buf);
//     course.push_back(pre);
//     }
//   }
// }

std::tuple<int, double>Pursuit::pursuit_control(Pstate pstate, TargetCourse& trajectory, int pind){
    //pind 目標点
    //戻り値がjntとdouble
    std::tie(Ind, lf) = trajectory.search_target_index(pstate); //目標点と見る距離
    if(pind >= Ind) Ind = pind;// 目標点

    if(Ind < trajectory.cx.size()){
        tx = trajectory.cx[Ind];    
        ty = trajectory.cy[Ind];    
    }

    if(Ind > trajectory.cx.size()){
        tx = trajectory.cx[trajectory.cx.size()-1];
        ty = trajectory.cy[trajectory.cy.size()-1];
        Ind = trajectory.cx.size()-1;
    }

    double alpha = atan2(ty-pstate.y, tx-pstate.x);//目標点と現在の座標の差から方位角[rad]を算出
    return std::forward_as_tuple(Ind, alpha);//目標点と方位角
}

void Pursuit::pursuit_update(std::vector<double>rx, std::vector<double>ry, double init[3]){
    Pstate pstate(init[0], init[1], init[2]);
    lastindex = rx.size()-1;
    TargetCourse target_couse(rx, ry);//create course
    std::tie(target_ind, lf_m) = target_couse.search_target_index(pstate); //init state tie複数の引数 最初の目標点
    while(T >= Time && lastindex > target_ind){ //Timeは速度が分かったら時間に変更
        std::tie(target_ind, delta) = pursuit_control(pstate, target_couse, target_ind); //最初以降の目標点
        pstate.update(speed, delta); //ここに自己位置推定したx, y座標
        Time += dt;
    }
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
 