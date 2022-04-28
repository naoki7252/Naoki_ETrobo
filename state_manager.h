#ifndef ETRC22_STATE_MANAGER_H_
#define ETRC22_STATE_MANAGER_H_

#include "driving.h"

enum State {
  kTimeAttack = 0,
  kGetBonus,
  kStateNum
};

class StateManager {
 public:
  StateManager(WheelsControl* wheels_control, Luminous* luminous);
  void Update();
  
  //char str[264];
  Hsv curr_hsv;
 private:
  void TimeAttack();
  void GetBonus();
  void TestRun();
  WheelsControl* wheels_control_;
  Luminous* luminous_;
  State state_;
};

#endif  // ETRC22_STATE_MANAGER_H_
