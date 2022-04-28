#ifndef ETRC22_STATE_MANAGER_H_
#define ETRC22_STATE_MANAGER_H_

#include "driving.h"
#include "etrc_info.h"
#include "info_type.h"

class StateManager {
 public:
  StateManager(WheelsControl* wheels_control, Luminous* luminous);
  void TimeAttack();
  void TestRun();
  //char str[264];
  Hsv curr_hsv;
 private:
  WheelsControl* wheels_control_;
  Luminous* luminous_;

};

#endif  // ETRC22_STATE_MANAGER_H_
