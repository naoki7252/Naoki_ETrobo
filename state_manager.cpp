#include "state_manager.h"


StateManager::StateManager(WheelsControl* wheels_control, Luminous* luminous) : wheels_control_(wheels_control), luminous_(luminous) {
}


void StateManager::TimeAttack() {
  curr_hsv = luminous_->hsv_;
  //sprintf(str, "H: %f, S: %f, V: %f\n ", val.h, val.s, val.v);
  //syslog(LOG_NOTICE, str);

  wheels_control_->LineTrace(curr_hsv);
}

void StateManager::TestRun() {
}

