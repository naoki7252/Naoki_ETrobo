#include "state_manager.h"


StateManager::StateManager(WheelsControl* wheels_control, Luminous* luminous)
    : wheels_control_(wheels_control), luminous_(luminous), state_(kTimeAttack) {
}


void StateManager::Update() {
  switch (state_) {
    case kTimeAttack:
      TimeAttack();
      break;

    case kGetBonus:
      GetBonus();
      break;

    default:
      break;
  }
}


void StateManager::TimeAttack() {
  curr_hsv = luminous_->hsv_;
  //sprintf(str, "H: %f, S: %f, V: %f\n ", val.h, val.s, val.v);
  //syslog(LOG_NOTICE, str);

  wheels_control_->LineTrace(curr_hsv);
}


void StateManager::GetBonus() {
}

void StateManager::TestRun() {
}

