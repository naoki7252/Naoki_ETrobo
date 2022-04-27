#include "state_manager.h"


StateManager::StateManager(WheelsControl* wheels_control) : wheels_control_(wheels_control){
}

void StateManager::TestRun() {
  int8_t power = 50;
  loopCount += 1;
  if (loopCount < 300) {
    wheels_control_->GoStraight(power);
  } else if (loopCount >= 300 && loopCount < 600) {
    wheels_control_->TurnLeft();
  } else {
    wheels_control_->Stop();
  }
}

