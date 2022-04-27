#ifndef ETRC22_STATE_MANAGER_H_
#define ETRC22_STATE_MANAGER_H_

#include "driving.h"

class StateManager {
 public:
  StateManager(WheelsControl* wheels_control);
  void TestRun();
  int loopCount = 0;
 private:
  WheelsControl* wheels_control_;

};

#endif  // ETRC22_STATE_MANAGER_H_
