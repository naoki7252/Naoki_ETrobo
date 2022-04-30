#ifndef ETRC22_STATE_MANAGER_H_
#define ETRC22_STATE_MANAGER_H_

#include "driving_manager.h"
#include "game_play.h"

enum State {
  kTimeAttack = 0,
  kGetBonus,
  kTestRun,
  kStateNum
};

class StateManager {
 public:
  StateManager(DrivingManager* driving_manager, BingoAgent* bingo_agent);
  void Update();
  
  //char str[264];
  Hsv curr_hsv;
 private:
  void TimeAttack();
  void GetBonus();
  void TestRun();
  DrivingManager* driving_manager_;
  BingoAgent* bingo_agent_;
  State state_;
};

#endif  // ETRC22_STATE_MANAGER_H_
