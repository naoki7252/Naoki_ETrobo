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

class TimeAttacker {
 public:
  TimeAttacker(DrivingManager* driving_manager, bool is_Lcourse);
  void Update();
  bool is_completed = false;
 private:
  void SetTimeAttackDriveParam(bool is_Lcourse);
  DrivingManager* driving_manager_;
  DrivingParam timeAttackDriveParams[100] = {};
  int currParamIndex = 0;
  int paramNum = 0;
};

class BonusGetter {
 public:
  BonusGetter(DrivingManager* driving_manager, bool is_Lcourse);
  void Update();
 private:
  DrivingManager* driving_manager_;
  bool is_Lcourse_;
};

class StateManager {
 public:
  StateManager(TimeAttacker* time_attacker, BonusGetter* bonus_getter, BingoAgent* bingo_agent);
  void Update();
  
 private:
  void TimeAttack();
  void GetBonus();
  void TestRun();
  TimeAttacker* time_attacker_;
  BonusGetter* bonus_getter_;
  BingoAgent* bingo_agent_;
  State state_;
};

#endif  // ETRC22_STATE_MANAGER_H_
