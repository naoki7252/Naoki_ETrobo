#include "state_manager.h"


const int kLcourseParamsNum = 1;
const DrivingParam kLcourseTimeAttackParams[kLcourseParamsNum] = {
  { kTraceRightEdge, 50, { 0.5, 0, 0 }, kDistanceEnd, kInvalidColor, 1000, false, false},
};

const int kRcourseParamsNum = 2;
const DrivingParam kRcourseTimeAttackParams[kRcourseParamsNum] = {
  { kTraceLeftEdge, 30, { 0.5, 0, 0 }, kDistanceEnd, kInvalidColor, 6400, false, false},
  { kGoForward, 10, { }, kDistanceEnd, kInvalidColor, 300, false, false},
};

TimeAttacker::TimeAttacker(DrivingManager* driving_manager, bool is_Lcourse) 
    : driving_manager_(driving_manager) {
  SetTimeAttackDriveParam(is_Lcourse);
}

void TimeAttacker::SetTimeAttackDriveParam(bool is_Lcourse_) {
  if (is_Lcourse_) {
    paramNum = kLcourseParamsNum;

    for (int i=0; i<paramNum; i++) {
      timeAttackDriveParams[i] = kLcourseTimeAttackParams[i];
    }
  } else {
    paramNum = kRcourseParamsNum;

    for (int i=0; i<paramNum; i++) {
      timeAttackDriveParams[i] = kRcourseTimeAttackParams[i];
    }
  }
}

void TimeAttacker::Update() {
  DrivingParam& curr_param = timeAttackDriveParams[currParamIndex];
  if (!curr_param.is_started) {
    driving_manager_->SetDriveParam(curr_param);
    curr_param.is_started = true;
  }

  driving_manager_->Update();

  if (driving_manager_->is_satisfied) {
    currParamIndex += 1;
  }

  if (currParamIndex >= paramNum) {
    is_completed = true;
  }
}

BonusGetter::BonusGetter(DrivingManager* driving_manager, bool is_Lcourse)
    : driving_manager_(driving_manager), is_Lcourse_(is_Lcourse) {
}

void BonusGetter::Update() {

}

StateManager::StateManager(TimeAttacker* time_attacker, BonusGetter* bonus_getter, BingoAgent* bingo_agent)
    : time_attacker_(time_attacker), bonus_getter_(bonus_getter), bingo_agent_(bingo_agent), state_(kTestRun) {
}


void StateManager::Update() {
  switch (state_) {
    case kTimeAttack:
      TimeAttack();
      break;

    case kGetBonus:
      GetBonus();
      break;

    case kTestRun:
      TestRun();
      break;

    default:
      break;
  }
}

void StateManager::TimeAttack() {
  time_attacker_->Update();
  if (time_attacker_->is_completed) {
    state_ = kGetBonus;
  }
}

void StateManager::GetBonus() {
  bonus_getter_->Update();
}

void StateManager::TestRun() {
  // driving_manager_->Update();
  // if (driving_manager_->DrivingParamsEmpty()) {
  //   state_ = kGetBonus;
  // }
}

