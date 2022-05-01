#include "state_manager.h"

int8_t baseMotorPower = 50;
const int lineTraceThreshold = 40;

const int kLcourseParamsNum = 1;
const DrivingParam kLcourseParams[kLcourseParamsNum] = {
  { kTraceRightEdge, baseMotorPower, { 0.5, 0, 0 }, kDistanceEnd, kInvalidColor, 1000, false},
};

const int kRcourseParamsNum = 5;
const DrivingParam kRcourseParams[kRcourseParamsNum] = {
  { kTraceLeftEdge, 30, { 0.5, 0, 0 }, kDistanceEnd, kInvalidColor, 6400 },
  { kGoForward, 10, { }, kDistanceEnd, kInvalidColor, 300 },
};

StateManager::StateManager(DrivingManager* driving_manager, BingoAgent* bingo_agent)
    : driving_manager_(driving_manager), bingo_agent_(bingo_agent), state_(kTestRun) {

  // bool is_Rcourse_ = false;
  /*
  if (is_Rcourse_) {
    for (int i = 0; i < kRcourseParamsNum; ++i) {
      driving_manager_->AddDrivingParam(kRcourseParams[i]);
    }
  } else {
    for (int i = 0; i < kLcourseParamsNum; ++i) {
      driving_manager_->AddDrivingParam(kLcourseParams[i]);
    }
  }*/
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
  driving_manager_->Update();
  if (driving_manager_->DrivingParamsEmpty()) {
    state_ = kGetBonus;
  }
}


void StateManager::GetBonus() {
  driving_manager_->Update();
}

void StateManager::TestRun() {
  driving_manager_->Update();
  // if (driving_manager_->DrivingParamsEmpty()) {
  //   state_ = kGetBonus;
  // }
}

