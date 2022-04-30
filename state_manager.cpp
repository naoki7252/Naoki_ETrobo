#include "state_manager.h"

const int kLcourseParamsNum = 5;
const DrivingParam kLcourseParams[kLcourseParamsNum] = {
  { kTraceRightEdge, 30, 50, { 0.4, 0, 0.05 }, kDistanceEnd, kInvalidColor, 6400 },
  { kGoForward, 10, 0, { }, kDistanceEnd, kInvalidColor, 300 },
  { kTraceRightEdge, 30, 50, { 0.4, 0, 0.05 }, kDistanceEnd, kInvalidColor, 800 },
  { kGoForward, 10, 0, { }, kDistanceEnd, kInvalidColor, 300 },
  { kTraceRightEdge, 30, 50, { 0.4, 0, 0.05 }, kColorEnd, kYellow, 0 },
};

const int kRcourseParamsNum = 5;
const DrivingParam kRcourseParams[kRcourseParamsNum] = {
  { kTraceLeftEdge, 30, 50, { 0.4, 0, 0.05 }, kDistanceEnd, kInvalidColor, 6400 },
  { kGoForward, 10, 0, { }, kDistanceEnd, kInvalidColor, 300 },
};

StateManager::StateManager(DrivingManager* driving_manager, BingoAgent* bingo_agent)
    // : driving_manager_(driving_manager), bingo_agent_(bingo_agent), state_(kTimeAttack) {
    : driving_manager_(driving_manager), bingo_agent_(bingo_agent), state_(kTestRun) {
  
  bool is_Rcourse_ = false;
  if (is_Rcourse_) {
    for (int i = 0; i < kRcourseParamsNum; ++i) {
      driving_manager_->AddDrivingParam(kRcourseParams[i]);
    }
  } else {
    for (int i = 0; i < kLcourseParamsNum; ++i) {
      driving_manager_->AddDrivingParam(kLcourseParams[i]);
    }
  }
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
}

void StateManager::TestRun() {
  driving_manager_->Update();
  // if (driving_manager_->DrivingParamsEmpty()) {
  //   state_ = kGetBonus;
  // }
}

