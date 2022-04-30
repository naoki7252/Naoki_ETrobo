#include "driving_manager.h"

DrivingManager::DrivingManager(BasicDriver* basic_driver, LineTracer* line_tracer, EndCondition* end_condition)
    : basic_driver_(basic_driver), line_tracer_(line_tracer), end_condition_(end_condition) {
}

void DrivingManager::Update() {
  /*
  if (driving_params_.size() <= 0) {
    return;
  }

  DrivingParam& curr_param = driving_params_.front();

  if (!curr_param.is_started) {
    SetDriveParam(curr_param);
    SetEndParam(curr_param);
    curr_param.is_started = true;
  }

  Drive(curr_param);

  if (end_condition_->IsSatisfied()) {
    driving_params_.pop_front();
  }

  if (driving_params_.empty()) {
    basic_driver_->Stop();
  }*/
}

void DrivingManager::AddDrivingParam(DrivingParam param) {
  //driving_params_.push_back(param);
}

bool DrivingManager::DrivingParamsEmpty() {
  //return driving_params_.empty();
  return false;
}

void DrivingManager::SetDriveParam(DrivingParam& param) {
  Move move_type = param.move_type;
  int8_t base_power = param.base_power;
  Gain gain = param.gain;

  switch (move_type) {
    case kTraceLeftEdge:
    case kTraceRightEdge:
      line_tracer_->SetParam(move_type, base_power, gain);
      break;

    case kGoForward:
    case kGoBackward:
    case kRotateLeft:
    case kRotateRight:
      basic_driver_->SetParam(move_type, base_power);
      break;

    default:
      break;
  }
}

void DrivingManager::SetEndParam(DrivingParam& param) {
  End end_type = param.end_type;
  Color end_color = param.end_color;
  float end_threshold = param.end_threshold;

  end_condition_->SetParam(end_type, end_color, end_threshold);
}

void DrivingManager::Drive(DrivingParam& param) {
  Move move_type = param.move_type;

  switch (move_type) {
    case kTraceLeftEdge:
    case kTraceRightEdge:
      line_tracer_->Run();
      break;

    case kGoForward:
    case kGoBackward:
    case kRotateLeft:
    case kRotateRight:
      basic_driver_->Run();
      break;

    default:
      break;
  }
}
