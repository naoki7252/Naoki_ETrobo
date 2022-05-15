#include "driving_manager.h"

DrivingManager::DrivingManager(BasicDriver* basic_driver, LineTracer* line_tracer, EndCondition* end_condition)
    : basic_driver_(basic_driver), line_tracer_(line_tracer), end_condition_(end_condition) {
}

void DrivingManager::Update() {
  if (is_satisfied) {
    is_satisfied = false;
    return;
  }

  Drive(curr_param);

  if (end_condition_->IsSatisfied()) {
    is_satisfied = true;
  }
}

void DrivingManager::SetDriveParam(DrivingParam param) {
  curr_param = param;
  SetMoveParam(curr_param);
  SetEndParam(curr_param);
}

void DrivingManager::SetMoveParam(DrivingParam& param) {
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
