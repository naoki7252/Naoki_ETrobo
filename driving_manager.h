#ifndef ETRC22_DRIVING_MANAGER_H_
#define ETRC22_DRIVING_MANAGER_H_

#include "driving_type.h"
#include "end_condition.h"
#include "info_type.h"
#include "utils.h"

//#include <list>

class DrivingManager {
 public:
  DrivingManager(BasicDriver* basic_driver, LineTracer* line_tracer, EndCondition* end_condition);
  void Update();
  void AddDrivingParam(DrivingParam param);
  bool DrivingParamsEmpty();

 private:
  void SetDriveParam(DrivingParam& param);
  void SetEndParam(DrivingParam& param);
  void Drive(DrivingParam& param);
  BasicDriver* basic_driver_;
  LineTracer* line_tracer_;
  EndCondition* end_condition_;
  //std::list<DrivingParam> driving_params_;
};

#endif  // ETRC22_DRIVING_MANAGER_H_
