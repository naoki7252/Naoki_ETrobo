#ifndef ETRC22_END_CONDITION_H_
#define ETRC22_END_CONDITION_H_

#include "etrc_info.h"

class EndCondition {
 public:
  EndCondition(Luminous* luminous, Localize* localize);
  void SetParam(End end_type, Color end_color, float end_threshold);
  bool IsSatisfied();

 private:
  Luminous* luminous_;
  Localize* localize_;
  End end_type_;
  Color end_color_;
  float end_threshold_;
  bool end_state_;
  float ref_distance_;
  float ref_theta_;
};

#endif  // ETRC22_END_CONDITION_H_