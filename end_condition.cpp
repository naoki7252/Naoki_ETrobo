#include "end_condition.h"

EndCondition::EndCondition(Luminous* luminous, Localize* localize)
    : luminous_(luminous), localize_(localize),
      end_type_(kInvalidEnd), end_color_(kInvalidColor), end_threshold_(0),
      end_state_(false), ref_distance_(0), ref_theta_(0) {
}

void EndCondition::SetParam(End end_type, Color end_color, float end_threshold) {
  end_type_ = end_type;
  end_color_ = end_color;
  end_threshold_ = end_threshold;
  end_state_ = false;

  // if (end_type_ == kDistanceEnd) {
  //   ref_distance_ = localize_->distance_;
  // } else if (end_type_ == kThetaEnd) {
  //   ref_theta_ = localize_->pose_.theta;
  // }
}

bool EndCondition::IsSatisfied() {
  switch (end_type_) {
    case kColorEnd:
      if (end_color_ == luminous_->color_)
        end_state_ = true;
      break;

    // case kDistanceEnd:
    //   if (end_threshold_ > 0 && localize_->distance_ - ref_distance_ > end_threshold_)
    //     end_state_ = true;
    //   else if (end_threshold_ < 0 && localize_->distance_ - ref_distance_ < end_threshold_)
    //     end_state_ = true;
    //   break;

    // case kThetaEnd:
    //   if (end_threshold_ > 0 && localize_->pose_.theta - ref_theta_ > end_threshold_)
    //     end_state_ = true;
    //   else if (end_threshold_ < 0 && localize_->pose_.theta - ref_theta_ < end_threshold_)
    //     end_state_ = true;
    //   break;

    default:
      break;
  }

  return end_state_;
}