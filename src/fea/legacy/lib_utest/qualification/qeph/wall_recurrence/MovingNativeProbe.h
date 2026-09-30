#pragma once
#include "lib_utest/qualification/qeph/free_response/RecurrenceNativeMap.h"

namespace tl::qualification::qeph::wall_recurrence {
struct MovingMatrixProbe {
  Vec3 velocity{}; // Physical common carried velocity, m/s.
  Eigen::VectorXd baseline;
  bool baseline_complete=false;
  recurrence::MatrixProbe derivative;
};
// Collect the full native matrix without a feedback reduction or an admission
// decision. The actual moving baseline and every completed column survive a
// later probe failure. No baseline residual is clamped to zero. Centering
// cancels the baseline algebraically, using the original difference ordering.
MovingMatrixProbe DifferentiateMoving(const recurrence::Model&,double h,
                                     double amplitude,const Vec3& velocity);
} // namespace tl::qualification::qeph::wall_recurrence
