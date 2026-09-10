#pragma once
#include "WallRecurrenceModel.h"

namespace tl::qualification::qeph::wall_recurrence {
constexpr double ScreenHorizon=4096*recurrence::H0;
constexpr double MaximumMetricCondition=1e8;
struct WallStateMetric {
  Eigen::VectorXd diagonal;
  double sound_speed=0,eta=0,tangential_weight=0,normal_weight=0,condition=0;
};
// The full native dictionary is retained. D depends on the named experiment
// and horizon, never on h, measured matrices, or selected modes.
bool BuildWallStateMetric(const WallRecurrenceModel&,WallStateMetric&,std::string&);
// Mathematical seam also admits small analytic matrices. No coordinates are
// projected out; malformed, singular or excessive-condition D preserves out.
bool ApplyWallStateMetric(const Eigen::MatrixXd&,const Eigen::VectorXd& diagonal,
                          Eigen::MatrixXd& out,std::string&);
bool ValidWallStateDiagonal(const Eigen::VectorXd&) noexcept;
} // namespace tl::qualification::qeph::wall_recurrence
