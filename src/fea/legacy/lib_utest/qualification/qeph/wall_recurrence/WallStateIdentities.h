#pragma once
#include "MovingNativeProbe.h"
#include "WallStateMetric.h"

namespace tl::qualification::qeph::wall_recurrence {
struct WallIdentityAnalysis {
  bool complete=false,passed=false;
  double observer_error=0,cached_kick_error=0,neutral_error=0;
  // Historical free-shell omitted-coordinate classification only. Positions
  // feed active wall mechanics and are not physically passive in this screen.
  // This diagnostic never removes coordinates or imposes a zero-feedback gate.
  double passive_feedback_max=0;
  unsigned passive_feedback_row=0,passive_feedback_column=0;
  std::vector<double> neutral_errors; // Y/Z translation, X spin, nodewise X drilling.
  std::string diagnostic;
};
// Check full rows, including passive history/cache output. Only wall-compatible
// neutral directions are tested; none are projected out of the matrix.
WallIdentityAnalysis CheckWallStateIdentities(const WallRecurrenceModel&,double h,
                                            const Eigen::MatrixXd&);
struct WallBaselineAnalysis {
  bool complete=false,passed=false;
  double maximum_error=0;
  unsigned controlling_coordinate=0;
  Eigen::VectorXd expected,residual;
  std::string diagnostic;
};
// Checks actual moving output before centered differences are interpreted.
// Expected physical drift and carried velocity remain visible; zero material,
// cache and work increments are compared, never snapped into the raw record.
WallBaselineAnalysis CheckWallMovingBaseline(const WallRecurrenceModel&,double h,
                                           const MovingMatrixProbe&);
} // namespace tl::qualification::qeph::wall_recurrence
