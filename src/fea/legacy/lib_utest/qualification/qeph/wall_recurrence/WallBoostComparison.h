#pragma once
#include "WallJobAnalysis.h"

namespace tl::qualification::qeph::wall_recurrence {
struct WallSequenceGainSummary {
  double raw=0,weighted=0;
  bool complete=false;
};
struct WallAmplitudeSummary {
  std::array<WallSequenceGainSummary,11> gains;
  bool complete=false,passed=false;
};
struct WallStepSummary {
  double h=0;
  Eigen::VectorXd diagonal;
  unsigned total_steps=0,ordinary_steps=0;
  std::array<WallSwitchingWindow,9> windows{};
  std::array<WallAmplitudeSummary,3> amplitudes;
  bool complete=false,passed=false;
};
// Numerical measurements from a fresh WallJobAnalysis or a derived reader
// that has checked root-pinned producer/indices and rederived verdicts from
// the authenticated fields. This POD does not authenticate its own values.
// No global job.passed field is used or retained.
struct WallJobSummary {
  unsigned cells=0,dimension=0;
  double normal_velocity=0;
  std::array<WallStepSummary,6> steps;
};
bool ValidateWallJobSummary(const WallJobSummary&,std::string&);
// Stages a compact copy of a fresh/reader-validated result; no Schur/Gram solve.
// It checks structure/context but does not replace the producer trust boundary.
bool CompactWallJobAnalysis(const RawJob&,const WallJobAnalysis&,WallJobSummary&,std::string&);

class WallZeroBoostReference {
 public:
  WallZeroBoostReference()=default;
  WallZeroBoostReference(const WallZeroBoostReference&)=delete;
  WallZeroBoostReference& operator=(const WallZeroBoostReference&)=delete;
  WallZeroBoostReference(WallZeroBoostReference&&)=default;
  WallZeroBoostReference& operator=(WallZeroBoostReference&&)=default;
  bool prepared() const { return prepared_; }
 private:
  WallRecurrenceModel model_;
  WallJobSummary summary_;
  std::array<std::array<MovingMatrixProbe,3>,6> native_;
  std::array<std::array<bool,3>,6> attempted_{};
  bool prepared_=false;
  friend bool PrepareWallZeroBoostReference(const RawJob&,const WallJobSummary&,WallZeroBoostReference&,std::string&);
  friend struct WallBoostAccess;
};
// Initialize once from complete identity/context. Individual failed numerical
// points remain available; in particular a failed 4H0 does not prevent caching.
bool PrepareWallZeroBoostReference(const RawJob&,const WallJobSummary&,WallZeroBoostReference&,std::string&);

struct WallBoostAmplitudeComparison {
  std::array<WallBaselineAnalysis,2> baselines; // Zero, then selected boost.
  // Actual baseline difference minus the expected hV/V lift. Diagnostic only;
  // the two independent absolute baseline gates retain their frozen budgets.
  double lift_residual_max=0;
  unsigned lift_controlling_coordinate=0;
  WallDifference native_matrix;
  std::array<WallDifference,2> branch_matrices;
  std::array<WallDifference,11> raw_gains,weighted_gains;
  bool complete=false,passed=false;
  std::string diagnostic;
};
struct WallBoostStepComparison {
  double h=0;
  std::array<WallBoostAmplitudeComparison,3> amplitudes;
  bool complete=false,passed=false;
};
struct WallBoostComparison {
  unsigned cells=0,dimension=0;
  double normal_velocity=0;
  std::array<WallBoostStepComparison,6> steps;
  bool input_valid=false;
  std::string diagnostic;
};
// One +/-8 job at a time; zero operands remain immutable. Complete numerical
// failures are returned with all available measurements. No native calls,
// spectra/Gram solves, file IO or source/receipt authentication occurs here.
WallBoostComparison CompareWallBoost(const WallZeroBoostReference&,const RawJob&,const WallJobSummary&);
} // namespace tl::qualification::qeph::wall_recurrence
