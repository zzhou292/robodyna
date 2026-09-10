#pragma once
#include "ContactBranchProbe.h"
#include "WallRecurrenceSpectrum.h"
#include "WallStateIdentities.h"
#include "WallSwitchingSchedule.h"

namespace tl::qualification::qeph::wall_recurrence {
constexpr double WallGainRelativeTolerance=.005,WallGainAbsoluteTolerance=1e-10;
struct WallConstantAnalysis {
  ContactBranch branch=ContactBranch::Inactive;
  Eigen::MatrixXd full,weighted;
  WallIdentityAnalysis identities;
  WallSpectrum spectrum,weighted_spectrum;
  WallSequenceAnalysis continuous;
};
struct WallEventAnalysis {
  WallSwitchingWindow window;
  WallSequenceAnalysis sequence;
};
struct WallBranchAnalysis {
  double h=0;
  WallStateMetric metric;
  WallSwitchingSchedule schedule;
  std::array<WallConstantAnalysis,2> branches;
  std::array<WallEventAnalysis,9> events;
  unsigned completed_branches=0,completed_events=0;
  bool complete=false,passed=false;
  std::string diagnostic;
};
// Pure analysis of a supplied full native derivative. The caller must bind it
// to retained raw probes, h/amplitude/boost, baseline and directional checks.
// All numerical evidence is retained on a valid failed screen; no owner or
// trajectory admission is produced by this function.
WallBranchAnalysis AnalyzeWallBranches(const WallRecurrenceModel&,double h,const Eigen::MatrixXd& shell);

struct WallDifference {
  double difference=0,budget=0;
  bool complete=false,passed=false;
};
WallDifference CompareWallMatrices(const Eigen::MatrixXd&,const Eigen::MatrixXd&);
WallDifference CompareWallGains(double,double);
struct WallAnalysisComparison {
  std::array<WallDifference,2> matrices;
  // Continuous inactive, continuous active, then entry-major nine windows.
  // Raw comparisons are diagnostics; weighted mean differences control the
  // frozen consistency decision. Both require finite reported measurements.
  std::array<WallDifference,11> raw_gains,weighted_gains;
  bool complete=false,passed=false;
  std::string diagnostic;
};
WallAnalysisComparison CompareWallAnalyses(const WallBranchAnalysis&,const WallBranchAnalysis&);
// Pure frozen selector over already aggregated fixture/amplitude/boost verdicts.
// A returned number does not authenticate those verdicts or admit a trajectory.
double SelectWallScreenStep(const std::array<bool,6>& all_job_verdicts) noexcept;
} // namespace tl::qualification::qeph::wall_recurrence
