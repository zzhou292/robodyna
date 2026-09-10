#pragma once
#include "WallRawCapture.h"
#include "WallRecurrenceAnalysis.h"
#include <functional>

namespace tl::qualification::qeph::wall_recurrence {
struct WallDirectionRecheck {
  std::string name;
  std::array<Eigen::VectorXd,2> quotients;
  std::array<double,2> residual_upper{},budget_lower{};
  bool samples_valid=false,complete=false,passed=false;
  std::string diagnostic;
};
struct WallContactRecheck {
  ContactBranch branch=ContactBranch::Inactive;
  WallDifference operator_difference;
  WallBaselineAnalysis baseline;
  std::vector<WallDirectionRecheck> directions;
  unsigned completed_directions=0;
  bool input_valid=false,complete=false,passed=false;
  std::string diagnostic;
};
// Rebuilds the branch and frozen directions, then recomputes both quotients
// from actual recorded baseline/sample states. Saved pass bits, quotients and
// budgets are evidence only and are never trusted as a numerical verdict.
WallContactRecheck RecheckWallContact(const WallRecurrenceModel&,double h,double normal_velocity,
                                    ContactBranch,const Eigen::MatrixXd& shell,const ContactBranchProbe&);
struct WallAmplitudeAnalysis {
  double amplitude=0;
  WallBaselineAnalysis baseline;
  WallBranchAnalysis branches;
  bool attempted=false,input_complete=false,complete=false,passed=false;
  std::string diagnostic;
};
struct WallStepAnalysis {
  double h=0;
  std::array<WallAmplitudeAnalysis,3> amplitudes;
  std::array<WallDifference,2> native_amplitude_comparisons;
  std::array<WallAnalysisComparison,2> derived_amplitude_comparisons;
  std::array<WallContactRecheck,2> contact;
  bool complete=false,passed=false;
  std::string diagnostic;
};
struct WallJobAnalysis {
  unsigned cells=0,dimension=0;
  double normal_velocity=0;
  std::array<WallStepAnalysis,6> steps;
  unsigned completed_amplitudes=0,completed_contacts=0,completed_steps=0;
  bool input_valid=false,complete=false,passed=false;
  std::string diagnostic;
};
enum class WallJobProgressKind { Amplitude,Contact,Step,Finished };
struct WallJobProgress { WallJobProgressKind kind; unsigned step=0,index=0; };
using WallJobProgressCallback=std::function<void(const WallJobAnalysis&,WallJobProgress)>;
// One authenticated-read RawJob at a time. It retains failed component results
// and calls progress after each amplitude/contact/step; callback exceptions
// stop immediately. No reader, native evaluation, timestep selector or boost
// comparison is hidden here. Input bytes/provenance must be authenticated by
// the caller; complete native matrices remain recorded observations.
WallJobAnalysis AnalyzeWallRawJob(const RawJob&,const WallJobProgressCallback& callback={});
} // namespace tl::qualification::qeph::wall_recurrence
