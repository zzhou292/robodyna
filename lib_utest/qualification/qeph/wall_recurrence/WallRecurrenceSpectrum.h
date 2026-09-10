#pragma once
#include "WallStateMetric.h"
#include "lib_utest/qualification/qeph/free_response/RecurrencePowerGram.h"
#include <complex>
#include <vector>

namespace tl::qualification::qeph::wall_recurrence {
struct WallSpectrum {
  bool complete=false,passed=false;
  double matrix_norm=0,schur_residual=0,orthogonality_error=0,spectral_radius=0;
  unsigned near_one=0,near_zero=0;
  std::vector<std::complex<double>> eigenvalues;
  std::string diagnostic;
};
// Only constant branches receive this radius test. A finite event product is
// never treated as an indefinitely repeated map.
WallSpectrum AnalyzeConstantWallSpectrum(const Eigen::MatrixXd&);

struct WallGramAnalysis {
  bool complete=false,within_gain_budget=false;
  unsigned ordinary_steps=0,state_count=0,controlling_coordinate=0;
  double gram_norm=0,antisymmetry=0,minimum_eigenvalue=0,maximum_eigenvalue=0;
  double mean_gain=0,individual_power_bound=0;
  Eigen::VectorXd eigenvalues,controlling_direction;
  std::string diagnostic;
};
// Builds G_0..N including P_N^T P_N exactly once. The result keeps measured
// values when a numerical budget fails. Small analytic matrices are supported.
WallGramAnalysis AnalyzeWallGram(const recurrence::PowerGramBlock&);

struct WallOperatorRun {
  const Eigen::MatrixXd* matrix=nullptr; // Borrowed only for this call.
  unsigned count=0;
};
struct WallSequenceAnalysis {
  WallGramAnalysis raw,weighted;
  bool complete=false,passed=false;
  std::string diagnostic;
};
// At most 64 chronological runs and 32768 ordinary transitions. This bounded
// mathematical seam also tests noncommuting analytic sequences independently
// of the named experiment's three-run schedule. No input pointer is retained.
WallSequenceAnalysis AnalyzeWallSequence(const std::vector<WallOperatorRun>&,
                                        const Eigen::VectorXd& diagonal);
} // namespace tl::qualification::qeph::wall_recurrence
