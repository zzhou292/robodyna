#include "WallRecurrenceSpectrum.h"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>

namespace tl::qualification::qeph::wall_recurrence {
WallSpectrum AnalyzeConstantWallSpectrum(const Eigen::MatrixXd& a) {
  WallSpectrum out;
  if(a.rows()<1||a.rows()>194||a.rows()!=a.cols()||!a.allFinite()) {
    out.diagnostic="Invalid full constant-branch matrix"; return out;
  }
  out.matrix_norm=a.norm();
  if(!std::isfinite(out.matrix_norm)) { out.diagnostic="Nonfinite matrix norm"; return out; }
  Eigen::RealSchur<Eigen::MatrixXd> schur(a);
  if(schur.info()!=Eigen::Success||!schur.matrixT().allFinite()||!schur.matrixU().allFinite()) {
    out.diagnostic="Full nonsymmetric Schur decomposition failed"; return out;
  }
  const auto& u=schur.matrixU(); const auto& t=schur.matrixT();
  out.schur_residual=(a-u*t*u.transpose()).norm()/std::max(1.,out.matrix_norm);
  out.orthogonality_error=(u.transpose()*u-Eigen::MatrixXd::Identity(a.rows(),a.cols())).norm();
  Eigen::EigenSolver<Eigen::MatrixXd> spectrum(a,false);
  if(spectrum.info()!=Eigen::Success||!spectrum.eigenvalues().allFinite()) {
    out.diagnostic="Full nonsymmetric eigenvalue extraction failed"; return out;
  }
  for(const auto value:spectrum.eigenvalues()) {
    out.eigenvalues.push_back(value); out.spectral_radius=std::max(out.spectral_radius,std::abs(value));
    if(std::abs(value-std::complex<double>(1,0))<=1e-6) ++out.near_one;
    if(std::abs(value)<=1e-6) ++out.near_zero;
  }
  out.complete=std::isfinite(out.schur_residual)&&std::isfinite(out.orthogonality_error)&&
    std::isfinite(out.spectral_radius);
  out.passed=out.complete&&out.schur_residual<=recurrence::DecompositionTolerance&&
    out.orthogonality_error<=recurrence::DecompositionTolerance&&
    out.spectral_radius<=1+recurrence::MatrixTolerance;
  if(!out.passed) out.diagnostic="Full constant-branch Schur/radius budget failed";
  return out;
}

WallGramAnalysis AnalyzeWallGram(const recurrence::PowerGramBlock& block) {
  WallGramAnalysis out; out.ordinary_steps=block.count; out.state_count=block.count+1;
  Eigen::MatrixXd gram;
  if(!recurrence::EndpointInclusiveGram(block,gram,out.diagnostic)) return out;
  out.gram_norm=gram.norm();
  out.antisymmetry=(gram-gram.transpose()).norm()/std::max(1.,out.gram_norm);
  if(!std::isfinite(out.gram_norm)||!std::isfinite(out.antisymmetry)||
     out.antisymmetry>recurrence::DecompositionTolerance) {
    out.diagnostic="Endpoint-inclusive Gram lost declared symmetry"; return out;
  }
  // Symmetrization occurs only after the original antisymmetry was measured.
  const Eigen::MatrixXd symmetric=(.5*(gram+gram.transpose())).eval();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(symmetric);
  if(spectrum.info()!=Eigen::Success||!spectrum.eigenvalues().allFinite()||!spectrum.eigenvectors().allFinite()) {
    out.diagnostic="Endpoint-inclusive Gram spectrum failed"; return out;
  }
  out.eigenvalues=spectrum.eigenvalues();
  out.minimum_eigenvalue=out.eigenvalues[0]; out.maximum_eigenvalue=out.eigenvalues.tail(1)[0];
  out.controlling_direction=spectrum.eigenvectors().col(gram.rows()-1);
  Eigen::Index controlling=0; out.controlling_direction.cwiseAbs().maxCoeff(&controlling);
  out.controlling_coordinate=static_cast<unsigned>(controlling);
  if(out.minimum_eigenvalue < -recurrence::DecompositionTolerance*std::max(1.,out.gram_norm)||
     out.maximum_eigenvalue<0) {
    out.diagnostic="Unresolved nonpositive Gram spectrum"; return out;
  }
  out.mean_gain=std::sqrt(out.maximum_eigenvalue/out.state_count);
  out.individual_power_bound=std::sqrt(out.maximum_eigenvalue);
  out.complete=std::isfinite(out.mean_gain)&&std::isfinite(out.individual_power_bound);
  out.within_gain_budget=out.complete&&out.mean_gain<=recurrence::MaximumGramGain;
  if(!out.within_gain_budget) out.diagnostic="Mean Gram gain exceeds the frozen budget";
  return out;
}
} // namespace tl::qualification::qeph::wall_recurrence
