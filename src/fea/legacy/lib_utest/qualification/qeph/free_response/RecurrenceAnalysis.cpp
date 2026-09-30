#include "RecurrenceIdentity.h"
#include "RecurrencePowerGram.h"
#include <Eigen/Eigenvalues>
#include <cmath>

namespace tl::qualification::qeph::recurrence {
bool PowerGram(const Eigen::MatrixXd& a,unsigned count,Eigen::MatrixXd& output,std::string& error) {
  if(a.rows()<1||a.rows()!=a.cols()||a.rows()>194||!a.allFinite()||count<1||count>32769) {
    error="Invalid finite-horizon Gram dimensions/count"; return false;
  }
  PowerGramBlock block;
  if(!BuildPowerGramBlock(a,count,block,error)) return false;
  output=std::move(block.gram); error.clear(); return true;
}
MapAnalysis Analyze(const Model& m,double h,const MatrixProbe& p) {
  MapAnalysis out;
  if(!p.complete||p.full.rows()!=static_cast<Eigen::Index>(m.dictionary.size())||p.full.rows()!=p.full.cols()||!p.full.allFinite()) {
    out.diagnostic="Incomplete/nonfinite native matrix"; return out;
  }
  const double intervals=4096*H0/h;
  if(!std::isfinite(intervals)||intervals<1||intervals>32768||intervals!=std::floor(intervals)) {
    out.diagnostic="Unadmitted finite-horizon step count"; return out;
  }
  if(!CheckStructuralIdentities(m,h,p.full,out)) return out;
  if(out.zero_feedback_error>MatrixTolerance) {
    out.diagnostic="Claimed passive columns feed mechanics; block reduction is unresolved"; return out;
  }
  const auto indices=FeedbackIndices(m); Eigen::MatrixXd a(indices.size(),indices.size());
  for(unsigned row=0;row<indices.size();++row) for(unsigned col=0;col<indices.size();++col) a(row,col)=p.full(indices[row],indices[col]);
  Eigen::RealSchur<Eigen::MatrixXd> schur(a);
  if(schur.info()!=Eigen::Success||!schur.matrixT().allFinite()||!schur.matrixU().allFinite()) {
    out.diagnostic="Nonsymmetric Schur decomposition failed"; return out;
  }
  const auto& u=schur.matrixU(); const auto& t=schur.matrixT();
  out.schur_residual=(a-u*t*u.transpose()).norm()/std::max(1.,a.norm());
  out.orthogonality_error=(u.transpose()*u-Eigen::MatrixXd::Identity(a.rows(),a.cols())).norm();
  // EigenSolver supplies the conjugate spectrum; Schur residual above is the
  // numerical decomposition check. No eigenvector inversion or symmetrizing A.
  Eigen::EigenSolver<Eigen::MatrixXd> spectrum(a,false);
  if(spectrum.info()!=Eigen::Success||!spectrum.eigenvalues().allFinite()) {
    out.diagnostic="Nonsymmetric eigenvalue extraction failed"; return out;
  }
  for(const auto value:spectrum.eigenvalues()) {
    out.eigenvalues.push_back(value); out.spectral_radius=std::max(out.spectral_radius,std::abs(value));
    if(std::abs(value-std::complex<double>(1,0))<=1e-6) ++out.near_one;
    if(std::abs(value)<=1e-6) ++out.near_zero;
  }
  Eigen::MatrixXd gram;
  if(!PowerGram(a,1+static_cast<unsigned>(intervals),gram,out.diagnostic)) return out;
  out.gram_antisymmetry=(gram-gram.transpose()).norm()/std::max(1.,gram.norm());
  if(!std::isfinite(out.gram_antisymmetry)||out.gram_antisymmetry>DecompositionTolerance) {
    out.diagnostic="Finite-horizon Gram lost declared numerical symmetry"; return out;
  }
  const Eigen::MatrixXd symmetric=(.5*(gram+gram.transpose())).eval();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> gs(symmetric,Eigen::EigenvaluesOnly);
  if(gs.info()!=Eigen::Success||!gs.eigenvalues().allFinite()||gs.eigenvalues()[0]<-DecompositionTolerance*std::max(1.,gram.norm())||
     gs.eigenvalues().tail(1)[0]<0) { out.diagnostic="Finite-horizon Gram spectrum failed"; return out; }
  out.gram_gain=std::sqrt(gs.eigenvalues().tail(1)[0]/(1+intervals));
  out.complete=true;
  out.passed=out.schur_residual<=DecompositionTolerance&&out.orthogonality_error<=DecompositionTolerance&&
    out.spectral_radius<=1+MatrixTolerance&&out.gram_gain<=MaximumGramGain&&
    out.zero_feedback_error<=MatrixTolerance&&out.observer_error<=MatrixTolerance&&out.rigid_error<=MatrixTolerance&&
    out.rigid_basis_condition<=1e8&&out.rigid_projection_residual<=DecompositionTolerance;
  if(!out.passed) out.diagnostic="One or more frozen map/spectrum/Gram/neutral-identity budgets failed";
  return out;
}
} // namespace tl::qualification::qeph::recurrence
