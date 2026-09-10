#include "WallDerivedReadFields.h"
#include <algorithm>
#include <cmath>

namespace tl::qualification::qeph::wall_recurrence::derived_read {
WallSpectrum Spectrum(const io::Value& v,const Eigen::MatrixXd& matrix,unsigned dimension) {
  WallSpectrum out; const bool declared=rd::Boolean(rd::Field(v,"complete"));
  out.diagnostic=Diagnostic(v);
  out.matrix_norm=rd::Scalar(rd::Field(v,"matrix_norm"),!declared);
  out.schur_residual=rd::Scalar(rd::Field(v,"schur_residual"),!declared);
  out.orthogonality_error=rd::Scalar(rd::Field(v,"orthogonality_error"),!declared);
  const auto& values=rd::Field(v,"eigenvalues_real_imaginary");
  io::Require(values.IsArray()&&(values.Empty()||values.Size()==dimension),"Invalid retained full spectrum extent");
  if(matrix.size()) {
    io::Require(io::Bits(out.matrix_norm)==io::Bits(matrix.norm()),"Spectrum norm differs from reconstructed operator");
  } else io::Require(values.Empty(),"Spectrum has no reconstructed operator");
  for(const auto& pair:values.GetArray()) {
    io::Require(pair.IsArray()&&pair.Size()==2,"Invalid retained complex eigenvalue");
    const std::complex<double> value(rd::Number(pair[0]),rd::Number(pair[1]));
    out.eigenvalues.push_back(value); out.spectral_radius=std::max(out.spectral_radius,std::abs(value));
    out.near_one+=std::abs(value-std::complex<double>(1,0))<=1e-6;
    out.near_zero+=std::abs(value)<=1e-6;
  }
  if(!values.Empty()) {
    io::Require(out.matrix_norm>=0&&out.schur_residual>=0&&out.orthogonality_error>=0,
                "Negative spectrum residual/norm");
    out.complete=std::isfinite(out.schur_residual)&&std::isfinite(out.orthogonality_error)&&
      std::isfinite(out.spectral_radius);
    out.passed=out.complete&&out.schur_residual<=recurrence::DecompositionTolerance&&
      out.orthogonality_error<=recurrence::DecompositionTolerance&&out.spectral_radius<=1+recurrence::MatrixTolerance;
  }
  Same(v,job_json::Spectrum(out,dimension),"Changed spectrum shape, extrema, counts or verdict");
  return out;
}

WallGramAnalysis Gram(const io::Value& v,unsigned dimension,unsigned ordinary_steps) {
  WallGramAnalysis out; const bool declared=rd::Boolean(rd::Field(v,"complete"));
  out.diagnostic=Diagnostic(v);
  out.ordinary_steps=rd::Count(rd::Field(v,"ordinary_steps"),32768);
  out.state_count=rd::Count(rd::Field(v,"state_count"),32769);
  io::Require((out.state_count==0&&out.ordinary_steps==0)||
    (out.ordinary_steps==ordinary_steps&&out.state_count==ordinary_steps+1),"Gram phase/count differs from chronological context");
  out.gram_norm=rd::Scalar(rd::Field(v,"gram_norm"),!declared);
  out.antisymmetry=rd::Scalar(rd::Field(v,"antisymmetry"),!declared);
  out.eigenvalues=rd::Vector(rd::Field(v,"eigenvalues"));
  out.controlling_direction=rd::Vector(rd::Field(v,"controlling_direction"));
  io::Require((out.eigenvalues.size()==0&&out.controlling_direction.size()==0)||
    (out.eigenvalues.size()==dimension&&out.controlling_direction.size()==dimension),"Incomplete Gram eigensystem shape");
  if(out.eigenvalues.size()) {
    io::Require(out.state_count==ordinary_steps+1&&std::isfinite(out.gram_norm)&&out.gram_norm>=0&&
      std::isfinite(out.antisymmetry)&&out.antisymmetry>=0&&out.antisymmetry<=recurrence::DecompositionTolerance,
      "Gram eigensystem precedes a resolved symmetry/count check");
    for(unsigned i=1;i<dimension;++i)
      io::Require(out.eigenvalues[i]>=out.eigenvalues[i-1],"Gram eigenvalues are not sorted");
    out.minimum_eigenvalue=out.eigenvalues[0]; out.maximum_eigenvalue=out.eigenvalues[dimension-1];
    Eigen::Index controlling=0; out.controlling_direction.cwiseAbs().maxCoeff(&controlling);
    out.controlling_coordinate=static_cast<unsigned>(controlling);
    if(out.minimum_eigenvalue>=-recurrence::DecompositionTolerance*std::max(1.,out.gram_norm)&&out.maximum_eigenvalue>=0) {
      out.mean_gain=std::sqrt(out.maximum_eigenvalue/out.state_count);
      out.individual_power_bound=std::sqrt(out.maximum_eigenvalue);
      out.complete=std::isfinite(out.mean_gain)&&std::isfinite(out.individual_power_bound);
      out.within_gain_budget=out.complete&&out.mean_gain<=recurrence::MaximumGramGain;
    }
  }
  Same(v,job_json::Gram(out,dimension),"Changed Gram extrema, direction, gain, count or verdict");
  return out;
}
WallSequenceAnalysis Sequence(const io::Value& v,unsigned dimension,unsigned ordinary_steps) {
  WallSequenceAnalysis out; out.diagnostic=Diagnostic(v);
  out.raw=Gram(rd::Field(v,"raw"),dimension,ordinary_steps);
  out.weighted=Gram(rd::Field(v,"weighted"),dimension,ordinary_steps);
  out.complete=out.raw.complete&&out.weighted.complete;
  out.passed=out.complete&&out.weighted.within_gain_budget;
  Same(v,job_json::Sequence(out,dimension),"Changed sequence verdict; raw gain is diagnostic");
  return out;
}
} // namespace tl::qualification::qeph::wall_recurrence::derived_read
