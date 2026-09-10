#include "WallJobJson.h"

namespace tl::qualification::qeph::wall_recurrence {
std::string WallDerivedMatrixHash(const Eigen::MatrixXd& matrix) {
  return crash::output::Sha256(EncodeRawJson(raw_detail::Matrix(matrix,true)));
}
}
namespace tl::qualification::qeph::wall_recurrence::job_json {
void Scalar(io::Document& d,const char* name,double value,bool partial) {
  d.AddMember(io::Value(name,d.GetAllocator()),raw_detail::Scalar(d,value,partial),d.GetAllocator());
}
void Status(io::Document& d,bool complete,bool passed,const std::string& diagnostic) {
  io::Require(!passed||complete,"Passed derived field is incomplete");
  io::Require(diagnostic.size()<=4096,"Unbounded derived diagnostic");
  io::Boolean(d,"complete",complete); io::Boolean(d,"passed",passed); io::String(d,"diagnostic",diagnostic);
}
io::Document Baseline(const WallBaselineAnalysis& a,unsigned dimension) {
  io::Require((a.expected.size()==0||a.expected.size()==dimension)&&(a.residual.size()==0||a.residual.size()==dimension)&&
    (!a.complete||(a.expected.size()==dimension&&a.residual.size()==dimension))&&a.controlling_coordinate<dimension,
    "Invalid derived baseline dimensions");
  io::Document d; d.SetObject(); Status(d,a.complete,a.passed,a.diagnostic);
  Scalar(d,"maximum_error",a.maximum_error,!a.complete); io::Integer(d,"controlling_coordinate",a.controlling_coordinate);
  d.AddMember("expected",raw_detail::Vector(d,a.expected,!a.complete),d.GetAllocator());
  d.AddMember("residual",raw_detail::Vector(d,a.residual,!a.complete),d.GetAllocator()); return d;
}
io::Document Identities(const WallIdentityAnalysis& a,unsigned dimension) {
  io::Require(a.neutral_errors.size()<=10&&a.passive_feedback_row<dimension&&a.passive_feedback_column<dimension,
              "Invalid identity diagnostics dimensions");
  io::Document d; d.SetObject(); Status(d,a.complete,a.passed,a.diagnostic);
  Scalar(d,"observer_error",a.observer_error,!a.complete); Scalar(d,"cached_kick_error",a.cached_kick_error,!a.complete);
  Scalar(d,"neutral_error",a.neutral_error,!a.complete); Scalar(d,"passive_feedback_max",a.passive_feedback_max,!a.complete);
  io::Integer(d,"passive_feedback_row",a.passive_feedback_row); io::Integer(d,"passive_feedback_column",a.passive_feedback_column);
  io::Value errors(rapidjson::kArrayType);
  for(double x:a.neutral_errors) errors.PushBack(raw_detail::Scalar(d,x,!a.complete),d.GetAllocator());
  d.AddMember("neutral_errors",errors,d.GetAllocator()); return d;
}
io::Document Spectrum(const WallSpectrum& a,unsigned dimension) {
  io::Require(a.eigenvalues.size()<=dimension&&(!a.complete||a.eigenvalues.size()==dimension)&&
    a.near_one<=dimension&&a.near_zero<=dimension,"Invalid full spectrum dimensions/counts");
  io::Document d; d.SetObject(); Status(d,a.complete,a.passed,a.diagnostic);
  Scalar(d,"matrix_norm",a.matrix_norm,!a.complete); Scalar(d,"schur_residual",a.schur_residual,!a.complete);
  Scalar(d,"orthogonality_error",a.orthogonality_error,!a.complete); Scalar(d,"spectral_radius",a.spectral_radius,!a.complete);
  io::Integer(d,"near_one",a.near_one); io::Integer(d,"near_zero",a.near_zero); io::Value values(rapidjson::kArrayType);
  for(const auto value:a.eigenvalues) {
    io::Value pair(rapidjson::kArrayType); pair.PushBack(raw_detail::Scalar(d,value.real(),!a.complete),d.GetAllocator());
    pair.PushBack(raw_detail::Scalar(d,value.imag(),!a.complete),d.GetAllocator()); values.PushBack(pair,d.GetAllocator());
  }
  d.AddMember("eigenvalues_real_imaginary",values,d.GetAllocator()); return d;
}
io::Document Gram(const WallGramAnalysis& a,unsigned dimension) {
  io::Require(a.ordinary_steps<=32768&&a.state_count<=32769&&a.controlling_coordinate<dimension&&
    (a.eigenvalues.size()==0||a.eigenvalues.size()==dimension)&&
    (a.controlling_direction.size()==0||a.controlling_direction.size()==dimension)&&
    (!a.complete||(a.eigenvalues.size()==dimension&&a.controlling_direction.size()==dimension&&a.state_count==a.ordinary_steps+1)),
    "Invalid Gram dimensions/counts");
  io::Document d; d.SetObject(); Status(d,a.complete,a.within_gain_budget,a.diagnostic);
  io::Integer(d,"ordinary_steps",a.ordinary_steps); io::Integer(d,"state_count",a.state_count);
  io::Integer(d,"controlling_coordinate",a.controlling_coordinate);
  Scalar(d,"gram_norm",a.gram_norm,!a.complete); Scalar(d,"antisymmetry",a.antisymmetry,!a.complete);
  Scalar(d,"minimum_eigenvalue",a.minimum_eigenvalue,!a.complete); Scalar(d,"maximum_eigenvalue",a.maximum_eigenvalue,!a.complete);
  Scalar(d,"mean_gain",a.mean_gain,!a.complete); Scalar(d,"individual_power_bound",a.individual_power_bound,!a.complete);
  d.AddMember("eigenvalues",raw_detail::Vector(d,a.eigenvalues,!a.complete),d.GetAllocator());
  d.AddMember("controlling_direction",raw_detail::Vector(d,a.controlling_direction,!a.complete),d.GetAllocator()); return d;
}
io::Document Sequence(const WallSequenceAnalysis& a,unsigned dimension) {
  io::Require(!a.complete||(a.raw.complete&&a.weighted.complete),"Completed sequence has incomplete Gram evidence");
  io::Require(!a.passed||a.weighted.within_gain_budget,"Passed sequence lacks weighted gain evidence");
  io::Document d; d.SetObject(); Status(d,a.complete,a.passed,a.diagnostic);
  raw_detail::Field(d,"raw",Gram(a.raw,dimension)); raw_detail::Field(d,"weighted",Gram(a.weighted,dimension)); return d;
}
io::Document Difference(const WallDifference& a) {
  io::Document d; d.SetObject(); Status(d,a.complete,a.passed,"");
  Scalar(d,"difference_upper",a.difference,!a.complete); Scalar(d,"budget_lower",a.budget,!a.complete); return d;
}
io::Document Comparison(const WallAnalysisComparison& a) {
  for(const auto& x:a.matrices) {
    io::Require(!a.complete||x.complete,"Completed comparison lacks matrix evidence");
    io::Require(!a.passed||x.passed,"Passed comparison lacks matrix evidence");
  }
  for(unsigned i=0;i<11;++i) {
    io::Require(!a.complete||(a.raw_gains[i].complete&&a.weighted_gains[i].complete),"Completed comparison lacks gain measurements");
    io::Require(!a.passed||a.weighted_gains[i].passed,"Passed comparison lacks weighted gain evidence");
  }
  io::Document d; d.SetObject(); Status(d,a.complete,a.passed,a.diagnostic);
  io::Value matrices(rapidjson::kArrayType),raw(rapidjson::kArrayType),weighted(rapidjson::kArrayType);
  for(const auto& x:a.matrices) raw_detail::Append(d,matrices,Difference(x));
  for(const auto& x:a.raw_gains) raw_detail::Append(d,raw,Difference(x));
  for(const auto& x:a.weighted_gains) raw_detail::Append(d,weighted,Difference(x));
  d.AddMember("matrices",matrices,d.GetAllocator()); d.AddMember("raw_gains_diagnostic",raw,d.GetAllocator());
  d.AddMember("weighted_gains_gate",weighted,d.GetAllocator()); return d;
}
} // namespace tl::qualification::qeph::wall_recurrence::job_json
