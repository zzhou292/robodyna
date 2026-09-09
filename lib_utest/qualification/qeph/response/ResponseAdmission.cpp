#include "ResponseProtocol.h"
#include "../free_response/RecurrenceAudit.h"
#include <algorithm>

namespace tl::qualification::qeph::response {
namespace a=tl::qualification::qeph::recurrence;
namespace p=protocol;
namespace {
void Header(const io::Value& d,bool raw) {
  io::Require(p::String(d,"schema")=="robo-dyna-qeph-recurrence-audit-v1"&&
    p::String(d,"phase")== (raw?"raw-probes":"decision"),"Wrong matrix report phase/schema");
  io::Require(!p::Boolean(d,"simulation_ready")&&!p::Boolean(d,"trajectory_execution_qualified"),"Matrix report overclaims trajectory qualification");
  io::Require(p::Number(d,"horizon_s")==Horizon&&p::Number(d,"matrix_tolerance")==a::MatrixTolerance&&
    p::Number(d,"decomposition_tolerance")==a::DecompositionTolerance&&p::Number(d,"maximum_gram_mean_gain")==a::MaximumGramGain,
    "Changed matrix audit scope/threshold");
  double steps[6],amplitudes[3]; p::ReadArray(d,"fixed_dt_grid_s",steps,6); p::ReadArray(d,"fixed_amplitudes",amplitudes,3);
  for(unsigned i=0;i<6;++i) io::Require(steps[i]==a::Steps[i],"Changed frozen matrix grid");
  for(unsigned i=0;i<3;++i) io::Require(amplitudes[i]==a::Amplitudes[i],"Changed frozen perturbation amplitude");
}
void Dictionary(const io::Value& d,const a::Model& m) {
  io::Require(p::Integer(d,"elements")==m.elements&&p::Integer(d,"nodes")==m.nodes&&p::Boolean(d,"complete"),"Incomplete matrix fixture");
  const auto& list=p::Array(d,"dictionary",m.dictionary.size());
  const char* groups[]{"position","orientation_tangent","velocity","spin","history","force_cache"};
  for(unsigned i=0;i<m.dictionary.size();++i) {
    const auto& actual=list[i]; const auto& expected=m.dictionary[i];
    io::Require(p::String(actual,"group")==groups[static_cast<unsigned>(expected.group)]&&p::String(actual,"name")==expected.name&&
      p::String(actual,"unit")==expected.unit&&p::Integer(actual,"entity")==expected.entity&&p::Integer(actual,"component")==expected.component&&
      p::Number(actual,"scale")==expected.scale&&p::Boolean(actual,"feedback")==expected.feedback,"Changed frozen recurrence dictionary/model");
  }
}
void Analysis(const io::Value& v,unsigned dimension) {
  io::Require(p::Boolean(v,"complete")&&p::Boolean(v,"passed"),"Unresolved required matrix analysis");
  const char* finite_errors[]{"schur_residual","orthogonality_error","gram_antisymmetry","rigid_projection_residual"};
  for(auto key:finite_errors) { const double x=p::Number(v,key); io::Require(x>=0&&x<=a::DecompositionTolerance,"Matrix decomposition/Gram residual rejected"); }
  for(auto key:{"zero_feedback_error","observer_error","rigid_error"}) {
    const double x=p::Number(v,key); io::Require(x>=0&&x<=a::MatrixTolerance,"Matrix structural identity rejected");
  }
  const double radius=p::Number(v,"spectral_radius"),gain=p::Number(v,"gram_mean_gain"),condition=p::Number(v,"rigid_basis_condition");
  io::Require(radius>=0&&radius<=1+5e-8&&gain>=0&&gain<=64&&condition>=1&&condition<=1e8,"Matrix amplification/conditioning rejected");
  const auto& eigen=p::Array(v,"eigenvalues_real_imag",dimension);
  double actual_radius=0;
  for(const auto& z:eigen.GetArray()) {
    io::Require(z.IsArray()&&z.Size()==2&&z[0].IsNumber()&&z[1].IsNumber(),"Invalid retained spectrum");
    const double magnitude=std::hypot(z[0].GetDouble(),z[1].GetDouble()); io::Require(std::isfinite(magnitude),"Nonfinite retained spectrum");
    actual_radius=std::max(actual_radius,magnitude);
  }
  io::Require(std::abs(actual_radius-radius)<=8*std::numeric_limits<double>::epsilon()*std::max(1.,radius),"Spectrum/radius mismatch");
}
}
Binding CheckAdmission(const std::string& decision_bytes,const std::string& raw_bytes,const io::Document& provenance) {
  const auto decision_hash=io::Sha256(decision_bytes);
  io::Require(p::String(provenance,"matrix_decision_sha256")==decision_hash,"Decision differs from root-supplied fingerprint");
  const auto decision=p::Parse(decision_bytes),raw=p::Parse(raw_bytes); Header(decision,false); Header(raw,true);
  io::Require(p::Boolean(decision,"audit_passed")&&p::Number(decision,"selected_candidate_dt_s")==H0,"This fixed runtime requires retained H0 selection");
  io::Require(!p::Boolean(raw,"audit_passed")&&p::Number(raw,"selected_candidate_dt_s")==0,"Raw matrices must precede decisions");
  const auto raw_hash=io::Sha256(raw_bytes);
  io::Require(p::String(decision,"raw_report_sha256")==raw_hash&&p::Integer(decision,"raw_report_bytes")==raw_bytes.size()&&
      p::String(decision,"raw_report_name")=="raw-matrices.json","Matrix raw byte/hash binding mismatch");
  const auto& cases=p::Array(decision,"cases",2); const auto& raw_cases=p::Array(raw,"cases",2);
  for(unsigned c=0;c<2;++c) {
    a::Model model; std::string error; io::Require(a::BuildModel(c+1,model,error),"Cannot construct frozen audit reference");
    Dictionary(cases[c],model); Dictionary(raw_cases[c],model);
    const auto& steps=p::Array(cases[c],"steps",6); const auto& raw_steps=p::Array(raw_cases[c],"steps",6);
    const auto dimension=model.dictionary.size();
    for(unsigned s=0;s<6;++s) {
      io::Require(p::Number(steps[s],"fixed_dt_s")==a::Steps[s]&&p::Number(raw_steps[s],"fixed_dt_s")==a::Steps[s],"Step identity mismatch");
      const auto& probes=p::Array(steps[s],"probes",3); const auto& raw_probes=p::Array(raw_steps[s],"probes",3);
      const auto& analyses=p::Array(steps[s],"analyses",3);
      for(unsigned k=0;k<3;++k) {
        for(const auto* probe:{&probes[k],&raw_probes[k]}) {
          io::Require(p::Number(*probe,"amplitude")==a::Amplitudes[k]&&p::Integer(*probe,"dimension")==dimension&&
            p::Integer(*probe,"completed_columns")<=dimension,"Raw probe identity mismatch");
          if(s<=4) io::Require(p::Boolean(*probe,"complete")&&p::Integer(*probe,"completed_columns")==dimension,"Required complete raw probe mismatch");
        }
        const auto& matrix=p::Member(raw_probes[k],"full_matrix_rows");
        io::Require(matrix.IsArray()&&(matrix.Size()==dimension||(s>4&&matrix.Empty())),"Raw matrix row count mismatch");
        for(const auto& row:matrix.GetArray()) {
          io::Require(row.IsArray()&&row.Size()==dimension,"Raw matrix dimensions mismatch");
          for(const auto& value:row.GetArray()) io::Require(value.IsNumber()&&std::isfinite(value.GetDouble()),"Nonfinite raw matrix");
        }
        if(s<=4) Analysis(analyses[k],6*model.nodes+44*model.elements);
      }
      if(s>4) continue; // 4H0 is retained evidence, not needed for selected H0.
      io::Require(p::Boolean(steps[s],"passed"),"Missing factor-two sampled margin");
      const auto& differences=p::Array(steps[s],"matrix_difference_max_abs",2);
      const auto& gram_differences=p::Array(steps[s],"gram_gain_relative_difference",2);
      for(unsigned k=0;k<2;++k) {
        const auto& coarse=raw_probes[k]["full_matrix_rows"]; const auto& fine=raw_probes[k+1]["full_matrix_rows"];
        double difference=0,magnitude=0;
        for(unsigned row=0;row<dimension;++row) for(unsigned column=0;column<dimension;++column) {
          const double f=fine[row][column].GetDouble(); magnitude=std::max(magnitude,std::abs(f));
          difference=std::max(difference,std::abs(coarse[row][column].GetDouble()-f));
        }
        io::Require(differences[k].IsNumber()&&differences[k].GetDouble()==difference&&difference<=a::MatrixTolerance*std::max(1.,magnitude),
          "Raw matrix finite-difference refinement rejected");
        const double g0=p::Number(analyses[k],"gram_mean_gain"),g1=p::Number(analyses[k+1],"gram_mean_gain");
        const double relative=std::abs(g0-g1)/std::max({std::abs(g0),std::abs(g1),1e-10});
        io::Require(gram_differences[k].IsNumber()&&gram_differences[k].GetDouble()==relative&&
          std::abs(g0-g1)<=.005*std::max(std::abs(g0),std::abs(g1))+1e-10,"Gram amplitude refinement rejected");
      }
    }
  }
  return {decision_hash,raw_hash,{}};
}
} // namespace tl::qualification::qeph::response
