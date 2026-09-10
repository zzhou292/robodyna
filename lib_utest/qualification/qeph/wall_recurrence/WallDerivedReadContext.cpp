#include "WallDerivedReadFields.h"
#include <set>
#include <stdexcept>

namespace tl::qualification::qeph::wall_recurrence::derived_read {
std::string RawHash(const RawReadResult& raw,const std::string& name) {
  for(const auto& f:raw.receipt.files) if(f.name==name) return f.sha256;
  return {};
}
void RawLink(const io::Value& v,const RawReadResult& raw,const std::string& name) {
  const auto hash=RawHash(raw,name); io::Document expected; expected.SetObject();
  io::Boolean(expected,"present",!hash.empty()); io::String(expected,"file",name);
  if(!hash.empty()) io::String(expected,"sha256",hash);
  Same(v,expected,"Derived payload names a foreign or missing raw operand");
}
void Header(const io::Document& d,const char* kind,const State& state) {
  std::set<std::string> allowed;
  auto text=[&](const char* name,const std::string& expected) {
    allowed.insert(name); io::Require(rd::Text(rd::Field(d,name))==expected,(std::string("Derived header mismatch: ")+name).c_str());
  };
  auto integer=[&](const char* name,std::uint64_t expected) {
    allowed.insert(name); io::Require(rd::Unsigned(rd::Field(d,name))==expected,(std::string("Derived header count mismatch: ")+name).c_str());
  };
  auto number=[&](const char* name,double expected) {
    allowed.insert(name); io::Require(io::Bits(rd::Number(rd::Field(d,name)))==io::Bits(expected),
      (std::string("Derived header scalar mismatch: ")+name).c_str());
  };
  text("schema","robo-dyna-qeph-wall-derived-v1"); text("kind",kind); integer("cells",state.raw.job.cells);
  number("normal_velocity_m_s",state.raw.job.normal_velocity);
  integer("normal_velocity_binary64_bits",io::Bits(state.raw.job.normal_velocity));
  text("raw_index_sha256",state.receipt.raw_index_sha256); text("raw_provenance_sha256",state.receipt.raw_provenance_sha256);
  allowed.insert("raw_model"); RawLink(rd::Field(d,"raw_model"),state.raw,"model.json");
  text("analysis_provenance_file","provenance.json"); text("analysis_provenance_sha256",state.binding.analysis_provenance_sha256);
  integer("analysis_provenance_bytes",state.receipt.files.front().bytes); text("analysis_provenance_origin",state.origin);
  text("authentication_scope","Externally authenticated raw reader result and exact analysis provenance; listed source/build trust remains external");
  text("qualification_scope","One prescribed native/contact recurrence analysis; no startup or trajectory admission");
  for(const char* name:{"simulation_ready","six_job_screen_decision_included"}) {
    allowed.insert(name); io::Require(!rd::Boolean(rd::Field(d,name)),"Derived job claims unauthorized admission");
  }
  number("matrix_tolerance",recurrence::MatrixTolerance); number("decomposition_tolerance",recurrence::DecompositionTolerance);
  number("weighted_mean_gain_limit",recurrence::MaximumGramGain);
  number("gain_relative_tolerance",WallGainRelativeTolerance); number("gain_absolute_tolerance",WallGainAbsoluteTolerance);
  const std::string type=kind;
  if(type=="context") for(const char* name:{"step_index","context"}) allowed.insert(name);
  else if(type=="amplitude") for(const char* name:{"step_index","fixed_dt_s","amplitude_index","raw_native_matrix","analysis",
      "context_available","context_file","context_sha256"}) allowed.insert(name);
  else if(type=="contact-recheck") for(const char* name:{"step_index","fixed_dt_s","branch_index","raw_native_matrix","raw_contact_branch","analysis"}) allowed.insert(name);
  else if(type=="step") for(const char* name:{"step_index","fixed_dt_s","derived_amplitudes","derived_contact","analysis"}) allowed.insert(name);
  else if(type=="progress-index"||type=="final-index") for(const char* name:{"final_index","analysis_complete","analysis_passed",
      "bytes_before_this_index","remaining_set_budget_at_job_start","file_byte_cap","shared_raw_and_derived_byte_cap",
      "inventory_scope","selection_scope","steps","completed_amplitudes","completed_contacts","completed_steps","input_valid","diagnostic","files"}) allowed.insert(name);
  else io::Require(false,"Unknown derived payload kind");
  for(auto it=d.MemberBegin();it!=d.MemberEnd();++it)
    io::Require(allowed.count(std::string(it->name.GetString(),it->name.GetStringLength()))==1,"Unexpected derived header field");
}
void Context(const io::Document& d,unsigned s,State& state) {
  io::Require(!state.context_seen[s]&&!state.step_seen[s],"Duplicate or late derived context");
  WallBranchAnalysis expected; expected.h=recurrence::Steps[s]; std::string error;
  if(!BuildWallStateMetric(state.raw.job.model,expected.metric,error)) throw std::runtime_error(error);
  expected.schedule=AnalyzeWallSwitchingSchedule(state.raw.job.model,expected.h);
  const auto& saved=rd::Field(d,"context");
  if(rd::Same(saved,job_json::Context(expected,state.analysis.dimension))) {
    state.context_usable[s]=expected.schedule.passed;
    state.contexts[s]=std::move(expected);
  } else {
    // These are the two staged early-return forms before a scalar schedule is
    // available. Their absence is preserved, never filled in as observation.
    WallBranchAnalysis partial; partial.h=recurrence::Steps[s];
    const auto& schedule=rd::Field(saved,"schedule"); partial.schedule.diagnostic=Diagnostic(schedule);
    if(!rd::Same(saved,job_json::Context(partial,state.analysis.dimension))) {
      partial.metric=expected.metric;
      Same(saved,job_json::Context(partial,state.analysis.dimension),"Changed fixed metric/scalar context");
    }
    state.contexts[s]=std::move(partial);
  }
  state.context_seen[s]=true;
}
} // namespace tl::qualification::qeph::wall_recurrence::derived_read
