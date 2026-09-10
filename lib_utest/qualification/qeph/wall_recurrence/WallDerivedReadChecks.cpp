#include "WallDerivedReadFields.h"

namespace tl::qualification::qeph::wall_recurrence::derived_read {
void Contact(const io::Document& d,unsigned s,unsigned b,State& state) {
  io::Require(!state.contact_seen[s][b]&&!state.step_seen[s],"Repeated or late derived contact");
  for(bool seen:state.amplitude_seen[s]) io::Require(seen,"Contact recheck precedes amplitude files");
  if(b) io::Require(state.contact_seen[s][0],"Skipped derived contact callback");
  RawLink(rd::Field(d,"raw_contact_branch"),state.raw,"contact-h"+std::to_string(s)+"-b"+std::to_string(b)+".json");
  RawLink(rd::Field(d,"raw_native_matrix"),state.raw,"native-h"+std::to_string(s)+"-a2.json");
  const auto& raw=state.raw.job; const auto& source=raw.steps[s]; auto& step=state.analysis.steps[s];
  WallContactRecheck result; result.branch=b?ContactBranch::Active:ContactBranch::Inactive;
  if(source.contact_attempted[b]&&step.amplitudes[2].input_complete)
    result=RecheckWallContact(raw.model,source.h,raw.normal_velocity,result.branch,source.native[2].derivative.full,source.contact[b]);
  else result.diagnostic="Contact samples or their finest native matrix are missing";
  Same(rd::Field(d,"analysis"),job_json::Contact(result,state.analysis.dimension,raw.model.native().nodes),
       "Changed contact samples, derivative measurements or verdict");
  step.contact[b]=std::move(result); state.contact_seen[s][b]=true;
  state.analysis.completed_contacts+=step.contact[b].complete;
}
void Step(const io::Document& d,unsigned s,State& state) {
  io::Require(!state.step_seen[s],"Repeated derived step");
  for(bool seen:state.amplitude_seen[s]) io::Require(seen,"Step precedes amplitude records");
  for(bool seen:state.contact_seen[s]) io::Require(seen,"Step precedes contact records");
  auto links=[&](const char* key,unsigned count,const char* prefix) {
    const auto& list=rd::Field(d,key); io::Require(list.IsArray()&&list.Size()==count,"Changed derived operand count");
    for(unsigned i=0;i<count;++i) {
      const auto name=std::string(prefix)+std::to_string(s)+(count==3?"-a":"-b")+std::to_string(i)+".json";
      rd::Keys(list[i],{"file","sha256"});
      io::Require(rd::Text(rd::Field(list[i],"file"))==name&&rd::Text(rd::Field(list[i],"sha256"))==state.hashes.at(name),
                  "Changed derived local-comparison operand hash");
    }
  };
  links("derived_amplitudes",3,"amplitude-h"); links("derived_contact",2,"contact-h");
  auto& step=state.analysis.steps[s]; const auto& source=state.raw.job.steps[s];
  bool complete=true,passed=true;
  for(const auto& a:step.amplitudes) { complete=complete&&a.complete; passed=passed&&a.passed; }
  for(const auto& c:step.contact) { complete=complete&&c.complete; passed=passed&&c.passed; }
  for(unsigned p=0;p<2;++p) {
    if(step.amplitudes[p].input_complete&&step.amplitudes[p+1].input_complete)
      step.native_amplitude_comparisons[p]=CompareWallMatrices(source.native[p].derivative.full,source.native[p+1].derivative.full);
    step.derived_amplitude_comparisons[p]=CompareWallAnalyses(step.amplitudes[p].branches,step.amplitudes[p+1].branches);
    complete=complete&&step.native_amplitude_comparisons[p].complete&&step.derived_amplitude_comparisons[p].complete;
    passed=passed&&step.native_amplitude_comparisons[p].passed&&step.derived_amplitude_comparisons[p].passed;
  }
  step.complete=complete; step.passed=complete&&passed;
  step.diagnostic=Diagnostic(rd::Field(d,"analysis"));
  Same(rd::Field(d,"analysis"),job_json::Step(step),"Changed local amplitude/gain comparisons or step verdict");
  state.step_seen[s]=true; ++state.next_step; state.analysis.completed_steps+=step.complete;
}
void ReleaseDense(WallStepAnalysis& step) {
  for(auto& a:step.amplitudes) {
    a.baseline.expected.resize(0); a.baseline.residual.resize(0); a.branches.schedule.scalar.clear();
    auto release=[](WallSequenceAnalysis& x) {
      x.raw.eigenvalues.resize(0); x.raw.controlling_direction.resize(0);
      x.weighted.eigenvalues.resize(0); x.weighted.controlling_direction.resize(0);
    };
    for(auto& b:a.branches.branches) {
      b.full.resize(0,0); b.weighted.resize(0,0); b.spectrum.eigenvalues.clear(); b.weighted_spectrum.eigenvalues.clear();
      release(b.continuous);
    }
    for(auto& e:a.branches.events) release(e.sequence);
  }
  for(auto& c:step.contact) { c.directions.clear(); c.baseline.expected.resize(0); c.baseline.residual.resize(0); }
}
} // namespace tl::qualification::qeph::wall_recurrence::derived_read
