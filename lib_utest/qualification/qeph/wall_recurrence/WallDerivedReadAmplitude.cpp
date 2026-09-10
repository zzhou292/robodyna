#include "WallDerivedReadFields.h"
#include <stdexcept>

namespace tl::qualification::qeph::wall_recurrence::derived_read {
void Amplitude(const io::Document& d,unsigned s,unsigned a,State& state) {
  io::Require(!state.amplitude_seen[s][a]&&!state.step_seen[s],"Repeated or late derived amplitude");
  for(unsigned i=0;i<a;++i) io::Require(state.amplitude_seen[s][i],"Skipped derived amplitude callback");
  const auto& raw=state.raw.job; const auto& source=raw.steps[s].native[a];
  const unsigned dimension=state.analysis.dimension;
  RawLink(rd::Field(d,"raw_native_matrix"),state.raw,"native-h"+std::to_string(s)+"-a"+std::to_string(a)+".json");
  const bool context=rd::Boolean(rd::Field(d,"context_available"));
  io::Require(context==state.context_seen[s],"Amplitude context publication association differs");
  if(context) {
    const auto name="context-h"+std::to_string(s)+".json";
    io::Require(rd::Text(rd::Field(d,"context_file"))==name&&rd::Text(rd::Field(d,"context_sha256"))==state.hashes.at(name),
                "Amplitude context file/hash differs");
  } else io::Require(!d.HasMember("context_file")&&!d.HasMember("context_sha256"),"Absent context has a link");
  const auto& v=rd::Field(d,"analysis"); WallAmplitudeAnalysis result;
  result.amplitude=recurrence::Amplitudes[a]; result.attempted=raw.steps[s].native_attempted[a];
  result.input_complete=result.attempted&&source.baseline_complete&&source.derivative.complete;
  result.diagnostic=Diagnostic(v);
  if(result.input_complete) result.baseline=CheckWallMovingBaseline(raw.model,raw.steps[s].h,source);
  Same(rd::Field(v,"moving_baseline"),job_json::Baseline(result.baseline,dimension),"Changed moving baseline measurement/verdict");
  auto& branches=result.branches; const auto& saved=rd::Field(v,"branches"); branches.diagnostic=Diagnostic(saved);
  if(result.input_complete) {
    io::Require(context,"Complete native input lacks its captured context");
    branches.h=raw.steps[s].h; branches.metric=state.contexts[s].metric; branches.schedule=state.contexts[s].schedule;
  }
  const auto& constants=rd::Field(saved,"constant_branches"); const auto& events=rd::Field(saved,"events");
  io::Require(constants.IsArray()&&constants.Size()==2&&events.IsArray()&&events.Size()==9,"Changed full branch/event counts");
  bool all_complete=result.input_complete&&state.context_usable[s],all_passed=all_complete;
  for(unsigned b=0;b<2;++b) {
    auto& c=branches.branches[b]; const auto& item=constants[b];
    c.branch=b?ContactBranch::Active:ContactBranch::Inactive;
    const bool full=rd::Boolean(rd::Field(rd::Field(item,"full_operator"),"present"));
    const bool weighted=rd::Boolean(rd::Field(rd::Field(item,"weighted_operator"),"present"));
    io::Require(!weighted||full,"Weighted operator has no full operator");
    io::Require(!full||(result.input_complete&&state.context_usable[s]),"Operator precedes complete native/context operands");
    std::string error;
    if(full&&!BuildContactBranch(raw.model,raw.steps[s].h,c.branch,source.derivative.full,c.full,error)) throw std::runtime_error(error);
    if(weighted&&!ApplyWallStateMetric(c.full,branches.metric.diagonal,c.weighted,error)) throw std::runtime_error(error);
    if(weighted) c.identities=CheckWallStateIdentities(raw.model,raw.steps[s].h,c.full);
    Same(rd::Field(item,"identities"),job_json::Identities(c.identities,dimension),"Changed full-state identity measurements");
    c.spectrum=Spectrum(rd::Field(item,"raw_spectrum"),weighted?c.full:Eigen::MatrixXd{},dimension);
    c.weighted_spectrum=Spectrum(rd::Field(item,"weighted_spectrum"),c.weighted,dimension);
    c.continuous=Sequence(rd::Field(item,"continuous_gram"),dimension,branches.schedule.ordinary_steps);
    io::Require(weighted||(!c.spectrum.complete&&!c.weighted_spectrum.complete&&!c.continuous.complete),
                "Completed numerical evidence precedes operator construction");
    const bool done=c.identities.complete&&c.spectrum.complete&&c.weighted_spectrum.complete&&c.continuous.complete;
    branches.completed_branches+=done; all_complete=all_complete&&done;
    all_passed=all_passed&&c.identities.passed&&c.spectrum.passed&&c.weighted_spectrum.passed&&c.continuous.passed;
  }
  for(unsigned i=0;i<9;++i) {
    const auto& item=events[i]; auto& event=branches.events[i];
    const bool available=rd::Boolean(rd::Field(item,"window_available"));
    io::Require(!available||(result.input_complete&&state.context_usable[s]),"Event lacks its physical schedule");
    if(available) event.window=branches.schedule.windows[i];
    event.sequence=Sequence(rd::Field(item,"gram"),dimension,branches.schedule.ordinary_steps);
    io::Require(!event.sequence.complete||available,"Completed event has no actual window");
    branches.completed_events+=event.sequence.complete; all_complete=all_complete&&event.sequence.complete;
    all_passed=all_passed&&event.sequence.passed;
  }
  branches.complete=all_complete; branches.passed=all_complete&&all_passed;
  result.complete=result.input_complete&&result.baseline.complete&&branches.complete;
  result.passed=result.complete&&result.baseline.passed&&branches.passed;
  // Reusing the owning serializer checks every key, recipe, matrix descriptor
  // hash and child status against the reconstructed/validated values.
  Same(v,job_json::Amplitude(result,raw,s,a),"Changed amplitude fields, reconstruction hashes or verdict");
  state.analysis.steps[s].amplitudes[a]=std::move(result); state.amplitude_seen[s][a]=true;
  state.analysis.completed_amplitudes+=state.analysis.steps[s].amplitudes[a].complete;
}
} // namespace tl::qualification::qeph::wall_recurrence::derived_read
