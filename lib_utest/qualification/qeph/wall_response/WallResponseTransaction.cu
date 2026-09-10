#include "WallResponseInputs.h"

namespace tl::qualification::qeph::wall_response::runtime_detail {
bool PrepareStep(Run& run,Participants& p,StepStage& output,std::string& error,bool reverse_order) {
  if(!p.initialized||run.completed||run.accepted_steps>=Steps(run.config)||run.attempted_steps>=MaxSteps) {
    error="No available response interval"; return false;
  }
  auto& r=p.wall.shell; StepStage next;
  ++run.attempted_steps;
  if(!c::Read(r.owner,next.base)) { error="Accepted owner readback failed"; return false; }
  c::OwnerAgreement(r,next.base,p.native.state);
  EXPECT_EQ(next.base.stamp.epoch,p.native.epoch); EXPECT_EQ(next.base.stamp.time,p.native.time);
  EXPECT_EQ(next.base.stamp.epoch,run.accepted_steps);
  c::PortResults saved{}; port::BatchDiagnostics accepted;
  if(!c::ReadAccepted(r,saved,accepted)) { error="Accepted shell readback failed"; return false; }
  EXPECT_EQ(c::Bytes(saved),c::Bytes(p.cache));
  // Independent native accepted-base contact precedes the native kick/force.
  const auto native_contact=p.wall.Host(p.native.state,p.native.epoch,1);
  if(!native_contact.valid||!p.native.Propose(r,c::ContactLoads(native_contact),next.native)) {
    error="Native accepted-base proposal failed"; return false;
  }
  run.native_cell_intervals+=r.count; // Completed native proposals, including a later rejected attempt.
  if(!c::Begin(p.wall,next.trial)||!c::Evaluate(p.wall,next.trial,next.components,(p.native.epoch%2)!=reverse_order)) {
    error="Owner preparation or participant evaluation failed"; return false;
  }
  c::Identity(p.wall,next.trial,next.components);
  c::Agreement(p.wall,next.trial,next.components,next.native,p.native.time,p.native.epoch);
  const auto host=p.wall.Host(next.base,next.base.stamp.epoch,next.trial.nodal.view.attempt);
  c::ContactAgreement(next.trial.contact_base_result,host);
  next.source=p.source; next.contact=p.contact;
  next.regular_maximum=p.regular_maximum; next.hourglass_maximum=p.hourglass_maximum;
  c::CheckLedgers(r,next.base,next.trial.nodal,c::ContactLoads(host),p.cache,next.components.shell,p.scales,next.source);
  c::CheckSourceWork(r,p.cache,next.components.elements,next.components.shell,p.scales.energy,next.source);
  c::ContactLedgers(p.wall,next.base,next.trial,next.components,p.scales,next.contact);
  c::CheckScreenedRigid(p.wall,next.trial,next.components,p.scales,next.regular_maximum,next.hourglass_maximum);
  EXPECT_EQ(next.components.contact.node_count,r.n); EXPECT_LE(next.components.contact.node_count,MaxNodes);
  if(::testing::Test::HasFailure()) { error="Native/contact/rigid/ledger check failed"; return false; }
  const auto& d=next.components.contact; const auto& v=next.trial.nodal.view;
  auto& input=next.endpoint;
  input.epoch=next.base.stamp.epoch+1; input.time=v.proposed_time;
  input.carried_velocity_time=v.velocity_time; input.kick_dt=v.kick_dt;
  input.state=StateOf(next.trial.nodal.state); input.elements=ResultsOf(next.components.elements);
  input.contact=ContactOf(next.components.contact_result);
  if(!AddImpulse(p.impulse,p.impulse_error,d.wall_kick_impulse,d.wall_kick_impulse_error,
                 input.wall_impulse,input.wall_impulse_error)) {
    error="Cumulative wall impulse enclosure failed"; return false;
  }
  next.physical_ready=true; output=next; error.clear(); return true;
}
bool ObserveStep(const Run& run,StepStage& stage,std::string& error) {
  stage.observation_ready=false;
  if(!stage.physical_ready||run.accepted_steps!=stage.base.stamp.epoch||
     run.summary.last_epoch!=run.accepted_steps||stage.endpoint.epoch!=run.accepted_steps+1) {
    error="Response observation does not follow the accepted base"; return false;
  }
  Sample sample; Summary summary;
  if(!ObserveEndpoint(run.model,run.config,stage.endpoint,sample,error)||
     !StageSummary(run.model,run.config,run.summary,sample,summary,error)) return false;
  const bool save=sample.epoch%SampleStride(run.config)==0;
  if(save&&(run.samples.size()>=SampleCount||run.samples.size()>=run.samples.capacity())) {
    error="Reserved response sample capacity exhausted"; return false;
  }
  stage.sample=sample; stage.summary=summary; stage.save_sample=save;
  stage.observation_ready=true; error.clear(); return true;
}
bool PublishStep(Run& run,Participants& p,const StepStage& stage,std::string& error) {
  if(!stage.physical_ready||!stage.observation_ready||run.completed||
     run.accepted_steps!=stage.base.stamp.epoch||p.native.epoch!=run.accepted_steps||
     stage.sample.epoch!=run.accepted_steps+1||stage.summary.last_epoch!=stage.sample.epoch||
     (stage.save_sample&&(run.samples.size()>=SampleCount||run.samples.size()>=run.samples.capacity()))) {
    error="Response stage is unavailable for publication"; return false;
  }
  if(::testing::Test::HasFailure()) { error="Scientific assertion prohibits response receipt"; return false; }
  // Prepare all report values before the existing single owner/history commit.
  const auto maxima=Ledgers(stage);
  if(!c::Publish(p.wall,stage.trial,stage.components,p.published)) {
    error="Joint response publication rejected"; return false;
  }
  p.native.Accept(p.wall.shell,stage.native); p.cache=stage.components.elements;
  p.source=stage.source; p.contact=stage.contact;
  p.regular_maximum=stage.regular_maximum; p.hourglass_maximum=stage.hourglass_maximum;
  p.impulse=stage.endpoint.wall_impulse; p.impulse_error=stage.endpoint.wall_impulse_error;
  run.last_accepted=stage.sample; run.summary=stage.summary; run.accepted_steps=stage.sample.epoch;
  run.ledger_maxima=maxima;
  if(stage.save_sample) run.samples.push_back(stage.sample); // Already reserved, trivial fixed-value copy.
  error.clear(); return true;
}
} // namespace tl::qualification::qeph::wall_response::runtime_detail
