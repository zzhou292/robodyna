#include "WallResponseInputs.h"

namespace tl::qualification::qeph::wall_response::runtime_detail {
namespace {
c::WallExperiment Experiment(const Model& model) {
  c::WallExperiment e; e.qualification=Qualification; e.shell_configuration=ShellConfiguration;
  e.wall_configuration=WallConfiguration; e.wall_binding=WallBinding; e.law=model.screened().law();
  e.startup={port::BatchStartupKind::ReferenceUniformTranslation,{wr::ImpactSpeed,0,0}};
  return e;
}
bool InitialContact(Participants& p,contact::NodalWallDeviceResults& output) {
  auto& w=p.wall; auto& r=w.shell;
  c::fe::NodalTrialToken token; c::fe::NodalAssemblyView view;
  const auto begin=r.owner.BeginTrial(&token,&view);
  EXPECT_EQ(begin.status,c::fe::NodalStatus::Ok); if(begin.status!=c::fe::NodalStatus::Ok) return false;
  const auto shell=r.batch.AssembleAccepted(r.owner,view);
  EXPECT_EQ(shell.status,port::BatchStatus::Success);
  if(shell.status!=port::BatchStatus::Success) { p.Discard(); return false; }
  contact::NodalWallDiagnostics d;
  const auto assembled=w.wall.AssembleAccepted(view,&d);
  EXPECT_EQ(assembled.status,contact::NodalWallDeviceStatus::Ok);
  if(assembled.status!=contact::NodalWallDeviceStatus::Ok) { p.Discard(); return false; }
  EXPECT_TRUE(d.valid); EXPECT_EQ(d.owner_id,r.owner.accepted().owner_id);
  EXPECT_EQ(d.configuration_id,WallConfiguration); EXPECT_EQ(d.qualification_id,Qualification);
  EXPECT_EQ(d.wall_binding_id,WallBinding); EXPECT_EQ(d.base_epoch,0u); EXPECT_EQ(d.attempt,view.attempt);
  EXPECT_EQ(d.phase,contact::NodalWallDevicePhase::AcceptedBase);
  EXPECT_EQ(d.node_count,r.n); EXPECT_LE(d.node_count,MaxNodes); EXPECT_EQ(d.parent_count,r.count);
  contact::NodalWallDeviceResults result;
  const auto copied=w.wall.CopyResults(d,&result);
  EXPECT_EQ(copied.status,contact::NodalWallDeviceStatus::Ok); p.Discard();
  if(copied.status!=contact::NodalWallDeviceStatus::Ok||::testing::Test::HasFailure()) return false;
  output=result; return true;
}
}
Participants::Participants(const Config& config_in,const Model& model_in)
  :config(config_in),model(model_in),wall(config_in.cells,Experiment(model_in)) {}
bool Initialize(Run& run,Participants& p,std::string& error) {
  if(p.initialized||&run.config!=&p.config||&run.model!=&p.model||!ValidConfig(run.config)||
     !run.model.prepared()||run.model.fields().cells!=run.config.cells||
     !run.samples.empty()||run.samples.capacity()!=SampleCount||run.summary.initialized||
     run.accepted_steps||run.attempted_steps||run.completed) {
    error="Response requires a fresh matching model/config and exactly257 reserved samples"; return false;
  }
  auto& r=p.wall.shell; r.h=Step(run.config);
  if(!c::InitializeScreenedWall(p.wall,run.model.screened(),p.velocity,p.scales)) {
    error="Screened reference/owner/participant initialization failed"; return false;
  }
  const auto& model=run.model.fields();
  for(unsigned n=0;n<r.n;++n) {
    EXPECT_EQ(r.mass[n],model.mass[n]); EXPECT_EQ(r.inertia[n],model.inertia[n]);
    EXPECT_EQ(r.physical[n],model.physical[n]); EXPECT_EQ(r.added[n],model.added[n]);
    for(unsigned a=0;a<3;++a) EXPECT_EQ(r.x[3*n+a],model.initial_position[3*n+a]);
  }
  const long double coefficient=256*std::numeric_limits<double>::epsilon()+1e-12L;
  EXPECT_LE(std::abs(p.scales.energy-run.model.energy()),coefficient*p.scales.energy);
  EXPECT_LE(std::abs(p.scales.linear-run.model.momentum()),coefficient*p.scales.linear);
  const auto owner=r.owner.allocations(); const auto shell=r.batch.allocations(); const auto wall=p.wall.wall.allocations();
  run.owner_id=r.owner.accepted().owner_id;
  run.owner_device_bytes=owner.device_bytes; run.owner_allocations=owner.device_allocations;
  run.batch_device_bytes=shell.device_bytes; run.batch_allocations=shell.device_allocations;
  run.wall_device_bytes=wall.device_bytes; run.wall_allocations=wall.device_allocations;
  if(!c::InitializeScreenedNative(p.wall,p.velocity,p.native)) { error="Native startup failed"; return false; }
  c::Snapshot initial; port::BatchDiagnostics accepted;
  if(!c::Read(r.owner,initial)||!c::ReadAccepted(r,p.cache,accepted)) { error="Initial accepted readback failed"; return false; }
  c::OwnerAgreement(r,initial,p.native.state); c::CheckScreenedInitial(p.wall,p.scales,initial,p.cache,accepted);
  EXPECT_EQ(accepted.configuration_id,ShellConfiguration);
  contact::NodalWallDeviceResults contact;
  if(!InitialContact(p,contact)) { error="Initial finite-wall assembly/readback failed"; return false; }
  c::ContactAgreement(contact,p.wall.Host(initial,0,contact.diagnostics.attempt));
  if(::testing::Test::HasFailure()) { error="Initial model/native/K0/identity check failed"; return false; }
  EndpointInput input; input.state=StateOf(initial); input.elements=ResultsOf(p.cache);
  input.contact=ContactOf(contact);
  Sample sample; Summary summary;
  if(!ObserveEndpoint(run.model,run.config,input,sample,error)||
     !StageSummary(run.model,run.config,run.summary,sample,summary,error)) return false;
  run.last_accepted=sample; run.summary=summary; run.samples.push_back(sample);
  p.published=contact; p.initialized=true; error.clear(); return true;
}
} // namespace tl::qualification::qeph::wall_response::runtime_detail
