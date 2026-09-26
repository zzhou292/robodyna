// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FullLedgerRig.h"
namespace type25_source_test {
void FullLedgerRig::PrepareSolidModel() {
  fe::solids::Input18 a;fe::solids::Input24 b;fe::solids::Input6z c;
  nodal_empty_test::Fixture::Require(fe::solid18::InitializeReference(fixture.source.a,a.reference)==fe::solid18::Status::Success,"Solid18 reference");
  nodal_empty_test::Fixture::Require(fe::solid24::InitializeReference(fixture.source.b,b.reference)==fe::solid24::Status::Success,"Solid24 reference");
  nodal_empty_test::Fixture::Require(fe::solid6z::InitializeReference(fixture.source.c,c.reference)==fe::solid6z::Status::Success,"Solid6z reference");
  const double strain[]{0,.2,.4},stress[]{1e6,2e6,3e6};
  nodal_empty_test::Fixture::Require(tl::material::law36::Prepare(100e6,.3,fixture.source.a.density_kg_m3,{strain,stress,3},a.material)==tl::material::law36::Status::Ok,"Solid18 material");
  nodal_empty_test::Fixture::Require(tl::material::law42::Prepare(24e6,.463,fixture.source.b.density_kg_m3,1e26,b.material)==tl::material::law42::Status::Ok,"Solid24 material");
  c.material=b.material;
  const auto report=solid_model.Initialize(fixture.domain,{1,{&a,1},{&b,1},{&c,1}});
  nodal_empty_test::Fixture::Require(bool(report),report.message);
  nodal_empty_test::Fixture::Require(solid_model.contributions()->Matches(fixture.solids),"Actual solid mechanics and ledger differ");
}
void FullLedgerRig::PrepareOwner() {
  const auto count=fixture.domain.node_count();
  x.resize(3*count);v.resize(3*count);w.resize(3*count);q.resize(4*count);
  m.resize(count);j.resize(count);im.resize(count);ij.resize(count);
  fixed.resize(count);rotation_fixed.resize(count);present.resize(count);
  for(auto node:fixture.primary.front().nodes)fixed[node]=7;
  for(std::size_t i=0;i<count;++i) {
    const auto& value=fixture.ledger.nodes()[i].coefficients;
    m[i]=value.mass;j[i]=value.isotropic_inertia;present[i]=j[i]>0;
    rotation_fixed[i]=fixed[i]==7&&present[i];
    im[i]=fixed[i]==7?0:1/m[i];ij[i]=rotation_fixed[i]||!present[i]?0:1/j[i];
    const auto position=fixture.domain.nodes()[i].position;
    x[3*i]=position.x;x[3*i+1]=position.y;x[3*i+2]=position.z;q[4*i]=1;
    fixture.nodes[i].constraint=fixed[i];
  }
  const auto report=tl::constraints::tied_shell::PrepareEmptyCinAttachments(fixture.domain,&cin);
  nodal_empty_test::Fixture::Require(bool(report),"Explicit empty CIN attachment source");
  fe::NodalStateConfig config;config.node_count=count;config.fixed_dt=Dt;
  config.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  const fe::NodalCinStartup raw{&cin,m.data(),j.data(),nullptr,nullptr,0,Qualification};
  Check(owner.Initialize(config,{x.data(),v.data(),w.data(),count,q.data()},im.data(),
      {fixed.data(),rotation_fixed.data(),ij.data(),present.data()},fixture.rigid,&raw));
}
void FullLedgerRig::Initialize(bool attach_contact) {
  PrepareSolidModel();PrepareOwner();
  fe::qeph::QephBatchConfig qconfig;qconfig.startup=startup;qconfig.owner=owner.accepted();
  qconfig.configuration_id=Configuration;qconfig.qualification_id=Qualification;
  qconfig.element_count=fixture.shells.qeph_count();qconfig.usage=fe::qeph::BatchUsage::CoupledForces;
  Check(qeph.InitializeMapped(qconfig,fixture.physical,owner,Witnesses()));
  fe::t3::T3BatchConfig tconfig;tconfig.startup=startup;tconfig.owner=owner.accepted();
  tconfig.configuration_id=Configuration;tconfig.qualification_id=Qualification;
  tconfig.element_count=fixture.shells.t3_count();tconfig.usage=fe::t3::BatchUsage::CoupledForces;
  Check(t3.InitializeMapped(tconfig,fixture.physical,owner,Witnesses()));
  fe::type25::BatchConfig wc;wc.startup=startup;wc.owner=owner.accepted();
  wc.configuration_id=Configuration;wc.qualification_id=Qualification;wc.element_count=fixture.welds.connection_count();
  Check(welds.InitializeMapped(wc,fixture.physical,owner,Witnesses(),fe::type25::CapacityProfile::Legacy));
  fe::type13::BatchConfig bc;bc.startup=startup;bc.owner=owner.accepted();
  bc.configuration_id=Configuration;bc.qualification_id=Qualification;bc.assembly=fe::type13::BatchAssembly::CinNativeStiffness;
  Check(beams.InitializeMapped(bc,fixture.physical,fixture.rigid,owner,Witnesses()));
  fe::solids::BatchConfig sc;sc.startup=startup;sc.owner=owner.accepted();
  sc.configuration_id=Configuration;sc.qualification_id=Qualification;sc.profile=fe::solids::BatchProfile::PhysicalCinV1;
  Check(solids.InitializeJoined(sc,solid_model));
  // Existing mapped producers establish their real initial caches before the
  // common publisher joins them. This proof is discarded without a time step.
  FullLedgerAttempt proof;Check(owner.BeginTrial(&proof.token,&proof.assembly));
  Check(qeph.AssembleMappedAccepted(owner,proof.token,proof.assembly));
  Check(t3.AssembleMappedAccepted(owner,proof.token,proof.assembly));
  Check(welds.AssembleMappedAccepted(owner,proof.token,proof.assembly));
  Check(beams.AssembleMappedAccepted(owner,proof.token,proof.assembly));
  Discard();
  Check(publication.InitializePhysical(owner,fixture.physical,fixture.rigid,Witnesses(),Participants(),Identity()));
  if(attach_contact) {
    Check(contact.Initialize(fixture.Config(),Source(),owner,publication,fixture.physical,Participants(),Identity()));
    Check(publication.ConfigurePhysicalScratchParticipation(owner,fixture.physical,Participants(),Identity(),{{},contact.roster_entry()}));
  }
}
void FullLedgerRig::Begin(FullLedgerAttempt& a) {
  Check(owner.BeginTrial(&a.token,&a.assembly));
  Check(qeph.AssembleMappedAccepted(owner,a.token,a.assembly));
  Check(t3.AssembleMappedAccepted(owner,a.token,a.assembly));
  Check(welds.AssembleMappedAccepted(owner,a.token,a.assembly));
  Check(beams.AssembleMappedAccepted(owner,a.token,a.assembly));
  Check(solids.AssembleAccepted(owner,a.token,a.assembly));
}
void FullLedgerRig::Prepare(FullLedgerAttempt& a) {
  Check(owner.SealAssembly(a.token));
  Check(fe::AdvanceStaggeredCin(owner,a.token,{a.assembly.owner_id,a.assembly.accepted.base_epoch,a.assembly.attempt,
    Qualification,Dt,.2,true,{fe::NodalCinStructuralProfile::NativeOrdinaryRigidTrace,.8,true}}));
  Check(owner.BorrowPrepared(a.token,&a.prepared));
  Check(qeph.EvaluateCandidate(owner,a.token,a.prepared,&a.candidates.qeph));
  Check(t3.EvaluateCandidate(owner,a.token,a.prepared,&a.candidates.t3));
  Check(welds.EvaluateCandidate(owner,a.token,a.prepared,&a.candidates.type25));
  Check(beams.EvaluateCandidate(owner,a.token,a.prepared,&a.candidates.type13));
  Check(solids.EvaluateCandidate(owner,a.token,a.prepared,&a.candidates.solids));
  Check(publication.PreparePhysical(owner,a.token,{&a.candidates.qeph,&a.candidates.t3,nullptr,
    &a.candidates.type25,&a.candidates.type13,&a.candidates.solids},&a.common));
}
void FullLedgerRig::Seal(FullLedgerAttempt& a) {
  Check(contact.SealCandidate(owner,a.token,a.prepared,a.common,&a.contact));
  Check(publication.SealPhysicalScratchParticipation(owner,a.token,{nullptr,&a.contact}));
}
fe::ShellPublicationReport FullLedgerRig::Commit(FullLedgerAttempt& a,bool accept) {
  return publication.CommitPhysical(owner,a.token,a.common,{a.prepared.owner_id,a.prepared.kinematics.base_epoch,
      a.prepared.attempt,Qualification,accept});
}
void FullLedgerRig::Discard() {
  contact.DiscardTrial();publication.DiscardTrial();
  qeph.DiscardTrial();t3.DiscardTrial();welds.DiscardTrial();beams.DiscardTrial();solids.DiscardTrial();owner.Discard();
}
std::vector<double> FullLedgerRig::Force(const FullLedgerAttempt& a) {
  fe::NodalCinAssemblyView cin_view;Check(owner.BorrowCinAssembly(a.token,&cin_view));
  const auto count=m.size();std::vector<double> values(4*count);
  struct Drain{cudaStream_t stream;~Drain(){cudaStreamSynchronize(stream);}} drain{a.assembly.stream};
  Check(cudaMemcpyAsync(values.data(),a.assembly.forces.force_x,count*sizeof(double),cudaMemcpyDeviceToHost,a.assembly.stream));
  Check(cudaMemcpyAsync(values.data()+count,a.assembly.forces.force_y,count*sizeof(double),cudaMemcpyDeviceToHost,a.assembly.stream));
  Check(cudaMemcpyAsync(values.data()+2*count,a.assembly.forces.force_z,count*sizeof(double),cudaMemcpyDeviceToHost,a.assembly.stream));
  Check(cudaMemcpyAsync(values.data()+3*count,cin_view.translational_stiffness,count*sizeof(double),cudaMemcpyDeviceToHost,a.assembly.stream));
  Check(cudaStreamSynchronize(a.assembly.stream));return values;
}
} // namespace type25_source_test
