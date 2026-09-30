// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Rig.h"
namespace glass_removal_test {
void Rig::Initialize() {
  std::vector<std::uint8_t> fixed(11),rotation_fixed(11);
  for(unsigned i=0;i<4;++i){fixed[i]=7;rotation_fixed[i]=1;}
  base.fixed_dt=Dt;base.Prepare(source,fixed,rotation_fixed,{0,0,0});
  const auto declarations=Failures(source);
  Check(failure.InitializeExecution(base.catalog,declarations.data(),declarations.size()));
  const auto bound=physical.InitializeExecution({&base.shells,&base.catalog,&failure,nullptr},base.ledger,base.execution);
  base.Require(bool(bound),bound.message);
  Check(base.Initialize(owner));
  fe::qeph::QephBatchConfig qc;qc.startup=base.startup;qc.owner=owner.accepted();
  qc.configuration_id=Configuration;qc.qualification_id=base.Qualification;
  qc.element_count=2;qc.usage=fe::qeph::BatchUsage::CoupledForces;
  Check(qeph.InitializeMapped(qc,physical,owner,base.Witnesses()));
  fe::t3::T3BatchConfig tc;tc.startup=base.startup;tc.owner=owner.accepted();
  tc.configuration_id=Configuration;tc.qualification_id=base.Qualification;
  tc.element_count=1;tc.usage=fe::t3::BatchUsage::CoupledForces;
  Check(triangle.InitializeMapped(tc,physical,owner,base.Witnesses()));
  // The existing mapped producers authenticate their actual initial caches;
  // this startup proof publishes no physical time or damage.
  Attempt proof;Check(owner.BeginTrial(&proof.token,&proof.assembly));
  Check(qeph.AssembleMappedAccepted(owner,proof.token,proof.assembly));
  Check(triangle.AssembleMappedAccepted(owner,proof.token,proof.assembly));
  qeph.DiscardTrial();triangle.DiscardTrial();owner.Discard();
  Check(publication.InitializePhysical(owner,physical,base.rigid,base.Witnesses(),Participants(),Identity()));
  self_source.Initialize(base,source,false);wall_source.Initialize(base,source,true);
  auto config=type25_source_test::Fixture::Config();
  // SourceAdmissionFixture defaults to a zero clamp for admission-only tests.
  // This mechanics coupon declares the same open coefficient range as the
  // existing MovingCacheRig, admitting its explicit1e6 source coefficients.
  config.lifecycle.minimum_coefficient=0;
  config.lifecycle.maximum_coefficient=1e30;
  config.response_mass=n::ResponseMassPolicy::AcceptedOwnerCoefficients;
  config.physical_source=n::PhysicalSourceProfile::CompleteBoundLedger;
  config.activity=n::ContactActivityPolicy::ShellRemoval;
  self_source.PrepareInitial(base,config);wall_source.PrepareInitial(base,config);
  Check(self.GeneralInitialize(config,self_source.Moving(),self_source.initial,owner,publication,physical,Participants(),Identity()));
  Check(wall.GeneralInitialize(config,wall_source.Common(),wall_source.ready,wall_source.initial,
      owner,publication,physical,Participants(),Identity()));
  base.Require(self.initialization_diagnostics().available&&wall.initialization_diagnostics().available,
      "Both contacts require the genuine initial-source handoff");
  const std::array<fe::NativeContactRosterEntry,2> entries{self.native_roster_entry(),wall.native_roster_entry()};
  Check(publication.ConfigurePhysicalScratchParticipation(owner,physical,Participants(),Identity(),
      {{},{},{entries.data(),entries.size()}}));
}
} // namespace glass_removal_test
