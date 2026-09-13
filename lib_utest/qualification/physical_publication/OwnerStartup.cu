// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"

namespace physical_publication_test {
bool Rig::Initialize(bool initialize_solids,bool attach) {
  return InitializeAgainst(fixture.physical,initialize_solids,attach);
}
bool Rig::InitializeAgainst(const fe::ShellPhysicalBinding& physical,
    bool initialize_solids,bool attach) {
  const auto cin = fixture.CinStartup();
  if (!Good(owner.Initialize(fixture.OwnerConfig(),
      {fixture.x.data(),fixture.v.data(),fixture.w.data(),fixture.domain.node_count(),fixture.q.data()},
      fixture.im.data(),{fixture.fixed.data(),fixture.rotation_fixed.data(),fixture.ij.data(),fixture.present.data()},
      fixture.rigid,&cin))) return false;
  fe::qeph::QephBatchConfig q;
  q.owner = owner.accepted(); q.configuration_id = Configuration; q.qualification_id = Qualification;
  q.element_count = 2; q.usage = fe::qeph::BatchUsage::CoupledForces;
  if (!Good(qeph.InitializeMapped(q,physical,owner,fixture.WitnessSource()))) return false;
  fe::t3::T3BatchConfig t;
  t.owner = owner.accepted(); t.configuration_id = Configuration; t.qualification_id = Qualification;
  t.element_count = 1; t.usage = fe::t3::BatchUsage::CoupledForces;
  if (!Good(t3.InitializeMapped(t,physical,owner,fixture.WitnessSource()))) return false;
  fe::qbat::BatchConfig b;
  b.owner = owner.accepted(); b.configuration_id = Configuration; b.qualification_id = Qualification;
  b.element_count = 1; b.usage = fe::qbat::BatchUsage::CoupledForces;
  if (!Good(qbat.InitializeMapped(b,physical,owner,fixture.WitnessSource()))) return false;
  fe::type25::BatchConfig w;
  w.owner = owner.accepted(); w.configuration_id = Configuration; w.qualification_id = Qualification;
  w.element_count = 2;
  if (!Good(welds.InitializeMapped(w,physical,owner,fixture.WitnessSource(),fe::type25::CapacityProfile::Legacy)))
    return false;
  fe::type13::BatchConfig beam;
  beam.owner = owner.accepted(); beam.configuration_id = Configuration; beam.qualification_id = Qualification;
  beam.assembly = fe::type13::BatchAssembly::CinNativeStiffness;
  if (!Good(beams.InitializeMapped(beam,physical,fixture.rigid,owner,fixture.WitnessSource()))) return false;
  if (initialize_solids && !InitializeSolids()) return false;
  // Existing mapped/TYPE13 startup APIs bind their live initial cache on the
  // first actual accepted assembly. Discard this proof; no owner step occurs.
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  if (!Good(owner.BeginTrial(&token,&assembly)) ||
      !Good(qeph.AssembleMappedAccepted(owner,token,assembly)) ||
      !Good(t3.AssembleMappedAccepted(owner,token,assembly)) ||
      !Good(qbat.AssembleMappedAccepted(owner,token,assembly)) ||
      !Good(welds.AssembleMappedAccepted(owner,token,assembly)) ||
      !Good(beams.AssembleMappedAccepted(owner,token,assembly))) return false;
  owner.Discard();
  qeph.DiscardTrial(); t3.DiscardTrial(); qbat.DiscardTrial(); welds.DiscardTrial(); beams.DiscardTrial();
  return !attach || AttachAgainst(physical);
}
bool Rig::InitializeSolids() {
  fe::solids::BatchConfig config;
  config.owner = owner.accepted();
  config.configuration_id = Configuration;
  config.qualification_id = Qualification;
  config.profile = fe::solids::BatchProfile::PhysicalCinV1;
  config.cin_attachment_count = fixture.ranges.size();
  config.cin_witness_count = fixture.witnesses.size();
  return Good(solids.InitializeJoined(config,fixture.solids));
}
bool Rig::Attach() {
  return AttachAgainst(fixture.physical);
}
bool Rig::AttachAgainst(const fe::ShellPhysicalBinding& physical) {
  return Good(publication.InitializePhysical(owner,physical,fixture.rigid,
      fixture.WitnessSource(),Participants(),fixture.Identity()));
}
bool Rig::ConfigureScratch(bool mapped_wall,bool self_contact) {
  fe::ShellPhysicalScratchRoster roster;
  if(mapped_wall) roster.mapped_wall={&mapped_wall_participation,MappedWallSource};
  if(self_contact) roster.self_contact={&self_contact_participation,SelfContactSource};
  return Good(publication.ConfigurePhysicalScratchParticipation(owner,fixture.physical,
      Participants(),fixture.Identity(),roster));
}
} // namespace physical_publication_test
