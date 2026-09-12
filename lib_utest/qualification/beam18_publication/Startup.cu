// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
namespace beam18_publication_test {
fe::ShellPhysicalParticipants Rig::Participants() {
  auto result=mixed.Participants(); result.beam18=&beam; return result;
}
bool Rig::Initialize(bool initialize_beam,bool attach) {
  auto& f=mixed.fixture;
  if (!source.Initialize(f)) return false;
  const auto cin=f.CinStartup();
  const auto witness=f.WitnessSource();
  if (!Good(mixed.owner.Initialize(f.OwnerConfig(),
      {f.x.data(),f.v.data(),f.w.data(),f.domain.node_count(),f.q.data()},f.im.data(),
      {f.fixed.data(),f.rotation_fixed.data(),f.ij.data(),f.present.data()},source.rigid,&cin))) return false;
  const auto owner=mixed.owner.accepted();
  fe::qeph::QephBatchConfig q;
  q.owner=owner; q.configuration_id=existing::Configuration; q.qualification_id=existing::Qualification;
  q.element_count=2; q.usage=fe::qeph::BatchUsage::CoupledForces;
  if (!Good(mixed.qeph.InitializeMapped(q,source.physical,mixed.owner,witness))) return false;
  fe::t3::T3BatchConfig t;
  t.owner=owner; t.configuration_id=existing::Configuration; t.qualification_id=existing::Qualification;
  t.element_count=1; t.usage=fe::t3::BatchUsage::CoupledForces;
  if (!Good(mixed.t3.InitializeMapped(t,source.physical,mixed.owner,witness))) return false;
  fe::qbat::BatchConfig bq;
  bq.owner=owner; bq.configuration_id=existing::Configuration; bq.qualification_id=existing::Qualification;
  bq.element_count=1; bq.usage=fe::qbat::BatchUsage::CoupledForces;
  if (!Good(mixed.qbat.InitializeMapped(bq,source.physical,mixed.owner,witness))) return false;
  fe::type25::BatchConfig weld;
  weld.owner=owner; weld.configuration_id=existing::Configuration; weld.qualification_id=existing::Qualification;
  weld.element_count=2;
  if (!Good(mixed.welds.InitializeMapped(weld,source.physical,mixed.owner,witness,fe::type25::CapacityProfile::Legacy))) return false;
  fe::type13::BatchConfig spring;
  spring.owner=owner; spring.configuration_id=existing::Configuration; spring.qualification_id=existing::Qualification;
  spring.assembly=fe::type13::BatchAssembly::CinNativeStiffness;
  if (!Good(mixed.beams.InitializeMapped(spring,source.physical,source.rigid,mixed.owner,witness)) ||
      !mixed.InitializeSolids() || (initialize_beam && !InitializeBeam())) return false;
  // Reuse the existing epoch-zero accepted-force startup proof for the five
  // legacy participants. Neither owner time nor beam history advances here.
  fe::NodalTrialToken token; fe::NodalAssemblyView assembly;
  if (!Good(mixed.owner.BeginTrial(&token,&assembly)) ||
      !Good(mixed.qeph.AssembleMappedAccepted(mixed.owner,token,assembly)) ||
      !Good(mixed.t3.AssembleMappedAccepted(mixed.owner,token,assembly)) ||
      !Good(mixed.qbat.AssembleMappedAccepted(mixed.owner,token,assembly)) ||
      !Good(mixed.welds.AssembleMappedAccepted(mixed.owner,token,assembly)) ||
      !Good(mixed.beams.AssembleMappedAccepted(mixed.owner,token,assembly))) return false;
  mixed.owner.Discard(); mixed.qeph.DiscardTrial(); mixed.t3.DiscardTrial();
  mixed.qbat.DiscardTrial(); mixed.welds.DiscardTrial(); mixed.beams.DiscardTrial();
  return !attach || Attach();
}
bool Rig::InitializeBeam(const b::Model* override_model) {
  b::BatchConfig c;
  c.owner=mixed.owner.accepted(); c.configuration_id=existing::Configuration; c.qualification_id=existing::Qualification;
  c.profile=b::BatchProfile::PhysicalCinCircularFourPointLaw44V1;
  c.cin_attachment_count=mixed.fixture.ranges.size(); c.cin_witness_count=mixed.fixture.witnesses.size();
  return Good(beam.InitializeJoined(c,override_model ? *override_model : source.model));
}
bool Rig::Attach() {
  return Good(mixed.publication.InitializePhysical(mixed.owner,source.physical,source.rigid,
      mixed.fixture.WitnessSource(),Participants(),mixed.fixture.Identity()));
}
} // namespace beam18_publication_test
