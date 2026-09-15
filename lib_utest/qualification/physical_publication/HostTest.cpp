// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/elements/publication/PhysicalChecks.h"
#include "lib_src/elements/publication/PhysicalScratchParticipationState.h"
#include "lib_src/elements/publication/PhysicalState.h"
#include "lib_src/elements/ShellPhysicalOwner.h"
#include <cstring>
#include <type_traits>

namespace physical_publication_test {
TEST(PhysicalPublicationValues, CompleteRealFamilySourcesAndCoincidentWitnesses) {
  Fixture source;
  ASSERT_FALSE(::testing::Test::HasFailure());
  EXPECT_EQ(source.source.shells.qeph_count(),2u);
  EXPECT_EQ(source.source.shells.t3_count(),1u);
  EXPECT_EQ(source.source.shells.qbat_count(),1u);
  EXPECT_EQ(source.welds.connection_count(),2u);
  EXPECT_EQ(source.beams.connection_count(),1u);
  EXPECT_EQ(source.solids.solid18().size(),1u);
  EXPECT_EQ(source.solids.solid24().size(),1u);
  EXPECT_EQ(source.solids.solid6z().size(),1u);
  EXPECT_EQ(source.rigid.groups().size(),2u);
  EXPECT_EQ(source.cin.rows().count,1u);
  EXPECT_EQ(source.ledger.scope().uncovered_nodes,0u);
  const auto& row = source.cin.rows().data[0];
  EXPECT_EQ(row.original_nsv_row,17u);
  EXPECT_FALSE(source.rigid.FindMember(row.secondary_domain_node));
  for (auto node : row.master_domain_nodes) EXPECT_FALSE(source.rigid.FindMember(node));
  for (const auto& witness : source.witnesses) {
    for (unsigned slot = 0; slot < 4; ++slot) EXPECT_EQ(witness.nodes[slot],row.master_domain_nodes[slot]);
  }
  EXPECT_NE(source.witnesses[0].source_element_id,source.witnesses[1].source_element_id);
  EXPECT_NE(source.witnesses[1].source_element_id,source.witnesses[2].source_element_id);
}
TEST(PhysicalPublicationValues,
     ContactAcceptanceLayoutsKeepRigidAndCinRolesDisjoint) {
  Fixture cin_surface(false,2.5,true,
      ContactConstraintLayout::SurfaceCinSecondary);
  ASSERT_FALSE(::testing::Test::HasFailure());
  EXPECT_EQ(cin_surface.cin.rows().data[0].secondary_domain_node,
            cin_surface.domain.Find(14));
  EXPECT_FALSE(cin_surface.rigid.FindMember(cin_surface.domain.Find(14)));

  Fixture same(false,2.5,true,
      ContactConstraintLayout::SameMergedParts);
  ASSERT_FALSE(::testing::Test::HasFailure());
  EXPECT_EQ(same.source.shells.t3_count(),2u);
  EXPECT_EQ(same.beams.connection_count(),2u);
  EXPECT_EQ(same.topology.part_count(),2u);
  ASSERT_EQ(same.rigid.groups().size(),1u);
  EXPECT_EQ(same.rigid.groups()[0].source_kind,
            fe::RigidBindingSourceKind::Part);
  EXPECT_EQ(same.rigid.groups()[0].member_count,7u);
  for (const auto id : {14u,15u,16u,17u,18u,19u,777u})
    EXPECT_TRUE(same.rigid.FindMember(same.domain.Find(id)));
  for (const auto id : {10u,11u,12u,13u})
    EXPECT_FALSE(same.rigid.FindMember(same.domain.Find(id)));

  Fixture separate(false,2.5,true,
      ContactConstraintLayout::MergedPartAndPlain);
  ASSERT_FALSE(::testing::Test::HasFailure());
  ASSERT_EQ(separate.rigid.groups().size(),2u);
  EXPECT_EQ(separate.rigid.groups()[0].source_kind,
            fe::RigidBindingSourceKind::Part);
  EXPECT_EQ(separate.rigid.groups()[1].source_kind,
            fe::RigidBindingSourceKind::NodalGroup);
  EXPECT_EQ(separate.rigid.groups()[0].source_id,
            separate.rigid.groups()[1].source_id);
  for (const auto id : {14u,15u,16u,777u,778u,17u,18u,55u})
    EXPECT_TRUE(separate.rigid.FindMember(separate.domain.Find(id)));
  EXPECT_FALSE(separate.rigid.FindMember(separate.domain.Find(19)));
  for (const auto id : {10u,11u,12u,13u})
    EXPECT_FALSE(separate.rigid.FindMember(separate.domain.Find(id)));
  for (const auto master:separate.cin.rows().data[0].master_domain_nodes)
    EXPECT_FALSE(separate.rigid.FindMember(master));
}
TEST(PhysicalPublicationValues, MandatoryPresenceAndLateSolidOmission) {
  Fixture source;
  // Presence validation is value-only and never dereferences these markers.
  fe::ShellPhysicalParticipants participants{
      reinterpret_cast<fe::qeph::QephBatch*>(1),reinterpret_cast<fe::t3::T3Batch*>(1),
      reinterpret_cast<fe::qbat::Batch*>(1),reinterpret_cast<fe::type25::Batch*>(1),
      reinterpret_cast<fe::type13::Batch*>(1),reinterpret_cast<fe::solids::Batch*>(1)};
  EXPECT_TRUE(fe::shell_publication_detail::CompletePhysicalParticipants(source.physical,participants));
  auto omitted = participants;
  omitted.solids = nullptr;
  EXPECT_FALSE(fe::shell_publication_detail::CompletePhysicalParticipants(source.physical,omitted));
  omitted = participants;
  omitted.type13 = nullptr;
  EXPECT_FALSE(fe::shell_publication_detail::CompletePhysicalParticipants(source.physical,omitted));
  omitted = participants;
  omitted.t3 = nullptr;
  EXPECT_FALSE(fe::shell_publication_detail::CompletePhysicalParticipants(source.physical,omitted));
}
TEST(PhysicalPublicationValues, ExactBudgetAndLateFailureRetry) {
  Fixture source;
  fe::ShellPhysicalPublicationForecast forecast;
  fe::ShellPublicationLimits limits;
  ASSERT_EQ(fe::ShellBatchPublication::ForecastPhysical(source.physical,1,limits,forecast).status,
      fe::ShellPublicationStatus::Success);
  EXPECT_EQ(forecast.device_bytes,0u);
  EXPECT_EQ(forecast.owned_host_bytes,6888u);
  EXPECT_EQ(forecast.startup_host_bytes,8712u);
  EXPECT_GT(forecast.startup_host_bytes,forecast.owned_host_bytes);
  RecordProperty("owned_host_bytes",std::to_string(forecast.owned_host_bytes));
  RecordProperty("tiny_startup_host_bytes",std::to_string(forecast.startup_host_bytes));
  fe::shell_physical_owner::ProofLayout original_count_proof;
  ASSERT_TRUE(fe::shell_physical_owner::ForecastProof(372435,11165,
      fe::ShellPublicationLimits::Vehicle().max_host_bytes,original_count_proof));
  // Arithmetic forecast only; no original source or owner is constructed here.
  RecordProperty("original_count_additional_startup_forecast_bytes",
      std::to_string(forecast.owned_host_bytes+original_count_proof.bytes));
  limits.max_host_bytes = forecast.startup_host_bytes;
  ASSERT_EQ(fe::ShellBatchPublication::ForecastPhysical(source.physical,1,limits,forecast).status,
      fe::ShellPublicationStatus::Success);
  const auto good = forecast;
  --limits.max_host_bytes;
  EXPECT_EQ(fe::ShellBatchPublication::ForecastPhysical(source.physical,1,limits,forecast).status,
      fe::ShellPublicationStatus::ResourceLimit);
  EXPECT_EQ(forecast.startup_host_bytes,good.startup_host_bytes);
  limits.max_host_bytes = good.startup_host_bytes;
  EXPECT_EQ(fe::ShellBatchPublication::ForecastPhysical(source.physical,1,limits,forecast).status,
      fe::ShellPublicationStatus::Success);
  limits.max_nodes = fe::MaxShellCollectionNodes+1;
  EXPECT_EQ(fe::ShellBatchPublication::ForecastPhysical(source.physical,1,limits,forecast).status,
      fe::ShellPublicationStatus::ResourceLimit);
  limits = {};
  const auto last = source.domain.node_count()-1;
  const auto position = source.domain.nodes()[last].position;
  auto* overlapping = reinterpret_cast<fe::ShellPhysicalPublicationForecast*>(
      const_cast<tl::math::Vec3*>(&source.domain.nodes()[last].position));
  EXPECT_EQ(fe::ShellBatchPublication::ForecastPhysical(source.physical,1,limits,*overlapping).status,
      fe::ShellPublicationStatus::InvalidInput);
  EXPECT_EQ(Bits(source.domain.nodes()[last].position.x),Bits(position.x));
  EXPECT_EQ(Bits(source.domain.nodes()[last].position.y),Bits(position.y));
  EXPECT_EQ(Bits(source.domain.nodes()[last].position.z),Bits(position.z));
}
TEST(PhysicalPublicationValues, CompleteTypedDiagnosticComparisonIncludesLastSolidAndActivity) {
  fe::ShellPhysicalDiagnostics value;
  value.base_stamp.owner_id = 81;
  value.has_solids = true;
  value.valid = true;
  auto changed = value;
  changed.solids.physical_hourglass_work_increment_j[2] = 1;
  EXPECT_FALSE(fe::shell_publication_detail::SamePhysicalDiagnostics(value,changed));
  changed = value;
  changed.type13.active_count = 1;
  EXPECT_FALSE(fe::shell_publication_detail::SamePhysicalDiagnostics(value,changed));
  changed = value;
  changed.kinetic_available = true;
  EXPECT_FALSE(fe::shell_publication_detail::SamePhysicalDiagnostics(value,changed));
  EXPECT_TRUE(fe::shell_publication_detail::SamePhysicalDiagnostics(value,value));
}
TEST(PhysicalPublicationValues,
     ScratchRosterHasFixedAbiNonforgeableReceiptAndExactSeparateCap) {
  using Kind=fe::ShellPhysicalScratchContributorKind;
  using Receipt=fe::ShellPhysicalScratchParticipationReceipt;
  using Issuer=fe::ShellPhysicalScratchParticipation;
  EXPECT_EQ(static_cast<unsigned>(Kind::MappedWall),0u);
  EXPECT_EQ(static_cast<unsigned>(Kind::SelfContact),1u);
  EXPECT_EQ(static_cast<unsigned>(fe::ShellPublicationStatus::NonfiniteResult),8u);
  EXPECT_EQ(static_cast<unsigned>(fe::ShellPublicationStatus::ParticipationFailure),9u);
  EXPECT_FALSE(std::is_aggregate_v<Receipt>);
  EXPECT_TRUE(std::is_default_constructible_v<Receipt>);
  EXPECT_TRUE(std::is_copy_constructible_v<Receipt>);
  EXPECT_FALSE((std::is_constructible_v<Receipt,std::uint64_t,
      std::uint64_t,bool>));
  EXPECT_FALSE((std::is_constructible_v<Receipt,fe::NodalValidationReceipt>));
  EXPECT_FALSE((std::is_convertible_v<fe::NodalValidationReceipt,Receipt>));
  EXPECT_FALSE(Receipt{}.valid());
  static_assert(sizeof(Issuer)==112);
  static_assert(sizeof(Receipt)==288);
  static_assert(sizeof(fe::ShellPhysicalScratchRoster)==32);
  RecordProperty("scratch_participation_issuer_bytes",std::to_string(sizeof(Issuer)));
  RecordProperty("scratch_participation_receipt_bytes",std::to_string(sizeof(Receipt)));
  RecordProperty("scratch_participation_roster_bytes",
      std::to_string(sizeof(fe::ShellPhysicalScratchRoster)));

  auto* wall=reinterpret_cast<Issuer*>(std::uintptr_t{4096});
  auto* self=reinterpret_cast<Issuer*>(std::uintptr_t{8192});
  constexpr std::uint64_t wall_source=720,self_source=721;
  fe::ShellPhysicalScratchParticipationForecast forecast;
  const auto untouched=forecast;
  EXPECT_EQ(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(
      {},{},forecast).status,fe::ShellPublicationStatus::InvalidInput);
  EXPECT_EQ(forecast.total_host_bytes,untouched.total_host_bytes);
  fe::ShellPhysicalScratchRoster self_only{{},{self,self_source}};
  ASSERT_EQ(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(
      self_only,{},forecast).status,fe::ShellPublicationStatus::Success);
  EXPECT_EQ(forecast.publication_host_bytes,112u);
  EXPECT_EQ(forecast.total_host_bytes,224u);
  RecordProperty("scratch_participation_publication_bytes",
      std::to_string(forecast.publication_host_bytes));
  RecordProperty("scratch_participation_self_only_total_bytes",
      std::to_string(forecast.total_host_bytes));
  EXPECT_EQ(forecast.configured_issuer_host_bytes,sizeof(Issuer));
  EXPECT_EQ(forecast.total_host_bytes,
      forecast.publication_host_bytes+sizeof(Issuer));
  const auto exact=forecast;
  fe::ShellPhysicalScratchParticipationLimits cap{exact.total_host_bytes-1};
  EXPECT_EQ(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(
      self_only,cap,forecast).status,fe::ShellPublicationStatus::ResourceLimit);
  EXPECT_EQ(forecast.total_host_bytes,exact.total_host_bytes);
  ++cap.max_host_bytes;
  EXPECT_EQ(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(
      self_only,cap,forecast).status,fe::ShellPublicationStatus::Success);
  fe::ShellPhysicalScratchRoster both{{wall,wall_source},
                                      {self,self_source}};
  ASSERT_EQ(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(
      both,{},forecast).status,fe::ShellPublicationStatus::Success);
  EXPECT_EQ(forecast.total_host_bytes,336u);
  RecordProperty("scratch_participation_wall_self_total_bytes",
      std::to_string(forecast.total_host_bytes));
  EXPECT_EQ(forecast.configured_issuer_host_bytes,2*sizeof(Issuer));
  both.self_contact.issuer=wall;
  EXPECT_EQ(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(
      both,{},forecast).status,fe::ShellPublicationStatus::InvalidInput);
}
TEST(PhysicalPublicationValues,
     TaggedPhysicalRuntimeSupportsZeroWitnessByteCopyAndBothDestructionPaths) {
  using Runtime=fe::shell_publication_detail::PhysicalRuntimeState;
  using State=fe::shell_publication_detail::PhysicalScratchParticipationState;
  static_assert(sizeof(Runtime)==2*sizeof(std::size_t));
  static_assert(std::is_trivially_copyable_v<Runtime>);

  Runtime absent;
  absent.SetCinCounts(17,0);
  EXPECT_FALSE(absent.HasScratchParticipation());
  EXPECT_EQ(absent.CinAttachmentCount(),17u);
  EXPECT_EQ(absent.CinWitnessCount(),0u);
  std::array<unsigned char,sizeof(Runtime)> bytes{};
  std::memcpy(bytes.data(),&absent,sizeof(absent));
  Runtime absent_copy;
  std::memcpy(&absent_copy,bytes.data(),sizeof(absent_copy));
  EXPECT_FALSE(absent_copy.HasScratchParticipation());
  EXPECT_EQ(absent_copy.CinAttachmentCount(),17u);
  EXPECT_EQ(absent_copy.CinWitnessCount(),0u);

  State state;
  absent.SetScratchParticipation(&state);
  EXPECT_TRUE(absent.HasScratchParticipation());
  EXPECT_EQ(absent.ScratchParticipation(),&state);
  std::memcpy(bytes.data(),&absent,sizeof(absent));
  Runtime present_copy;
  std::memcpy(&present_copy,bytes.data(),sizeof(present_copy));
  EXPECT_TRUE(present_copy.HasScratchParticipation());
  EXPECT_EQ(present_copy.ScratchParticipation(),&state);
  present_copy.SetScratchParticipation(nullptr);
  EXPECT_FALSE(present_copy.HasScratchParticipation());
  EXPECT_EQ(present_copy.CinAttachmentCount(),0u);
  EXPECT_EQ(present_copy.CinWitnessCount(),0u);

  {
    fe::shell_publication_detail::PhysicalState physical{{},{}};
    physical.SetCinCounts(0,0);
  }
  {
    fe::shell_publication_detail::PhysicalState physical{{},{}};
    physical.runtime.SetScratchParticipation(new State);
  }
}
} // namespace physical_publication_test
