// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
namespace beam18_publication_test {
TEST(BeamPublicationCuda, LateMissingBatchAndForeignSourceClaimNothingThenValidRetry) {
  b::Batch foreign_batch; // Outlives the actual publisher even on a failed assertion.
  Rig rig; ASSERT_TRUE(rig.Initialize(false,false));
  const auto before=rig.mixed.owner.accepted();
  auto participants=rig.Participants(); participants.beam18=nullptr;
  EXPECT_NE(rig.mixed.publication.InitializePhysical(rig.mixed.owner,rig.source.physical,rig.source.rigid,
      rig.mixed.fixture.WitnessSource(),participants,rig.mixed.fixture.Identity()).status,
      fe::ShellPublicationStatus::Success);
  EXPECT_EQ(rig.mixed.publication.InitializePhysical(rig.mixed.owner,rig.source.physical,rig.source.rigid,
      rig.mixed.fixture.WitnessSource(),rig.Participants(),rig.mixed.fixture.Identity()).status,
      fe::ShellPublicationStatus::NotInitialized);
  std::vector<b::ParentInput> foreign;
  for (const auto& parent:rig.source.model.parents()) {
    auto input=parent.reference.input(); input.source_element_id+=100;
    b::ParentInput row;
    ASSERT_EQ(b::InitializeReference(input,row.reference),b::Status::Success);
    row.material=rig.source.model.materials()[parent.material_index].value;
    foreign.push_back(row);
  }
  b::Model foreign_model;
  ASSERT_TRUE(foreign_model.Initialize(rig.mixed.fixture.domain,{1,{foreign.data(),foreign.size()},
      b::ModelProfile::CircularFourPointLaw44V1}));
  b::BatchConfig config;
  config.owner=before; config.configuration_id=existing::Configuration; config.qualification_id=existing::Qualification;
  config.profile=b::BatchProfile::PhysicalCinCircularFourPointLaw44V1;
  config.cin_attachment_count=1; config.cin_witness_count=3;
  ASSERT_TRUE(Good(foreign_batch.InitializeJoined(config,foreign_model)));
  participants=rig.Participants(); participants.beam18=&foreign_batch;
  EXPECT_NE(rig.mixed.publication.InitializePhysical(rig.mixed.owner,rig.source.physical,rig.source.rigid,
      rig.mixed.fixture.WitnessSource(),participants,rig.mixed.fixture.Identity()).status,
      fe::ShellPublicationStatus::Success);
  EXPECT_TRUE(fe::trial_identity::SameStamp(before,rig.mixed.owner.accepted()));
  ASSERT_TRUE(rig.InitializeBeam()); ASSERT_TRUE(rig.Attach());
  fe::ShellBatchPublication second;
  EXPECT_NE(second.InitializePhysical(rig.mixed.owner,rig.source.physical,rig.source.rigid,
      rig.mixed.fixture.WitnessSource(),rig.Participants(),rig.mixed.fixture.Identity()).status,
      fe::ShellPublicationStatus::Success);
  Snapshot accepted; ASSERT_TRUE(rig.Read(accepted));
  auto* overlap=reinterpret_cast<fe::ShellPhysicalDiagnostics*>(
      const_cast<b::Parent*>(rig.source.model.parents().data()));
  EXPECT_EQ(rig.mixed.publication.CopyAcceptedPhysicalDiagnostics(rig.mixed.owner.accepted(),overlap).status,
      fe::ShellPublicationStatus::InvalidInput);
  auto* curve=reinterpret_cast<fe::ShellPhysicalDiagnostics*>(
      const_cast<double*>(rig.source.model.materials()[0].value.curve.plastic_strain));
  EXPECT_EQ(rig.mixed.publication.CopyAcceptedPhysicalDiagnostics(rig.mixed.owner.accepted(),curve).status,
      fe::ShellPublicationStatus::InvalidInput);
}
} // namespace beam18_publication_test
