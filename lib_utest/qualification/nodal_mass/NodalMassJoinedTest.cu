#include "NodalMassCudaFixture.h"

namespace nodal_mass_test {
TEST_F(NodalMassCuda, BothFamiliesBindCombinedActualCoefficientsWithoutPublishingMissingConnector) {
  Joined rig; ASSERT_TRUE(rig.Initialize(true,true));
  temporal::Snapshot before,after; ASSERT_TRUE(temporal::Read(rig.owner,before));
  for(unsigned retry=0;retry<2;++retry) {
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_EQ(rig.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
    EXPECT_EQ(rig.qb.AssembleAccepted(rig.owner,view).status,q::BatchStatus::Success);
    EXPECT_EQ(rig.tb.AssembleAccepted(rig.owner,view).status,t::BatchStatus::Success);
    rig.owner.Discard(); rig.qb.DiscardTrial(); rig.tb.DiscardTrial();
    fe::ShellBatchPublication publication;
    EXPECT_EQ(publication.Initialize(rig.owner,rig.qb,rig.tb).status,fe::ShellPublicationStatus::NotJoined);
    EXPECT_EQ(publication.allocations().device_bytes,0u);
    ASSERT_TRUE(temporal::Read(rig.owner,after)); temporal::SameState(before,after);
  }
}
TEST_F(NodalMassCuda, ShellOnlyAndCombinedCoefficientsCannotBeSubstitutedAtTheOwner) {
  for(bool combined_owner:{false,true}) {
    Joined rig; ASSERT_TRUE(rig.Initialize(combined_owner,!combined_owner));
    temporal::Snapshot before,after; ASSERT_TRUE(temporal::Read(rig.owner,before));
    fe::NodalTrialToken token; fe::NodalAssemblyView view;
    ASSERT_EQ(rig.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
    EXPECT_EQ(rig.qb.AssembleAccepted(rig.owner,view).status,q::BatchStatus::InvalidMass);
    // A rejected contributor makes the entire assembly sticky. Check the
    // second family's independent coefficient admission on a fresh attempt.
    rig.owner.Discard(); rig.qb.DiscardTrial(); rig.tb.DiscardTrial();
    ASSERT_EQ(rig.owner.BeginTrial(&token,&view).status,fe::NodalStatus::Ok);
    EXPECT_EQ(rig.tb.AssembleAccepted(rig.owner,view).status,t::BatchStatus::InvalidMass);
    rig.owner.Discard(); rig.qb.DiscardTrial(); rig.tb.DiscardTrial();
    ASSERT_TRUE(temporal::Read(rig.owner,after)); temporal::SameState(before,after);
  }
}
TEST_F(NodalMassCuda, CompleteSourceMismatchRejectsBeforeDeviceAllocationAndCanRetry) {
  Joined rig; ASSERT_TRUE(rig.Initialize(true,true));
  auto changed=shell_binding_test::Edge(); changed.qeph.young_modulus*=2.;
  fe::ShellBatchBinding wrong;
  ASSERT_EQ(wrong.Initialize(changed).status,fe::ShellBindingStatus::Success);
  q::QephBatch qb; q::QephBatchConfig qc; qc.owner=rig.owner.accepted();
  qc.element_count=1; qc.configuration_id=7; qc.qualification_id=8;
  qc.usage=q::BatchUsage::CoupledForces;
  EXPECT_EQ(qb.InitializeJoined(qc,wrong,rig.mass).status,q::BatchStatus::InvalidInput);
  EXPECT_EQ(qb.allocations().device_bytes,0u);
  EXPECT_EQ(qb.InitializeJoined(qc,rig.shells,rig.mass).status,q::BatchStatus::Success);
}
} // namespace nodal_mass_test
