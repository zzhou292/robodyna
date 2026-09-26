// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FullLedgerRig.h"
#include <cstring>
namespace type25_source_test {
TEST(NativeType25FullLedgerCuda, RealParticipantsPublishActiveContactUnderTheOneOwner) {
  FullLedgerRig rig;
    ASSERT_NO_THROW(rig.Initialize());
  std::uint64_t active=0;
  for(unsigned step=0;step<3;++step) {
    FullLedgerAttempt a;
    ASSERT_NO_THROW(rig.Begin(a));
    ASSERT_NO_THROW(Check(rig.contact.AssembleAccepted(rig.owner,a.token,a.assembly)));
    active+=rig.contact.last_diagnostics().active_forces;
    EXPECT_EQ(rig.contact.accepted().stamp.epoch,step);
    ASSERT_NO_THROW(rig.Prepare(a));
    ASSERT_NO_THROW(rig.Seal(a));
    ASSERT_NO_THROW(Check(rig.Commit(a)));
    EXPECT_EQ(rig.owner.accepted().epoch,step+1);
    const auto contact=rig.contact.accepted();
    EXPECT_TRUE(contact.available);EXPECT_EQ(contact.generation,step+1);
    EXPECT_EQ(contact.stamp.epoch,step+1);EXPECT_EQ(contact.force_base_stamp.epoch,step);
  }
  EXPECT_GT(active,0u);
}
TEST(NativeType25FullLedgerCuda, MissingActualParticipantsCannotInitializeTheNativeTransaction) {
  FullLedgerRig rig;
    ASSERT_NO_THROW(rig.Initialize(false));
  for(unsigned missing=0;missing<3;++missing) {
    auto participants=rig.Participants();
    if(missing==0)participants.solids=nullptr;
    if(missing==1)participants.type13=nullptr;
    if(missing==2)participants.type25=nullptr;
    n::Transaction rejected;
    EXPECT_EQ(rejected.Initialize(rig.fixture.Config(),rig.Source(),rig.owner,rig.publication,
        rig.fixture.physical,participants,rig.Identity()).status,n::TransactionStatus::PublicationFailure);
    EXPECT_FALSE(rejected.source_info().available);EXPECT_EQ(rig.owner.accepted().epoch,0u);
  }
  ASSERT_NO_THROW(Check(rig.contact.Initialize(rig.fixture.Config(),rig.Source(),rig.owner,rig.publication,
      rig.fixture.physical,rig.Participants(),rig.Identity())));
}
TEST(NativeType25FullLedgerCuda, ActiveCommonRejectionPreservesAcceptedStateAndReassemblesExactly) {
  FullLedgerRig rig;
    ASSERT_NO_THROW(rig.Initialize());
  const auto original=rig.owner.accepted();const auto old_contact=rig.contact.accepted();
  FullLedgerAttempt rejected;
    ASSERT_NO_THROW(rig.Begin(rejected));
  ASSERT_NO_THROW(Check(rig.contact.AssembleAccepted(rig.owner,rejected.token,rejected.assembly)));
  ASSERT_GT(rig.contact.last_diagnostics().active_forces,0u);
  std::vector<double> first;
    ASSERT_NO_THROW(first=rig.Force(rejected));
  ASSERT_NO_THROW(rig.Prepare(rejected));
    ASSERT_NO_THROW(rig.Seal(rejected));
  EXPECT_NE(rig.Commit(rejected,false).status,fe::ShellPublicationStatus::Success);
  rig.Discard();
  EXPECT_TRUE(fe::trial_identity::SameStamp(original,rig.owner.accepted()));
  EXPECT_EQ(rig.contact.accepted().generation,old_contact.generation);
  EXPECT_EQ(rig.contact.accepted().selectors.history,old_contact.selectors.history);
  EXPECT_EQ(rig.contact.accepted().selectors.has_reference,old_contact.selectors.has_reference);
  FullLedgerAttempt retry;
    ASSERT_NO_THROW(rig.Begin(retry));
  ASSERT_NO_THROW(Check(rig.contact.AssembleAccepted(rig.owner,retry.token,retry.assembly)));
  std::vector<double> repeated;
    ASSERT_NO_THROW(repeated=rig.Force(retry));
  ASSERT_EQ(first.size(),repeated.size());EXPECT_EQ(std::memcmp(first.data(),repeated.data(),first.size()*sizeof(double)),0);
  ASSERT_NO_THROW(rig.Prepare(retry));
    ASSERT_NO_THROW(rig.Seal(retry));
    ASSERT_NO_THROW(Check(rig.Commit(retry)));
  EXPECT_EQ(rig.owner.accepted().epoch,1u);EXPECT_EQ(rig.contact.accepted().generation,1u);
}
} // namespace type25_source_test
