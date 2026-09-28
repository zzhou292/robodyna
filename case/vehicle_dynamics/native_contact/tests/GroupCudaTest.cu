#include "GroupFixture.h"
namespace native_app_group_test {
TEST(NativeAppGroupCuda, BothHistoriesMatchDirectNativeGroupAcrossActiveImpact) {
    Rig wrapped;base::Rig direct;
    ASSERT_NO_THROW(wrapped.Initialize());
    ASSERT_NO_THROW(direct.Initialize());
    std::size_t active=0;
    for(unsigned i=0;i<470;++i) {
        ASSERT_NO_THROW(wrapped.Step());
        ASSERT_NO_THROW(direct.Step());
        ASSERT_EQ(wrapped.observed.count,2u);
        for(unsigned slot=0;slot<2;++slot)active+=wrapped.observed.interfaces[slot].diagnostics.active_forces;
    }
    EXPECT_GT(active,0u);
    ASSERT_NO_FATAL_FAILURE(NumericalParity(wrapped.Read(),direct.Read()));
    EXPECT_EQ(wrapped.group->role(0),app::Role::Self);EXPECT_EQ(wrapped.group->role(1),app::Role::MeshWall);
    EXPECT_EQ(wrapped.group->scratch_receipts().native_interfaces.count,0u);
    EXPECT_GT(wrapped.group->device_bytes(),0u);
}
TEST(NativeAppGroupCuda, RejectedCommonAttemptPreservesBothAndRetriesExactForce) {
    Rig rig;ASSERT_NO_THROW(rig.Initialize());
    for(unsigned i=0;i<380;++i)ASSERT_NO_THROW(rig.Step());
    const auto before=rig.Read();base::Attempt attempt;
    ASSERT_NO_THROW(rig.Assemble(attempt));
    std::array<double,54> force,again;std::array<double,18> stiffness,repeated;
    ASSERT_NO_THROW(rig.backend.physical.Force(attempt,force,stiffness));
    ASSERT_NO_THROW(rig.Prepare(attempt));
    EXPECT_NE(rig.backend.physical.Commit(attempt,false).status,base::fe::ShellPublicationStatus::Success);
    rig.Discard();ASSERT_NO_FATAL_FAILURE(base::Same(before,rig.Read()));
    base::Attempt retry;ASSERT_NO_THROW(rig.Assemble(retry));
    ASSERT_NO_THROW(rig.backend.physical.Force(retry,again,repeated));
    base::Bits(force,again);base::Bits(stiffness,repeated);
    ASSERT_NO_THROW(rig.Prepare(retry));
    ASSERT_NO_THROW(Check(rig.backend.physical.Commit(retry)));
    rig.group->Committed();EXPECT_EQ(rig.Read().physical.stamp.epoch,before.physical.stamp.epoch+1);
}
TEST(NativeAppGroupCuda, WrongAppPhaseKeepsOutputAndRevokesEveryPendingChild) {
    Rig rig;ASSERT_NO_THROW(rig.Initialize());
    const auto before=rig.Read();base::Attempt attempt;
    ASSERT_NO_THROW(rig.Assemble(attempt));
    rig.observed.count=77;
    EXPECT_THROW(rig.group->Assemble(rig.backend.physical.owner,attempt.token,attempt.assembly,rig.observed),std::exception);
    EXPECT_EQ(rig.observed.count,77u);EXPECT_EQ(rig.group->scratch_receipts().native_interfaces.count,0u);
    rig.Discard();ASSERT_NO_FATAL_FAILURE(base::Same(before,rig.Read()));
    ASSERT_NO_THROW(rig.Step());EXPECT_EQ(rig.Read().physical.stamp.epoch,1u);
}
TEST(NativeAppGroupCuda, DetachedPublisherGroupRejectsBeforeDereferencingFormerPublisher) {
  base::Rig rig;
  ASSERT_NO_THROW(rig.Initialize());
  base::Attempt a;
  ASSERT_NO_THROW(rig.Begin(a));
  ASSERT_NO_THROW(rig.Assemble(a));
  ASSERT_NO_THROW(rig.PrepareMaterials(a));
  const auto stamp=rig.physical.owner.accepted();
  rig.physical.publication.reset();
  base::fe::ShellBatchPublication replacement;
  std::array<base::n::Transaction*,2> members{
    {
      rig.native[0].get(),rig.native[1].get()
    }
  };
  const auto report=base::n::Transaction::SealCandidateGroup(members.data(),2,replacement,rig.physical.owner,a.token,a.prepared,a.common,rig.receipts.data(),2);
  EXPECT_EQ(report.interface_index,0u);
  EXPECT_EQ(report.report.status,base::n::TransactionStatus::StaleAttempt);
  EXPECT_TRUE(base::fe::trial_identity::SameStamp(stamp,rig.physical.owner.accepted()));
}

}
