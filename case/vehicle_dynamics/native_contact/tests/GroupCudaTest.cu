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
}
