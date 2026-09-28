// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeContactFixture.h"
namespace physical_publication_test {
using namespace native_contact_test;
namespace {
fe::NativeContactSelectors ActivityPlan(unsigned history,unsigned reference,
    std::uint64_t reference_generation,unsigned activity,std::uint64_t generation,
    std::uint64_t reference_activity) {
  fe::NativeContactSelectors p{history,reference,reference_generation,true};
  p.activity=activity;p.activity_generation=generation;
  p.reference_activity_generation=reference_activity;return p;
}
}
TEST(NativeContactActivityPublicationCuda,CommonCommitKeepsForceBaseAndNextActivityDistinct) {
  Rig rig;fe::NativeContactPublicationState state;
  ASSERT_TRUE(Bind(rig,state,true));
  const auto initial=state.Accepted(rig.owner);
  ASSERT_TRUE(initial.available);
  EXPECT_EQ(initial.selectors.activity_generation,1u);
  EXPECT_EQ(initial.selectors.reference_activity_generation,0u);
  Attempt first;ASSERT_TRUE(Prepare(rig,first));
  ASSERT_TRUE(Stage(rig,state,first,ActivityPlan(1,1,1,1,2,1)));
  EXPECT_EQ(state.Accepted(rig.owner).selectors.activity_generation,1u);
  ASSERT_TRUE(Good(rig.publication.CommitPhysical(rig.owner,first.token,first.common,Receipt(first))));
  const auto accepted=state.Accepted(rig.owner);
  EXPECT_EQ(accepted.selectors.activity,1u);
  EXPECT_EQ(accepted.selectors.activity_generation,2u);
  EXPECT_EQ(accepted.selectors.reference_activity_generation,1u);
  EXPECT_EQ(accepted.force_base_stamp.epoch,0u);
  EXPECT_EQ(accepted.stamp.epoch,1u);
  Attempt next;ASSERT_TRUE(Prepare(rig,next));
  // The old search inventory used activity1. It cannot masquerade as activity2.
  EXPECT_FALSE(Access::Stage(state,next.prepared,ActivityPlan(0,1,1,1,2,2)));
  ASSERT_TRUE(Stage(rig,state,next,ActivityPlan(0,0,2,1,2,2)));
  ASSERT_TRUE(Good(rig.publication.CommitPhysical(rig.owner,next.token,next.common,Receipt(next))));
  EXPECT_EQ(state.Accepted(rig.owner).selectors.reference_activity_generation,2u);
}
TEST(NativeContactActivityPublicationCuda,LateRejectionPreservesActivityThenExactlyRetries) {
  Rig rig;fe::NativeContactPublicationState state;
  ASSERT_TRUE(Bind(rig,state,true));
  Snapshot before,after;ASSERT_TRUE(rig.Read(before));
  Attempt rejected;ASSERT_TRUE(Prepare(rig,rejected));
  ASSERT_TRUE(Stage(rig,state,rejected,ActivityPlan(1,1,1,1,2,1)));
  EXPECT_NE(rig.publication.CommitPhysical(rig.owner,rejected.token,rejected.common,
      Receipt(rejected,false)).status,fe::ShellPublicationStatus::Success);
  ASSERT_TRUE(rig.Read(after));Exact(before,after);
  EXPECT_FALSE(Access::Pending(state));
  EXPECT_EQ(state.Accepted(rig.owner).selectors.activity,0u);
  EXPECT_EQ(state.Accepted(rig.owner).selectors.activity_generation,1u);
  Attempt retry;ASSERT_TRUE(Prepare(rig,retry));
  ASSERT_TRUE(Stage(rig,state,retry,ActivityPlan(1,1,1,1,2,1)));
  ASSERT_TRUE(Good(rig.publication.CommitPhysical(rig.owner,retry.token,retry.common,Receipt(retry))));
  EXPECT_EQ(state.Accepted(rig.owner).selectors.activity_generation,2u);
}
TEST(NativeContactActivityPublicationCuda,ActivityCannotJumpOverwriteAcceptedSlabsOrChangeLegacyPolicy) {
  Rig rig;fe::NativeContactPublicationState state;
  ASSERT_TRUE(Bind(rig,state,true));
  Attempt a;ASSERT_TRUE(Prepare(rig,a));
  for (const auto bad:{ActivityPlan(1,1,1,2,2,1),ActivityPlan(1,1,1,1,0,1),
       ActivityPlan(1,1,1,0,2,1),ActivityPlan(1,1,1,1,1,1),
       ActivityPlan(1,1,1,1,3,1),ActivityPlan(1,1,1,1,2,0),
       ActivityPlan(1,1,1,1,2,2)}) {
    EXPECT_FALSE(Access::Stage(state,a.prepared,bad));
    EXPECT_FALSE(Access::Pending(state));
  }
  ASSERT_TRUE(Stage(rig,state,a,ActivityPlan(1,1,1,0,1,1)));
  ASSERT_TRUE(Good(rig.publication.CommitPhysical(rig.owner,a.token,a.common,Receipt(a))));
  Attempt reuse;ASSERT_TRUE(Prepare(rig,reuse));
  ASSERT_TRUE(Stage(rig,state,reuse,ActivityPlan(0,1,1,0,1,1)));
  ASSERT_TRUE(Good(rig.publication.CommitPhysical(rig.owner,reuse.token,reuse.common,Receipt(reuse))));
  Rig legacy;fe::NativeContactPublicationState old;
  ASSERT_TRUE(Bind(legacy,old));
  Attempt unchanged;ASSERT_TRUE(Prepare(legacy,unchanged));
  EXPECT_FALSE(Access::Stage(old,unchanged.prepared,ActivityPlan(1,1,1,0,1,0)));
  EXPECT_FALSE(Access::Pending(old));
  legacy.owner.Discard();legacy.publication.DiscardTrial();
}
} // namespace physical_publication_test
