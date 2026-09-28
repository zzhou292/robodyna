// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeContactFixture.h"
namespace physical_publication_test {
using namespace native_contact_test;
TEST(NativeContactPublicationCuda,CommonCommitPublishesHistoryAndReferenceTogether) {
  Rig rig;fe::NativeContactPublicationState state;ASSERT_TRUE(Bind(rig,state));
  RecordProperty("native_publication_state_host_bytes",std::to_string(sizeof(state)));
  auto initial=state.Accepted(rig.owner);ASSERT_TRUE(initial.available);EXPECT_EQ(initial.generation,0u);
  EXPECT_FALSE(initial.selectors.has_reference);EXPECT_FALSE(initial.force_phase_available);
  const fe::NativeContactSelectors plans[]{{1,1,1,true},{0,1,1,true},{1,0,2,true}};
  for(unsigned step=0;step<3;++step) {
    const auto force_base=rig.owner.accepted();
    Attempt a;ASSERT_TRUE(Prepare(rig,a));ASSERT_TRUE(Stage(rig,state,a,plans[step]));
    EXPECT_EQ(state.Accepted(rig.owner).generation,step);EXPECT_EQ(rig.owner.accepted().epoch,step);
    ASSERT_TRUE(Good(rig.publication.CommitPhysical(rig.owner,a.token,a.common,Receipt(a))));
    const auto accepted=state.Accepted(rig.owner);ASSERT_TRUE(accepted.available);
    EXPECT_TRUE(fe::trial_identity::SameStamp(accepted.stamp,rig.owner.accepted()));
    ASSERT_TRUE(accepted.force_phase_available);
    EXPECT_TRUE(fe::trial_identity::SameStamp(accepted.force_base_stamp,force_base));
    EXPECT_EQ(accepted.force_base_stamp.epoch+1,accepted.stamp.epoch);
    EXPECT_EQ(accepted.generation,step+1);EXPECT_EQ(accepted.selectors.history,plans[step].history);
    EXPECT_EQ(accepted.selectors.reference,plans[step].reference);
    EXPECT_EQ(accepted.selectors.reference_generation,plans[step].reference_generation);
  }
}
TEST(NativeContactPublicationCuda,CommonRejectionAndRetryPreserveAcceptedSelectors) {
  Rig rig;fe::NativeContactPublicationState state;ASSERT_TRUE(Bind(rig,state));Snapshot before,after;
  ASSERT_TRUE(rig.Read(before));Attempt rejected;ASSERT_TRUE(Prepare(rig,rejected));
  ASSERT_TRUE(Stage(rig,state,rejected,{1,1,1,true}));
  EXPECT_NE(rig.publication.CommitPhysical(rig.owner,rejected.token,rejected.common,Receipt(rejected,false)).status,
      fe::ShellPublicationStatus::Success);
  ASSERT_TRUE(rig.Read(after));Exact(before,after);EXPECT_FALSE(Access::Pending(state));
  auto unchanged=state.Accepted(rig.owner);ASSERT_TRUE(unchanged.available);EXPECT_EQ(unchanged.generation,0u);
  EXPECT_EQ(unchanged.selectors.history,0u);EXPECT_FALSE(unchanged.selectors.has_reference);
  Attempt retried;ASSERT_TRUE(Prepare(rig,retried));ASSERT_TRUE(Stage(rig,state,retried,{1,1,1,true}));
  ASSERT_TRUE(Good(rig.publication.CommitPhysical(rig.owner,retried.token,retried.common,Receipt(retried))));
  EXPECT_EQ(state.Accepted(rig.owner).generation,1u);
}
TEST(NativeContactPublicationCuda,MissingPlanAndLateParticipantMismatchCannotCommit) {
  Rig rig;fe::NativeContactPublicationState state;ASSERT_TRUE(Bind(rig,state));Attempt missing;ASSERT_TRUE(Prepare(rig,missing));
  EXPECT_EQ(Access::Seal(rig.self_contact_participation,rig.owner,missing.token,missing.prepared,&missing.native).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  EXPECT_EQ(rig.owner.accepted().epoch,0u);EXPECT_EQ(state.Accepted(rig.owner).generation,0u);
  Attempt late;ASSERT_TRUE(Prepare(rig,late));ASSERT_TRUE(Stage(rig,state,late,{1,1,1,true}));
  auto changed=late.common;changed.base_stamp.epoch+=1;
  EXPECT_NE(rig.publication.CommitPhysical(rig.owner,late.token,changed,Receipt(late)).status,fe::ShellPublicationStatus::Success);
  EXPECT_EQ(rig.owner.accepted().epoch,0u);EXPECT_FALSE(Access::Pending(state));
  EXPECT_EQ(state.Accepted(rig.owner).generation,0u);
}
TEST(NativeContactPublicationCuda,ConsumedIssuerStillHasPrivatePlanUntilSuccessOrDiscard) {
  Rig rig;fe::NativeContactPublicationState state;ASSERT_TRUE(Bind(rig,state));Attempt a;
  ASSERT_TRUE(Prepare(rig,a));ASSERT_TRUE(Stage(rig,state,a,{1,1,1,true}));
  Access::Consume(rig.self_contact_participation);EXPECT_TRUE(Access::Pending(state));
  EXPECT_EQ(state.Accepted(rig.owner).generation,0u);
  rig.owner.Discard();rig.publication.DiscardTrial();EXPECT_FALSE(Access::Pending(state));
  EXPECT_EQ(state.Accepted(rig.owner).generation,0u);EXPECT_EQ(rig.owner.accepted().epoch,0u);
}
TEST(NativeContactPublicationCuda,SelectorAndGenerationAdmissionIsBounded) {
  Rig rig;fe::NativeContactPublicationState state;ASSERT_TRUE(Bind(rig,state));Attempt a;ASSERT_TRUE(Prepare(rig,a));
  EXPECT_FALSE(Access::Stage(state,a.prepared,{0,1,1,true})); // accepted history slab cannot be overwritten
  EXPECT_FALSE(Access::Stage(state,a.prepared,{1,2,1,true}));
  EXPECT_FALSE(Access::Stage(state,a.prepared,{1,1,0,true}));
  EXPECT_FALSE(Access::Stage(state,a.prepared,{1,1,1,false}));
  Access::Exhaust(state);EXPECT_FALSE(Access::Stage(state,a.prepared,{1,1,1,true}));
  EXPECT_EQ(rig.owner.accepted().epoch,0u);rig.owner.Discard();rig.publication.DiscardTrial();
}
TEST(NativeContactPublicationCuda,CompleteAcceptedStampMustMatch) {
  Rig rig;fe::NativeContactPublicationState state;ASSERT_TRUE(Bind(rig,state));
  Access::CorruptDt(state);EXPECT_FALSE(state.Accepted(rig.owner).available);
  Attempt a;ASSERT_TRUE(Prepare(rig,a));EXPECT_TRUE(Access::Stage(state,a.prepared,{1,1,1,true}));
  EXPECT_EQ(Access::Seal(rig.self_contact_participation,rig.owner,a.token,a.prepared,&a.native).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  EXPECT_EQ(rig.owner.accepted().epoch,0u);
}
TEST(NativeContactPublicationCuda,BothLifetimeOrdersRevokeNativeRegistration) {
  {
    Rig rig;ASSERT_TRUE(rig.Initialize());
    alignas(fe::NativeContactPublicationState) std::byte bytes[sizeof(fe::NativeContactPublicationState)];
    auto* first=new(bytes) fe::NativeContactPublicationState;
    ASSERT_TRUE(Access::Attach(*first,rig.owner,SelfContactSource,rig.self_contact_participation));
    ASSERT_TRUE(rig.ConfigureScratch(false,true));first->~NativeContactPublicationState();
    EXPECT_FALSE(rig.self_contact_participation.configured());
    auto* replacement=new(bytes) fe::NativeContactPublicationState;
    EXPECT_FALSE(replacement->Accepted(rig.owner).available);replacement->~NativeContactPublicationState();
  }
  {
    Rig rig;fe::NativeContactPublicationState state;ASSERT_TRUE(Bind(rig,state));
    rig.self_contact_participation.~ShellPhysicalScratchParticipation();
    EXPECT_FALSE(state.Accepted(rig.owner).available);
    new(&rig.self_contact_participation) fe::ShellPhysicalScratchParticipation;
    EXPECT_FALSE(rig.self_contact_participation.configured());
  }
}
TEST(NativeContactPublicationCuda,RawExternalOwnerAdvanceCannotExposeOldNativeStateAsCurrent) {
  Rig rig;fe::NativeContactPublicationState state;ASSERT_TRUE(Bind(rig,state));
  fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView prepared;
  ASSERT_TRUE(rig.Begin(token,assembly));ASSERT_TRUE(rig.Advance(token,assembly,prepared));
  ASSERT_TRUE(Good(fe::CompleteNodalValidation(rig.owner,token,
      {prepared.owner_id,prepared.kinematics.base_epoch,prepared.attempt,Qualification,true})));
  ASSERT_TRUE(Good(rig.owner.Commit(token)));EXPECT_EQ(rig.owner.accepted().epoch,1u);
  EXPECT_FALSE(state.Accepted(rig.owner).available);
}
TEST(NativeContactPublicationCuda,SourceOwnerAndFixedRosterKindCannotBeSubstituted) {
  Rig rig,other;fe::NativeContactPublicationState state;ASSERT_TRUE(rig.Initialize());ASSERT_TRUE(other.Initialize());
  ASSERT_TRUE(Access::Attach(state,rig.owner,SelfContactSource,rig.self_contact_participation));
  EXPECT_EQ(other.publication.ConfigurePhysicalScratchParticipation(other.owner,other.fixture.physical,
      other.Participants(),other.fixture.Identity(),{{},{&rig.self_contact_participation,SelfContactSource}}).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  EXPECT_EQ(rig.publication.ConfigurePhysicalScratchParticipation(rig.owner,rig.fixture.physical,
      rig.Participants(),rig.fixture.Identity(),{{},{&rig.self_contact_participation,SelfContactSource+1}}).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  EXPECT_EQ(rig.publication.ConfigurePhysicalScratchParticipation(rig.owner,rig.fixture.physical,
      rig.Participants(),rig.fixture.Identity(),{{&rig.self_contact_participation,SelfContactSource},{}}).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  EXPECT_FALSE(rig.self_contact_participation.configured());ASSERT_TRUE(rig.ConfigureScratch(false,true));
}
} // namespace physical_publication_test
