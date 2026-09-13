// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"

namespace physical_publication_test {
namespace {
struct Attempt {
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics materials,common;
  fe::ShellPhysicalScratchParticipationReceipt wall,self;
};
bool Prepare(Rig& rig,Attempt& a,bool record_wall,bool record_self,
             bool seal_wall=true,bool seal_self=true) {
  if(!rig.Begin(a.token,a.assembly)) return false;
  if(record_wall && !Good(rig.mapped_wall_participation.RecordAcceptedAssembly(
      MappedWallSource,rig.owner,a.token,a.assembly))) return false;
  if(record_self && !Good(rig.self_contact_participation.RecordAcceptedAssembly(
      SelfContactSource,rig.owner,a.token,a.assembly))) return false;
  if(!rig.Advance(a.token,a.assembly,a.prepared) ||
      !rig.Evaluate(a.token,a.prepared,a.materials) ||
      !Good(rig.publication.PreparePhysical(rig.owner,a.token,
          {&a.materials.qeph,&a.materials.t3,&a.materials.qbat,
           &a.materials.type25,&a.materials.type13,&a.materials.solids},
          &a.common))) return false;
  if(record_wall && seal_wall &&
      !Good(rig.mapped_wall_participation.SealCandidate(MappedWallSource,
          rig.owner,a.token,a.prepared,&a.wall))) return false;
  if(record_self && seal_self &&
      !Good(rig.self_contact_participation.SealCandidate(SelfContactSource,
          rig.owner,a.token,a.prepared,&a.self))) return false;
  return true;
}
fe::ShellPhysicalScratchReceiptRoster Receipts(
    const Attempt& a,bool wall,bool self) {
  return {wall?&a.wall:nullptr,self?&a.self:nullptr};
}
bool Commit(Rig& rig,Attempt& a,bool wall,bool self) {
  return Good(rig.publication.SealPhysicalScratchParticipation(
             rig.owner,a.token,Receipts(a,wall,self))) &&
      Good(rig.publication.CommitPhysical(rig.owner,a.token,a.common,
          {a.prepared.owner_id,a.prepared.kinematics.base_epoch,
           a.prepared.attempt,Qualification,true}));
}
} // namespace

TEST(PhysicalScratchParticipationCuda,
     AbsentRosterPreservesOldPathAndMappedSlotCoversTwoAttempts) {
  {
    Rig legacy;
    ASSERT_TRUE(legacy.Initialize());
    Attempt a;
    ASSERT_TRUE(Prepare(legacy,a,false,false));
    ASSERT_TRUE(Good(legacy.publication.CommitPhysical(legacy.owner,a.token,a.common,
        {a.prepared.owner_id,a.prepared.kinematics.base_epoch,
         a.prepared.attempt,Qualification,true})));
    EXPECT_EQ(legacy.owner.accepted().epoch,1u);
  }
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  fe::ShellPhysicalScratchRoster roster{
      {&rig.mapped_wall_participation,MappedWallSource},{}};
  fe::ShellPhysicalScratchParticipationForecast forecast;
  ASSERT_TRUE(Good(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(
      roster,{},forecast)));
  auto foreign_identity=rig.fixture.Identity();
  ++foreign_identity.configuration_id;
  EXPECT_EQ(rig.publication.ConfigurePhysicalScratchParticipation(rig.owner,
      rig.fixture.physical,rig.Participants(),foreign_identity,roster).status,
      fe::ShellPublicationStatus::NotJoined);
  EXPECT_FALSE(rig.mapped_wall_participation.configured());
  fe::ShellPhysicalScratchParticipationLimits cap{forecast.total_host_bytes-1};
  EXPECT_EQ(rig.publication.ConfigurePhysicalScratchParticipation(rig.owner,
      rig.fixture.physical,rig.Participants(),rig.fixture.Identity(),roster,cap).status,
      fe::ShellPublicationStatus::ResourceLimit);
  EXPECT_FALSE(rig.mapped_wall_participation.configured());
  ++cap.max_host_bytes;
  ASSERT_TRUE(Good(rig.publication.ConfigurePhysicalScratchParticipation(rig.owner,
      rig.fixture.physical,rig.Participants(),rig.fixture.Identity(),roster,cap)));
  EXPECT_EQ(rig.publication.ConfigurePhysicalScratchParticipation(rig.owner,
      rig.fixture.physical,rig.Participants(),rig.fixture.Identity(),roster,cap).status,
      fe::ShellPublicationStatus::InvalidInput);
  for(std::uint64_t epoch=1;epoch<=2;++epoch) {
    Attempt a;
    ASSERT_TRUE(Prepare(rig,a,true,false));
    EXPECT_TRUE(a.wall.valid());
    EXPECT_EQ(a.wall.kind(),fe::ShellPhysicalScratchContributorKind::MappedWall);
    EXPECT_EQ(a.wall.source_id(),MappedWallSource);
    ASSERT_TRUE(Commit(rig,a,true,false));
    EXPECT_EQ(rig.owner.accepted().epoch,epoch);
    EXPECT_EQ(rig.mapped_wall_participation.generation(),epoch);
  }
}

TEST(PhysicalScratchParticipationCuda,
     PublicSelfContactIssuerCannotClaimTransactionCompletion) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  ASSERT_TRUE(rig.ConfigureScratch(false,true));
  Attempt attempt;
  ASSERT_TRUE(rig.Begin(attempt.token,attempt.assembly));
  EXPECT_EQ(rig.self_contact_participation.RecordAcceptedAssembly(
      SelfContactSource,rig.owner,attempt.token,attempt.assembly).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  EXPECT_EQ(rig.owner.accepted().epoch,0u);
  EXPECT_EQ(rig.self_contact_participation.generation(),0u);
}

TEST(PhysicalScratchParticipationCuda,
     MappedSlotRejectsMissingDuplicateAndCandidateWithoutAssemblyThenRetry) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  ASSERT_TRUE(rig.ConfigureScratch(true,false));
  Snapshot before,after;
  ASSERT_TRUE(rig.Read(before));
  Attempt unsealed;
  ASSERT_TRUE(Prepare(rig,unsealed,true,false));
  EXPECT_EQ(rig.publication.CommitPhysical(rig.owner,unsealed.token,unsealed.common,
      {unsealed.prepared.owner_id,unsealed.prepared.kinematics.base_epoch,
       unsealed.prepared.attempt,Qualification,true}).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  ASSERT_TRUE(rig.Read(after)); Exact(before,after);

  Attempt missing;
  ASSERT_TRUE(Prepare(rig,missing,false,false));
  EXPECT_EQ(rig.publication.SealPhysicalScratchParticipation(
      rig.owner,missing.token,Receipts(missing,false,false)).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  ASSERT_TRUE(rig.Read(after)); Exact(before,after);

  Attempt duplicate;
  ASSERT_TRUE(rig.Begin(duplicate.token,duplicate.assembly));
  ASSERT_TRUE(Good(rig.mapped_wall_participation.RecordAcceptedAssembly(
      MappedWallSource,rig.owner,duplicate.token,duplicate.assembly)));
  EXPECT_EQ(rig.mapped_wall_participation.RecordAcceptedAssembly(
      MappedWallSource,rig.owner,duplicate.token,duplicate.assembly).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  ASSERT_TRUE(rig.Read(after)); Exact(before,after);

  Attempt skipped;
  ASSERT_TRUE(Prepare(rig,skipped,false,false));
  EXPECT_EQ(rig.mapped_wall_participation.SealCandidate(MappedWallSource,
      rig.owner,skipped.token,skipped.prepared,&skipped.wall).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  ASSERT_TRUE(rig.Read(after)); Exact(before,after);

  Attempt retry;
  ASSERT_TRUE(Prepare(rig,retry,true,false));
  ASSERT_TRUE(Commit(rig,retry,true,false));
  EXPECT_EQ(rig.owner.accepted().epoch,1u);
}

TEST(PhysicalScratchParticipationCuda,
     WrongSourceStreamOwnerTokenAndForeignReceiptRevokeWithoutAcceptedChange) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  ASSERT_TRUE(rig.ConfigureScratch(true,false));
  Snapshot before,after;
  ASSERT_TRUE(rig.Read(before));
  Attempt wrong_source;
  ASSERT_TRUE(rig.Begin(wrong_source.token,wrong_source.assembly));
  EXPECT_EQ(rig.mapped_wall_participation.RecordAcceptedAssembly(
      MappedWallSource+1,rig.owner,wrong_source.token,wrong_source.assembly).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  ASSERT_TRUE(rig.Read(after)); Exact(before,after);

  Attempt wrong_stream;
  ASSERT_TRUE(rig.Begin(wrong_stream.token,wrong_stream.assembly));
  auto altered=wrong_stream.assembly;
  altered.stream=nullptr;
  EXPECT_EQ(rig.mapped_wall_participation.RecordAcceptedAssembly(
      MappedWallSource,rig.owner,wrong_stream.token,altered).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  ASSERT_TRUE(rig.Read(after)); Exact(before,after);

  Attempt wrong_token;
  ASSERT_TRUE(rig.Begin(wrong_token.token,wrong_token.assembly));
  EXPECT_EQ(rig.mapped_wall_participation.RecordAcceptedAssembly(
      MappedWallSource,rig.owner,wrong_stream.token,wrong_token.assembly).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  ASSERT_TRUE(rig.Read(after)); Exact(before,after);

  Attempt prior_assembly;
  ASSERT_TRUE(rig.Begin(prior_assembly.token,prior_assembly.assembly));
  EXPECT_EQ(rig.mapped_wall_participation.RecordAcceptedAssembly(
      MappedWallSource,rig.owner,prior_assembly.token,wrong_token.assembly).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  ASSERT_TRUE(rig.Read(after)); Exact(before,after);

  Rig foreign;
  ASSERT_TRUE(foreign.Initialize());
  ASSERT_TRUE(foreign.ConfigureScratch(true,false));
  Attempt other;
  ASSERT_TRUE(foreign.Begin(other.token,other.assembly));
  EXPECT_EQ(rig.mapped_wall_participation.RecordAcceptedAssembly(
      MappedWallSource,foreign.owner,other.token,other.assembly).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  foreign.owner.Discard(); foreign.publication.DiscardTrial();
  ASSERT_TRUE(rig.Read(after)); Exact(before,after);

  Attempt local,foreign_attempt;
  ASSERT_TRUE(Prepare(rig,local,true,false));
  ASSERT_TRUE(Prepare(foreign,foreign_attempt,true,false));
  EXPECT_EQ(rig.publication.SealPhysicalScratchParticipation(rig.owner,local.token,
      {nullptr,&foreign_attempt.wall}).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  foreign.owner.Discard(); foreign.publication.DiscardTrial();
  ASSERT_TRUE(rig.Read(after)); Exact(before,after);

  ASSERT_TRUE(Prepare(rig,local,true,false));
  ASSERT_TRUE(Prepare(foreign,foreign_attempt,true,false));
  EXPECT_EQ(rig.publication.SealPhysicalScratchParticipation(rig.owner,local.token,
      {&foreign_attempt.wall,nullptr}).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  foreign.owner.Discard(); foreign.publication.DiscardTrial();
  ASSERT_TRUE(rig.Read(after)); Exact(before,after);
}

TEST(PhysicalScratchParticipationCuda,
     DuplicateSealReplayAndLateValidationFailurePreserveEverythingThenFreshCommit) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  ASSERT_TRUE(rig.ConfigureScratch(true,false));
  Snapshot before,after;
  ASSERT_TRUE(rig.Read(before));
  Attempt structural;
  ASSERT_TRUE(rig.Begin(structural.token,structural.assembly));
  ASSERT_TRUE(Good(rig.mapped_wall_participation.RecordAcceptedAssembly(
      MappedWallSource,rig.owner,structural.token,structural.assembly)));
  ASSERT_TRUE(rig.Advance(structural.token,structural.assembly,structural.prepared));
  ASSERT_TRUE(rig.Evaluate(structural.token,structural.prepared,
      structural.materials,false));
  EXPECT_NE(rig.publication.PreparePhysical(rig.owner,structural.token,
      {&structural.materials.qeph,&structural.materials.t3,
       &structural.materials.qbat,&structural.materials.type25,
       &structural.materials.type13,nullptr},&structural.common).status,
      fe::ShellPublicationStatus::Success);
  ASSERT_TRUE(rig.Read(after)); Exact(before,after);

  Attempt duplicate_candidate;
  ASSERT_TRUE(Prepare(rig,duplicate_candidate,true,false));
  EXPECT_EQ(rig.mapped_wall_participation.SealCandidate(MappedWallSource,
      rig.owner,duplicate_candidate.token,duplicate_candidate.prepared,
      &duplicate_candidate.wall).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  ASSERT_TRUE(rig.Read(after)); Exact(before,after);

  Attempt duplicate_receipt;
  ASSERT_TRUE(Prepare(rig,duplicate_receipt,true,false));
  EXPECT_EQ(rig.publication.SealPhysicalScratchParticipation(
      rig.owner,duplicate_receipt.token,
      {&duplicate_receipt.wall,&duplicate_receipt.wall}).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  ASSERT_TRUE(rig.Read(after)); Exact(before,after);

  Attempt duplicate_roster;
  ASSERT_TRUE(Prepare(rig,duplicate_roster,true,false));
  ASSERT_TRUE(Good(rig.publication.SealPhysicalScratchParticipation(
      rig.owner,duplicate_roster.token,Receipts(duplicate_roster,true,false))));
  EXPECT_EQ(rig.publication.SealPhysicalScratchParticipation(
      rig.owner,duplicate_roster.token,Receipts(duplicate_roster,true,false)).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  ASSERT_TRUE(rig.Read(after)); Exact(before,after);

  Attempt old;
  ASSERT_TRUE(Prepare(rig,old,true,false));
  ASSERT_TRUE(Good(rig.publication.SealPhysicalScratchParticipation(
      rig.owner,old.token,Receipts(old,true,false))));
  // Existing capture-receipt ordering remains ahead of participation commit:
  // this is the old StaleTrial result, not a replacement participation status.
  EXPECT_EQ(rig.publication.CommitPhysical(rig.owner,old.token,old.common,
      {old.prepared.owner_id,old.prepared.kinematics.base_epoch,
       old.prepared.attempt,Qualification,false}).status,
      fe::ShellPublicationStatus::StaleTrial);
  ASSERT_TRUE(rig.Read(after)); Exact(before,after);

  Attempt replay;
  ASSERT_TRUE(Prepare(rig,replay,true,false));
  EXPECT_EQ(rig.publication.SealPhysicalScratchParticipation(
      rig.owner,replay.token,Receipts(old,true,false)).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  ASSERT_TRUE(rig.Read(after)); Exact(before,after);

  Attempt retry;
  ASSERT_TRUE(Prepare(rig,retry,true,false));
  ASSERT_TRUE(Commit(rig,retry,true,false));
  EXPECT_EQ(rig.owner.accepted().epoch,1u);
}

} // namespace physical_publication_test
