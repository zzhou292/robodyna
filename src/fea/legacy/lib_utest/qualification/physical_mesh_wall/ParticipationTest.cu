// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <new>
#include <string>

namespace physical_wall_test {
namespace {
struct Attempt {
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics materials,common;
  c::NodalWallMappedDiagnostics base,candidate;
  c::NodalWallMappedTransactionReceipt receipt;
};
struct Transaction {
  p::Rig rig{true};
  std::unique_ptr<Geometry> geometry;
  c::NodalWallMappedContact wall;

  bool Initialize(bool configure=true) {
    if(!rig.Initialize()) return false;
    geometry=std::make_unique<Geometry>(rig.fixture);
    if(!Good(wall.Initialize(Config(rig),geometry->Wall(),geometry->weights,
                            Source(rig),rig.owner,geometry->Motion())))
      return false;
    if(!configure) return true;
    const auto entry=wall.roster_entry();
    if(!entry.issuer || entry.source_id!=Config(rig).wall_binding_id)
      return false;
    fe::ShellPhysicalScratchParticipationForecast forecast;
    if(!p::Good(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(
            {entry,{}},{},forecast)))
      return false;
    auto rejected=forecast;
    fe::ShellPhysicalScratchParticipationLimits short_cap{
        forecast.total_host_bytes-1};
    EXPECT_EQ(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(
        {entry,{}},short_cap,rejected).status,
        fe::ShellPublicationStatus::ResourceLimit);
    EXPECT_EQ(rejected.total_host_bytes,forecast.total_host_bytes);
    fe::ShellPhysicalScratchParticipationLimits cap{
        forecast.total_host_bytes};
    return p::Good(rig.publication.ConfigurePhysicalScratchParticipation(
        rig.owner,rig.fixture.physical,rig.Participants(),
        rig.fixture.Identity(),{entry,{}},cap));
  }
  bool Prepare(Attempt& attempt) {
    return rig.Begin(attempt.token,attempt.assembly) &&
        Good(wall.AssembleAccepted(rig.owner,attempt.token,
                                  attempt.assembly,&attempt.base)) &&
        rig.Advance(attempt.token,attempt.assembly,attempt.prepared) &&
        rig.Evaluate(attempt.token,attempt.prepared,attempt.materials) &&
        p::Good(rig.publication.PreparePhysical(
            rig.owner,attempt.token,
            {&attempt.materials.qeph,&attempt.materials.t3,
             &attempt.materials.qbat,&attempt.materials.type25,
             &attempt.materials.type13,&attempt.materials.solids},
            &attempt.common));
  }
  bool Complete(Attempt& attempt) {
    return Prepare(attempt) &&
        Good(wall.EvaluateCandidate(
            rig.owner,attempt.token,attempt.prepared,attempt.common,
            &attempt.candidate,&attempt.receipt));
  }
  bool Commit(Attempt& attempt) {
    return p::Good(rig.publication.SealPhysicalScratchParticipation(
               rig.owner,attempt.token,attempt.receipt.scratch_receipts())) &&
        p::Good(rig.publication.CommitPhysical(
            rig.owner,attempt.token,attempt.common,
            {attempt.prepared.owner_id,
             attempt.prepared.kinematics.base_epoch,
             attempt.prepared.attempt,p::Qualification,true}));
  }
  void Discard() {
    rig.owner.Discard();
    rig.publication.DiscardTrial();
    wall.DiscardTrial();
  }
};
} // namespace

TEST(PhysicalWallParticipationCuda,
     LegacyNoRosterIsUnchangedAndWallOnlyReceiptIsMandatory) {
  {
    Transaction legacy;
    ASSERT_TRUE(legacy.Initialize(false));
    Attempt attempt;
    ASSERT_TRUE(legacy.Prepare(attempt));
    ASSERT_TRUE(Good(legacy.wall.EvaluateCandidate(
        legacy.rig.owner,attempt.token,attempt.prepared,attempt.common,
        &attempt.candidate)));
    ASSERT_TRUE(p::Good(legacy.rig.publication.CommitPhysical(
        legacy.rig.owner,attempt.token,attempt.common,
        {attempt.prepared.owner_id,attempt.prepared.kinematics.base_epoch,
         attempt.prepared.attempt,p::Qualification,true})));
    EXPECT_EQ(legacy.rig.owner.accepted().epoch,1u);
  }
  Transaction transaction;
  ASSERT_TRUE(transaction.Initialize());
  const auto allocation=transaction.wall.allocations();
  Attempt missing;
  ASSERT_TRUE(transaction.Complete(missing));
  EXPECT_TRUE(missing.receipt.valid());
  EXPECT_EQ(missing.receipt.wall_binding_id(),Config(transaction.rig).wall_binding_id);
  EXPECT_EQ(transaction.rig.publication.CommitPhysical(
      transaction.rig.owner,missing.token,missing.common,
      {missing.prepared.owner_id,missing.prepared.kinematics.base_epoch,
       missing.prepared.attempt,p::Qualification,true}).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  transaction.wall.DiscardTrial();

  Attempt retry;
  ASSERT_TRUE(transaction.Complete(retry));
  ASSERT_TRUE(transaction.Commit(retry));
  EXPECT_EQ(transaction.rig.owner.accepted().epoch,1u);
  EXPECT_EQ(transaction.wall.allocations().device_bytes,
            allocation.device_bytes);
  EXPECT_EQ(transaction.wall.allocations().device_allocations,
            allocation.device_allocations);
}

TEST(PhysicalWallParticipationCuda,
     MissingDuplicateStaleForeignAndDuplicateSealRollbackThenRetry) {
  Transaction local,foreign;
  ASSERT_TRUE(local.Initialize());
  ASSERT_TRUE(foreign.Initialize());
  p::Snapshot before,after;
  ASSERT_TRUE(local.rig.Read(before));

  Attempt duplicate_record;
  ASSERT_TRUE(local.rig.Begin(
      duplicate_record.token,duplicate_record.assembly));
  ASSERT_TRUE(Good(local.wall.AssembleAccepted(
      local.rig.owner,duplicate_record.token,duplicate_record.assembly,
      &duplicate_record.base)));
  c::NodalWallMappedDiagnostics unchanged;
  unchanged.accepted_active_parents=619;
  EXPECT_NE(local.wall.AssembleAccepted(
      local.rig.owner,duplicate_record.token,duplicate_record.assembly,
      &unchanged).status,c::NodalWallDeviceStatus::Ok);
  EXPECT_EQ(unchanged.accepted_active_parents,619u);

  Attempt missing;
  ASSERT_TRUE(local.Complete(missing));
  EXPECT_EQ(local.rig.publication.SealPhysicalScratchParticipation(
      local.rig.owner,missing.token,{}).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  local.wall.DiscardTrial();
  ASSERT_TRUE(local.rig.Read(after));
  p::Exact(before,after);

  Attempt duplicate;
  ASSERT_TRUE(local.Complete(duplicate));
  auto repeated=duplicate.receipt.scratch_receipts();
  repeated.self_contact=repeated.mapped_wall;
  EXPECT_EQ(local.rig.publication.SealPhysicalScratchParticipation(
      local.rig.owner,duplicate.token,repeated).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  local.wall.DiscardTrial();

  Attempt stale;
  ASSERT_TRUE(local.Complete(stale));
  const auto stale_receipt=stale.receipt;
  local.Discard();
  Attempt current;
  ASSERT_TRUE(local.Complete(current));
  EXPECT_EQ(local.rig.publication.SealPhysicalScratchParticipation(
      local.rig.owner,current.token,stale_receipt.scratch_receipts()).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  local.wall.DiscardTrial();

  Attempt local_attempt,foreign_attempt;
  ASSERT_TRUE(local.Complete(local_attempt));
  ASSERT_TRUE(foreign.Complete(foreign_attempt));
  EXPECT_EQ(local.rig.publication.SealPhysicalScratchParticipation(
      local.rig.owner,local_attempt.token,
      foreign_attempt.receipt.scratch_receipts()).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  local.wall.DiscardTrial();
  foreign.Discard();

  Attempt duplicate_seal;
  ASSERT_TRUE(local.Complete(duplicate_seal));
  ASSERT_TRUE(p::Good(local.rig.publication.SealPhysicalScratchParticipation(
      local.rig.owner,duplicate_seal.token,
      duplicate_seal.receipt.scratch_receipts())));
  EXPECT_EQ(local.rig.publication.SealPhysicalScratchParticipation(
      local.rig.owner,duplicate_seal.token,
      duplicate_seal.receipt.scratch_receipts()).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  local.wall.DiscardTrial();

  Attempt retry;
  ASSERT_TRUE(local.Complete(retry));
  ASSERT_TRUE(local.Commit(retry));
  EXPECT_EQ(local.rig.owner.accepted().epoch,1u);
}

TEST(PhysicalWallParticipationCuda,
     LateWallCandidateFailureAndMissingReceiptOutputRollbackAtomically) {
  Transaction transaction;
  ASSERT_TRUE(transaction.Initialize());
  p::Snapshot before,after;
  ASSERT_TRUE(transaction.rig.Read(before));

  Attempt late;
  ASSERT_TRUE(transaction.Prepare(late));
  auto invalid=late.common;
  invalid.valid=false;
  late.candidate.accepted_active_parents=777;
  const auto unchanged_candidate=late.candidate;
  c::NodalWallMappedTransactionReceipt unchanged_receipt;
  EXPECT_NE(transaction.wall.EvaluateCandidate(
      transaction.rig.owner,late.token,late.prepared,invalid,
      &late.candidate,&unchanged_receipt).status,c::NodalWallDeviceStatus::Ok);
  EXPECT_EQ(late.candidate.accepted_active_parents,
            unchanged_candidate.accepted_active_parents);
  EXPECT_FALSE(unchanged_receipt.valid());
  ASSERT_TRUE(transaction.rig.Read(after));
  p::Exact(before,after);

  Attempt no_output;
  ASSERT_TRUE(transaction.Prepare(no_output));
  no_output.candidate.accepted_active_parents=991;
  EXPECT_EQ(transaction.wall.EvaluateCandidate(
      transaction.rig.owner,no_output.token,no_output.prepared,
      no_output.common,&no_output.candidate,nullptr).status,
      c::NodalWallDeviceStatus::InvalidInput);
  EXPECT_EQ(no_output.candidate.accepted_active_parents,991u);

  Attempt retry;
  ASSERT_TRUE(transaction.Complete(retry));
  ASSERT_TRUE(transaction.Commit(retry));
}

TEST(PhysicalWallParticipationCuda,
     WrongRosterKindRejectsOnlyAfterWallAssemblyAndLeavesOutputUnchanged) {
  p::Rig rig(true);
  ASSERT_TRUE(rig.Initialize());
  Geometry geometry(rig.fixture);
  c::NodalWallMappedContact wall;
  ASSERT_TRUE(Good(wall.Initialize(
      Config(rig),geometry.Wall(),geometry.weights,Source(rig),
      rig.owner,geometry.Motion())));
  ASSERT_TRUE(p::Good(rig.publication.ConfigurePhysicalScratchParticipation(
      rig.owner,rig.fixture.physical,rig.Participants(),rig.fixture.Identity(),
      {{},wall.roster_entry()})));
  p::Snapshot before,after;
  ASSERT_TRUE(rig.Read(before));
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(rig.Begin(token,assembly));
  c::NodalWallMappedDiagnostics unchanged;
  unchanged.accepted_active_parents=313;
  EXPECT_EQ(wall.AssembleAccepted(
      rig.owner,token,assembly,&unchanged).status,
      c::NodalWallDeviceStatus::ParticipationFailure);
  EXPECT_EQ(unchanged.accepted_active_parents,313u);
  ASSERT_TRUE(rig.Read(after));
  p::Exact(before,after);
}

TEST(PhysicalWallParticipationCuda,
     ActualWallIsFirstAndPlacementNewAbaAndDestructionOrdersReject) {
  using Wall=c::NodalWallMappedContact;
  p::Rig rig(true);
  ASSERT_TRUE(rig.Initialize());
  Geometry geometry(rig.fixture);
  alignas(Wall) std::byte storage[sizeof(Wall)];
  auto* first=new(storage) Wall;
  ASSERT_TRUE(Good(first->Initialize(
      Config(rig),geometry.Wall(),geometry.weights,Source(rig),
      rig.owner,geometry.Motion())));
  ASSERT_TRUE(p::Good(rig.publication.ConfigurePhysicalScratchParticipation(
      rig.owner,rig.fixture.physical,rig.Participants(),rig.fixture.Identity(),
      {first->roster_entry(),
       {&rig.self_contact_participation,p::SelfContactSource}})));

  Attempt attempt;
  ASSERT_TRUE(rig.Begin(attempt.token,attempt.assembly));
  ASSERT_TRUE(Good(first->AssembleAccepted(
      rig.owner,attempt.token,attempt.assembly,&attempt.base)));
  ASSERT_TRUE(rig.Advance(attempt.token,attempt.assembly,attempt.prepared));
  ASSERT_TRUE(rig.Evaluate(attempt.token,attempt.prepared,attempt.materials));
  ASSERT_TRUE(p::Good(rig.publication.PreparePhysical(
      rig.owner,attempt.token,
      {&attempt.materials.qeph,&attempt.materials.t3,
       &attempt.materials.qbat,&attempt.materials.type25,
       &attempt.materials.type13,&attempt.materials.solids},
      &attempt.common)));
  ASSERT_TRUE(Good(first->EvaluateCandidate(
      rig.owner,attempt.token,attempt.prepared,attempt.common,
      &attempt.candidate,&attempt.receipt)));
  const auto missing=rig.publication.SealPhysicalScratchParticipation(
      rig.owner,attempt.token,{});
  EXPECT_EQ(missing.status,fe::ShellPublicationStatus::ParticipationFailure);
  EXPECT_NE(std::string(missing.message).find("MappedWall"),std::string::npos);
  first->DiscardTrial();

  ASSERT_TRUE(rig.Begin(attempt.token,attempt.assembly));
  ASSERT_TRUE(Good(first->AssembleAccepted(
      rig.owner,attempt.token,attempt.assembly,&attempt.base)));
  ASSERT_TRUE(rig.Advance(attempt.token,attempt.assembly,attempt.prepared));
  ASSERT_TRUE(rig.Evaluate(attempt.token,attempt.prepared,attempt.materials));
  ASSERT_TRUE(p::Good(rig.publication.PreparePhysical(
      rig.owner,attempt.token,
      {&attempt.materials.qeph,&attempt.materials.t3,
       &attempt.materials.qbat,&attempt.materials.type25,
       &attempt.materials.type13,&attempt.materials.solids},
      &attempt.common)));
  ASSERT_TRUE(Good(first->EvaluateCandidate(
      rig.owner,attempt.token,attempt.prepared,attempt.common,
      &attempt.candidate,&attempt.receipt)));
  const auto stale=attempt.receipt;
  first->~Wall();
  auto* replacement=new(storage) Wall;
  ASSERT_TRUE(Good(replacement->Initialize(
      Config(rig),geometry.Wall(),geometry.weights,Source(rig),
      rig.owner,geometry.Motion())));
  EXPECT_EQ(rig.publication.SealPhysicalScratchParticipation(
      rig.owner,attempt.token,stale.scratch_receipts()).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  replacement->~Wall();

  {
    p::Rig publisher_first(true);
    ASSERT_TRUE(publisher_first.Initialize());
    Geometry second_geometry(publisher_first.fixture);
    Wall publisher_last;
    ASSERT_TRUE(Good(publisher_last.Initialize(
        Config(publisher_first),second_geometry.Wall(),
        second_geometry.weights,Source(publisher_first),
        publisher_first.owner,second_geometry.Motion())));
    ASSERT_TRUE(p::Good(
        publisher_first.publication.ConfigurePhysicalScratchParticipation(
            publisher_first.owner,publisher_first.fixture.physical,
            publisher_first.Participants(),publisher_first.fixture.Identity(),
            {publisher_last.roster_entry(),{}})));
    publisher_first.publication.~ShellBatchPublication();
    publisher_last.DiscardTrial();
    new(&publisher_first.publication) fe::ShellBatchPublication;
  }
}

} // namespace physical_wall_test
