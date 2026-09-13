// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <new>
#include <string>

namespace physical_publication_test {
namespace {
struct Attempt {
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  fe::NodalPreparedView prepared;
  fe::ShellPhysicalDiagnostics materials,common;
};
bool Prepare(Rig& rig,Attempt& a) {
  return rig.Begin(a.token,a.assembly) &&
      rig.Advance(a.token,a.assembly,a.prepared) &&
      rig.Evaluate(a.token,a.prepared,a.materials) &&
      Good(rig.publication.PreparePhysical(rig.owner,a.token,
          {&a.materials.qeph,&a.materials.t3,&a.materials.qbat,
           &a.materials.type25,&a.materials.type13,&a.materials.solids},
          &a.common));
}
} // namespace

TEST(PhysicalScratchParticipationCuda,
     AbsentRosterPreservesExactLegacyCommitPath) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  Attempt attempt;
  ASSERT_TRUE(Prepare(rig,attempt));
  ASSERT_TRUE(Good(rig.publication.CommitPhysical(
      rig.owner,attempt.token,attempt.common,
      {attempt.prepared.owner_id,attempt.prepared.kinematics.base_epoch,
       attempt.prepared.attempt,Qualification,true})));
  EXPECT_EQ(rig.owner.accepted().epoch,1u);
}

TEST(PhysicalScratchParticipationCuda,
     BothPublicIssuerKindsRejectDirectSelfAttestationAndLeaveOutputs) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  ASSERT_TRUE(rig.ConfigureScratch(true,true));
  fe::NodalTrialToken token;
  fe::NodalAssemblyView assembly;
  ASSERT_TRUE(rig.Begin(token,assembly));
  EXPECT_EQ(rig.mapped_wall_participation.RecordAcceptedAssembly(
      MappedWallSource,rig.owner,token,assembly).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  EXPECT_EQ(rig.mapped_wall_participation.generation(),0u);

  ASSERT_TRUE(rig.Begin(token,assembly));
  EXPECT_EQ(rig.self_contact_participation.RecordAcceptedAssembly(
      SelfContactSource,rig.owner,token,assembly).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  EXPECT_EQ(rig.self_contact_participation.generation(),0u);

  ASSERT_TRUE(rig.Begin(token,assembly));
  fe::ShellPhysicalScratchParticipationReceipt unchanged;
  EXPECT_EQ(rig.mapped_wall_participation.SealCandidate(
      MappedWallSource,rig.owner,token,{},&unchanged).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  EXPECT_FALSE(unchanged.valid());
  ASSERT_TRUE(rig.Begin(token,assembly));
  EXPECT_EQ(rig.self_contact_participation.SealCandidate(
      SelfContactSource,rig.owner,token,{},&unchanged).status,
      fe::ShellPublicationStatus::ParticipationFailure);
  EXPECT_FALSE(unchanged.valid());
}

TEST(PhysicalScratchParticipationCuda,
     MissingBothReportsMappedWallFirstAndRevokesPreparedTrial) {
  Rig rig;
  ASSERT_TRUE(rig.Initialize());
  ASSERT_TRUE(rig.ConfigureScratch(true,true));
  Snapshot before,after;
  ASSERT_TRUE(rig.Read(before));
  Attempt attempt;
  ASSERT_TRUE(Prepare(rig,attempt));
  const auto report=rig.publication.SealPhysicalScratchParticipation(
      rig.owner,attempt.token,{});
  EXPECT_EQ(report.status,fe::ShellPublicationStatus::ParticipationFailure);
  EXPECT_NE(std::string(report.message).find("MappedWall"),std::string::npos);
  ASSERT_TRUE(rig.Read(after));
  Exact(before,after);
}

TEST(PhysicalScratchParticipationCuda,
     PlacementNewIssuerAbaAndBothDestructionOrdersAreSafe) {
  using Issuer=fe::ShellPhysicalScratchParticipation;
  {
    Rig rig;
    ASSERT_TRUE(rig.Initialize());
    alignas(Issuer) std::byte storage[sizeof(Issuer)];
    auto* first=new(storage) Issuer;
    ASSERT_TRUE(Good(rig.publication.ConfigurePhysicalScratchParticipation(
        rig.owner,rig.fixture.physical,rig.Participants(),
        rig.fixture.Identity(),{{first,MappedWallSource},{}})));
    first->~Issuer();
    auto* replacement=new(storage) Issuer;
    EXPECT_FALSE(replacement->configured());
    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    ASSERT_TRUE(rig.Begin(token,assembly));
    EXPECT_EQ(replacement->RecordAcceptedAssembly(
        MappedWallSource,rig.owner,token,assembly).status,
        fe::ShellPublicationStatus::NotInitialized);
    replacement->~Issuer();
    rig.owner.Discard();
    rig.publication.DiscardTrial();
  }
  {
    Rig rig;
    ASSERT_TRUE(rig.Initialize());
    alignas(Issuer) std::byte storage[sizeof(Issuer)];
    auto* issuer=new(storage) Issuer;
    ASSERT_TRUE(Good(rig.publication.ConfigurePhysicalScratchParticipation(
        rig.owner,rig.fixture.physical,rig.Participants(),
        rig.fixture.Identity(),{{issuer,MappedWallSource},{}})));
    rig.publication.~ShellBatchPublication();
    EXPECT_FALSE(issuer->configured());
    issuer->~Issuer();
    new(&rig.publication) fe::ShellBatchPublication;
  }
}

} // namespace physical_publication_test
