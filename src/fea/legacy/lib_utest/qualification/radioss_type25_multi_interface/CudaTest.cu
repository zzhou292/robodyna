// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
namespace native_group_test {
TEST(NativeGroupCuda, WholeRosterAdmissionPrecedesAnyBindAndSupportsRetry) {
  Rig rig,other;
  ASSERT_NO_THROW(rig.Initialize(false));
  ASSERT_NO_THROW(other.Initialize(false));
  const auto original=rig.entries;
  const auto attempt=[&](const fe::ShellPhysicalScratchRoster& roster) {
    return rig.physical.publication->ConfigurePhysicalScratchParticipation(rig.physical.owner,
        rig.physical.fixture.physical,rig.physical.Participants(),rig.physical.Identity(),roster);
  };
  rig.entries[1]=rig.entries[0];
  EXPECT_NE(attempt(rig.Roster()).status,fe::ShellPublicationStatus::Success);
  rig.entries=original;
  rig.entries[1]=other.entries[1];
  EXPECT_NE(attempt(rig.Roster()).status,fe::ShellPublicationStatus::Success);
  rig.entries=original;
  auto mixed=rig.Roster();mixed.self_contact=rig.native[0]->roster_entry();
  EXPECT_EQ(attempt(mixed).status,fe::ShellPublicationStatus::InvalidInput);
  for(const auto& entry:rig.entries) EXPECT_FALSE(entry.issuer()->configured());
  fe::ShellPhysicalScratchParticipationForecast plan;
  ASSERT_EQ(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(rig.Roster(),{},plan).status,
      fe::ShellPublicationStatus::Success);
  // Rejection-only overlap probe: entries remain actual live typed objects;
  // the incompatible output lvalue is never accessed unless admission is broken.
  struct alignas(fe::ShellPhysicalScratchParticipationForecast) Aliased {
    std::array<fe::NativeContactRosterEntry,2> entries;
    std::byte tail[sizeof(fe::ShellPhysicalScratchParticipationForecast)];
  } alias{rig.entries,{}};
  const auto alias_roster=fe::ShellPhysicalScratchRoster{{},{},{alias.entries.data(),2}};
  auto* alias_output=reinterpret_cast<fe::ShellPhysicalScratchParticipationForecast*>(alias.entries.data());
  EXPECT_EQ(fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(alias_roster,{},*alias_output).status,
      fe::ShellPublicationStatus::InvalidInput);
  for(unsigned i=0;i<2;++i) {
    EXPECT_EQ(alias.entries[i].issuer(),rig.entries[i].issuer());
    EXPECT_EQ(alias.entries[i].source_id(),rig.entries[i].source_id());
  }
  fe::ShellPhysicalScratchParticipationLimits limit{plan.total_host_bytes-1,2};
  EXPECT_EQ(rig.Bind(limit).status,fe::ShellPublicationStatus::ResourceLimit);
  limit={plan.total_host_bytes,1};
  EXPECT_EQ(rig.Bind(limit).status,fe::ShellPublicationStatus::ResourceLimit);
  for(const auto& entry:rig.entries) EXPECT_FALSE(entry.issuer()->configured());
  limit.max_native_interfaces=2;
  ASSERT_NO_THROW(Check(rig.Bind(limit)));
  for(const auto& entry:rig.entries) EXPECT_TRUE(entry.issuer()->configured());
  ASSERT_NO_THROW(rig.Step());
  EXPECT_EQ(rig.Read().contact[0].generation,1u);EXPECT_EQ(rig.Read().contact[1].generation,1u);
}

TEST(NativeGroupCuda, SameSourceIdFromDistinctTransactionsCannotBind) {
  Rig rig;
  ASSERT_NO_THROW(rig.Initialize(false));
  n::Transaction duplicate;
  auto source=rig.physical.source.View();source.source_id=100;
  ASSERT_NO_THROW(Check(duplicate.Initialize(nr::ObservedSource::Config(),source,rig.physical.owner,
      *rig.physical.publication,rig.physical.fixture.physical,rig.physical.Participants(),
      rig.physical.Identity(),nr::ObservedSource::Limits())));
  const auto original=rig.entries[1];rig.entries[1]=duplicate.native_roster_entry();
  EXPECT_EQ(rig.Bind().status,fe::ShellPublicationStatus::InvalidInput);
  EXPECT_FALSE(rig.entries[0].issuer()->configured());
  EXPECT_FALSE(duplicate.native_roster_entry().issuer()->configured());
  rig.entries[1]=original;
  ASSERT_NO_THROW(Check(rig.Bind()));
}

TEST(NativeGroupCuda, WrongOrderRejectsBeforeWritesAndFreshAttemptResetsPrivateCaches) {
  Rig rig;
  ASSERT_NO_THROW(rig.Initialize());
  const auto before=rig.Read();
  Attempt wrong;
  ASSERT_NO_THROW(rig.Begin(wrong));
  std::array<double,54> forces{},after{};
  std::array<double,18> stiffness{},after_k{};
  ASSERT_NO_THROW(rig.physical.Force(wrong,forces,stiffness));
  EXPECT_EQ(rig.native[1]->AssembleAccepted(rig.physical.owner,wrong.token,wrong.assembly).status,
      n::TransactionStatus::PublicationFailure);
  // The existing qualification readback copies still-live trial storage after
  // revocation, solely to prove the pre-kernel order gate made no force writes.
  ASSERT_NO_THROW(rig.physical.Force(wrong,after,after_k));
  Bits(forces,after);Bits(stiffness,after_k);
  ASSERT_NO_FATAL_FAILURE(Same(before,rig.Read()));
  NoTrial(rig,wrong);
  Attempt partial;
  ASSERT_NO_THROW(rig.Begin(partial));
  ASSERT_NO_THROW(Check(rig.native[0]->AssembleAccepted(rig.physical.owner,partial.token,partial.assembly)));
  auto forged=partial.assembly;++forged.attempt;
  EXPECT_NE(rig.native[1]->AssembleAccepted(rig.physical.owner,partial.token,forged).status,n::TransactionStatus::Ok);
  ASSERT_NO_FATAL_FAILURE(Same(before,rig.Read()));
  NoTrial(rig,partial);
  // No per-transaction DiscardTrial: common revocation plus authentic new
  // attempt must clear each transaction's old private cache before use.
  ASSERT_NO_THROW(rig.Step());
  EXPECT_EQ(rig.Read().physical.stamp.epoch,1u);
}

TEST(NativeGroupCuda, CompleteGroupPublishesDistinctLawsAndIndependentReferenceSelectors) {
  Rig rig;
  ASSERT_NO_THROW(rig.Initialize());
  const auto bytes0=rig.native[0]->allocations(),bytes1=rig.native[1]->allocations();
  bool active=false,different=false,references=false;
  for(unsigned step=0;step<500;++step) {
    SCOPED_TRACE(step);
    Attempt a;
    ASSERT_NO_THROW(rig.Begin(a));
    ASSERT_NO_THROW(rig.Assemble(a));
    active|=rig.native[0]->last_diagnostics().active_forces>0&&rig.native[1]->last_diagnostics().active_forces>0;
    ASSERT_NO_THROW(rig.Prepare(a));
    ASSERT_NO_THROW(Check(rig.physical.Commit(a)));
    const auto s=rig.Read();
    for(unsigned i=0;i<2;++i) {
      EXPECT_EQ(s.contact[i].generation,step+1);
      EXPECT_EQ(s.contact[i].stamp.epoch,step+1);
      EXPECT_EQ(s.contact[i].force_base_stamp.epoch,step);
      EXPECT_EQ(s.contact[i].selectors.history,(step+1)%2);
    }
    references|=s.contact[0].selectors.reference_generation!=s.contact[1].selectors.reference_generation;
    for(unsigned row=0;row<18;++row)
      different|=s.history[0][row].row.history.normal.damping_half_force!=
                  s.history[1][row].row.history.normal.damping_half_force;
    if(HasFailure())return;
  }
  EXPECT_TRUE(active);EXPECT_TRUE(different);EXPECT_TRUE(references);
  EXPECT_EQ(rig.native[0]->allocations().device_bytes,bytes0.device_bytes);
  EXPECT_EQ(rig.native[1]->allocations().device_bytes,bytes1.device_bytes);
  EXPECT_EQ(rig.native[0]->allocations().host_bytes,bytes0.host_bytes);
  EXPECT_EQ(rig.native[1]->allocations().host_bytes,bytes1.host_bytes);
}

TEST(NativeGroupCuda, MissingSwappedDuplicateAndStaleReceiptsCannotPartiallyPublish) {
  Rig rig;
  ASSERT_NO_THROW(rig.Initialize());
  const auto before=rig.Read();
  fe::ShellPhysicalScratchParticipationReceipt stale;
  for(unsigned variant=0;variant<7;++variant) {
    SCOPED_TRACE(variant);
    Attempt a;
    ASSERT_NO_THROW(rig.Begin(a));
    ASSERT_NO_THROW(rig.Assemble(a));
    ASSERT_NO_THROW(rig.PrepareMaterials(a));
    ASSERT_NO_THROW(rig.SealChildren(a));
    auto pointers=rig.receipt_ptrs;
    if(variant==0){stale=rig.receipts[1];pointers[1]=nullptr;}
    if(variant==1)std::swap(pointers[0],pointers[1]);
    if(variant==2)pointers[1]=pointers[0];
    if(variant==3)pointers[1]=&stale;
    auto view=fe::NativeContactReceiptView{pointers.data(),2};
    if(variant==4)view.entries=nullptr;
    if(variant==5)view.entries=reinterpret_cast<const fe::ShellPhysicalScratchParticipationReceipt* const*>(
        reinterpret_cast<const unsigned char*>(pointers.data())+1);
    if(variant==6)pointers[1]=reinterpret_cast<const fe::ShellPhysicalScratchParticipationReceipt*>(
        reinterpret_cast<const unsigned char*>(&rig.receipts[1])+1);
    EXPECT_NE(rig.physical.publication->SealPhysicalScratchParticipation(rig.physical.owner,a.token,
        {nullptr,nullptr,view}).status,fe::ShellPublicationStatus::Success);
    ASSERT_NO_FATAL_FAILURE(Same(before,rig.Read()));
    NoTrial(rig,a);
  }
  ASSERT_NO_THROW(rig.Step());
  EXPECT_EQ(rig.Read().contact[0].generation,1u);EXPECT_EQ(rig.Read().contact[1].generation,1u);
}

TEST(NativeGroupCuda, ActiveSecondChildAndCommonRejectionRevokeEverySiblingThenRetry) {
  Rig rig;
  ASSERT_NO_THROW(rig.Initialize());
  bool exercised=false;
  for(unsigned step=0;step<500;++step) {
    const auto before=rig.Read();
    Attempt a;
    ASSERT_NO_THROW(rig.Begin(a));
    ASSERT_NO_THROW(rig.Assemble(a));
    if(!exercised&&rig.native[0]->last_diagnostics().active_forces&&rig.native[1]->last_diagnostics().active_forces) {
      ASSERT_NO_THROW(rig.PrepareMaterials(a));
      ASSERT_NO_THROW(Check(rig.native[0]->SealCandidate(rig.physical.owner,a.token,a.prepared,a.common,&rig.receipts[0])));
      auto wrong=a.prepared;++wrong.attempt;
      EXPECT_NE(rig.native[1]->SealCandidate(rig.physical.owner,a.token,wrong,a.common,&rig.receipts[1]).status,
          n::TransactionStatus::Ok);
      ASSERT_NO_FATAL_FAILURE(Same(before,rig.Read()));
      NoTrial(rig,a);
      Attempt common;
      ASSERT_NO_THROW(rig.Begin(common));
      ASSERT_NO_THROW(rig.Assemble(common));
      ASSERT_NO_THROW(rig.Prepare(common));
      EXPECT_NE(rig.physical.Commit(common,false).status,fe::ShellPublicationStatus::Success);
      ASSERT_NO_FATAL_FAILURE(Same(before,rig.Read()));
      NoTrial(rig,common);
      Attempt structural;
      ASSERT_NO_THROW(rig.Begin(structural));
      ASSERT_NO_THROW(rig.Assemble(structural));
      ASSERT_NO_THROW(rig.Prepare(structural));
      auto changed=structural.common;++changed.base_stamp.epoch;
      const auto receipt=fe::NodalValidationReceipt{structural.prepared.owner_id,
          structural.prepared.kinematics.base_epoch,structural.prepared.attempt,
          nodal_empty_test::Fixture::Qualification,true};
      EXPECT_NE(rig.physical.publication->CommitPhysical(rig.physical.owner,structural.token,changed,receipt).status,
          fe::ShellPublicationStatus::Success);
      ASSERT_NO_FATAL_FAILURE(Same(before,rig.Read()));
      ASSERT_NO_THROW(rig.Step());
      const auto after=rig.Read();
      EXPECT_EQ(after.physical.stamp.epoch,before.physical.stamp.epoch+1);
      for(unsigned i=0;i<2;++i) EXPECT_EQ(after.contact[i].generation,before.contact[i].generation+1);
      exercised=true;
      break;
    }
    ASSERT_NO_THROW(rig.Prepare(a));
    ASSERT_NO_THROW(Check(rig.physical.Commit(a)));
  }
  EXPECT_TRUE(exercised);
}

TEST(NativeGroupCuda, TombstoneAndPublisherDestructionRevokeMandatoryGroup) {
  Rig rig;
  ASSERT_NO_THROW(rig.Initialize());
  const auto before=rig.Read();
  Attempt a;
  ASSERT_NO_THROW(rig.Begin(a));
  ASSERT_NO_THROW(rig.Assemble(a));
  ASSERT_NO_THROW(rig.Prepare(a));
  const auto first=rig.native[0]->accepted();
  rig.native[1].reset();
  EXPECT_NE(rig.physical.Commit(a).status,fe::ShellPublicationStatus::Success);
  EXPECT_TRUE(fe::trial_identity::SameStamp(before.physical.stamp,rig.physical.owner.accepted()));
  EXPECT_EQ(rig.native[0]->accepted().generation,first.generation);
  n::runtime_qualification::Observation scratch;
  EXPECT_FALSE(n::runtime_qualification::Access::Read(*rig.native[0],rig.physical.owner,a.token,a.assembly,&scratch));
  Attempt retry;
  ASSERT_NO_THROW(rig.Begin(retry));
  EXPECT_EQ(rig.native[0]->AssembleAccepted(rig.physical.owner,retry.token,retry.assembly).status,
      n::TransactionStatus::PublicationFailure);
  rig.physical.publication.reset();
  EXPECT_FALSE(rig.native[0]->accepted().available);
  rig.native[0]->DiscardTrial();
  EXPECT_EQ(rig.physical.owner.accepted().epoch,0u);
}

TEST(NativeGroupCuda, OneNativeGroupMemberPreservesLegacySingleNumericalPath) {
  Rig group;
  nr::Rig legacy;
  ASSERT_NO_THROW(group.Initialize(true,1));
  ASSERT_NO_THROW(legacy.Initialize());
  bool active=false;
  for(unsigned i=0;i<470;++i) {
    SCOPED_TRACE(i);
    Attempt first,second;
    ASSERT_NO_THROW(group.Begin(first));
    ASSERT_NO_THROW(group.Assemble(first));
    ASSERT_NO_THROW(legacy.BeginMaterials(second));
    ASSERT_NO_THROW(Check(legacy.contact.AssembleAccepted(legacy.owner,second.token,second.assembly)));
    active|=group.native[0]->last_diagnostics().active_forces>0;
    EXPECT_EQ(group.native[0]->last_diagnostics().active_forces,legacy.contact.last_diagnostics().active_forces);
    if(i>=467) {
      std::array<double,54> x{},y{};
      std::array<double,18> kx{},ky{};
      ASSERT_NO_THROW(group.physical.Force(first,x,kx));
      ASSERT_NO_THROW(legacy.Force(second,y,ky));
      Bits(x,y);Bits(kx,ky);
    }
    ASSERT_NO_THROW(group.Prepare(first));
    ASSERT_NO_THROW(Check(group.physical.Commit(first)));
    ASSERT_NO_THROW(legacy.Prepare(second));
    ASSERT_NO_THROW(Check(legacy.Commit(second)));
    if(i>=467) {
      const auto a=group.Read();const auto b=legacy.Read();
      Bits(a.physical.x,b.x);Bits(a.physical.v,b.v);Bits(a.physical.q,b.q);
      Bits(a.physical.omega,b.omega);Bits(a.physical.reaction,b.reaction);Bits(a.physical.couple,b.couple);
      Bits(a.physical.mass,b.mass);Bits(a.physical.inertia,b.inertia);
      for(unsigned row=0;row<18;++row)type25_geometry_test::Same(a.history[0][row],b.history[row],true);
      EXPECT_EQ(a.flags[0],b.initial_contact);
      EXPECT_EQ(a.contact[0].generation,b.contact.generation);
      EXPECT_EQ(a.contact[0].selectors.reference_generation,b.contact.selectors.reference_generation);
    }
    if(HasFailure())return;
  }
  EXPECT_TRUE(active);
}
} // namespace native_group_test
