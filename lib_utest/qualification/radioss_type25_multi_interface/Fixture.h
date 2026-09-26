// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../radioss_type25_runtime/MovingSceneRig.h"
#include "../radioss_type25_local_geometry/Assertions.h"
#include <gtest/gtest.h>
#include <cstring>
namespace native_group_test {
namespace nr = native_runtime_test;
namespace fe = tl::fea;
namespace n = tlfea::contact::radioss_type25;
using nr::Check;
using nr::Attempt;
using nr::State;
struct Snapshot {
  State physical;
  std::array<std::array<n::NativeGeometryHistory,18>,2> history;
  std::array<std::array<int,18>,2> flags;
  std::array<fe::NativeContactPublicationSnapshot,2> contact;
};
struct Rig {
  nr::Rig physical;
  std::array<std::unique_ptr<n::Transaction>,2> native;
  std::array<fe::NativeContactRosterEntry,2> entries;
  std::array<fe::ShellPhysicalScratchParticipationReceipt,2> receipts;
  std::array<const fe::ShellPhysicalScratchParticipationReceipt*,2> receipt_ptrs;
  std::size_t count = 2;
  void Initialize(bool bind=true, std::size_t size=2) {
    count=size;
    physical.Initialize(nr::ObservedSource::Limits(),false);
    for(std::size_t i=0;i<count;++i) {
      native[i]=std::make_unique<n::Transaction>();
      auto source=physical.source.View();
      auto config=nr::ObservedSource::Config();
      source.source_id=100+i;
      if(i) {
        // Distinct numerical law and reference margin; no second shipping
        // source/deck authority is asserted by this protocol fixture.
        config.normal.damping_factor=.25;
        config.friction_coefficients={.3,{0,0,0,0,.3,-.001}};
        source.margin=.01;
      }
      Check(native[i]->Initialize(config,source,physical.owner,*physical.publication,
          physical.fixture.physical,physical.Participants(),physical.Identity(),nr::ObservedSource::Limits()));
      entries[i]=native[i]->native_roster_entry();
      receipt_ptrs[i]=&receipts[i];
    }
    if(bind)Check(Bind());
  }
  fe::ShellPhysicalScratchRoster Roster() const { return {{},{},{entries.data(),count}}; }
  fe::ShellPhysicalScratchReceiptRoster Receipts() const { return {nullptr,nullptr,{receipt_ptrs.data(),count}}; }
  fe::ShellPublicationReport Bind(fe::ShellPhysicalScratchParticipationLimits limits={}) {
    return physical.publication->ConfigurePhysicalScratchParticipation(physical.owner,
        physical.fixture.physical,physical.Participants(),physical.Identity(),Roster(),limits);
  }
  void Begin(Attempt& a) { physical.BeginMaterials(a); }
  void Assemble(Attempt& a) {
    for(std::size_t i=0;i<count;++i)
      Check(native[i]->AssembleAccepted(physical.owner,a.token,a.assembly));
  }
  void PrepareMaterials(Attempt& a) {
    Check(physical.owner.SealAssembly(a.token));
    Check(fe::AdvanceStaggeredCin(physical.owner,a.token,
        {a.assembly.owner_id,a.assembly.accepted.base_epoch,a.assembly.attempt,
         nodal_empty_test::Fixture::Qualification,physical.fixture.fixed_dt,.2,true,
         {fe::NodalCinStructuralProfile::NativeOrdinaryRigidTrace,.8,true}}));
    Check(physical.owner.BorrowPrepared(a.token,&a.prepared));
    Check(physical.quad.EvaluateCandidate(physical.owner,a.token,a.prepared,&a.material.qeph));
    Check(physical.triangle.EvaluateCandidate(physical.owner,a.token,a.prepared,&a.material.t3));
    Check(physical.publication->PreparePhysical(physical.owner,a.token,
        {&a.material.qeph,&a.material.t3},&a.common));
  }
  void SealChildren(Attempt& a) {
    for(std::size_t i=0;i<count;++i)
      Check(native[i]->SealCandidate(physical.owner,a.token,a.prepared,a.common,&receipts[i]));
  }
  void SealGroup(Attempt& a) {
    Check(physical.publication->SealPhysicalScratchParticipation(physical.owner,a.token,Receipts()));
  }
  void Prepare(Attempt& a) { PrepareMaterials(a);SealChildren(a);SealGroup(a); }
  void Step() { Attempt a;Begin(a);Assemble(a);Prepare(a);Check(physical.Commit(a)); }
  void Discard() {
    // Common discard alone must revoke all children. Individual transactions
    // intentionally are not reset here: retry tests exercise their existing
    // prior-attempt private-cache clearing in AssembleAccepted.
    physical.owner.Discard();
    physical.publication->DiscardTrial();
  }
  Snapshot Read() {
    Snapshot result;
    auto& s=result.physical;
    Check(physical.owner.CopyAccepted({s.x.data(),s.v.data(),18,s.q.data(),s.omega.data(),
        s.reaction.data(),s.couple.data()},&s.stamp));
    double numerical=0;
    fe::NodalStamp coefficient;
    Check(physical.owner.CopyAcceptedCin({s.mass.data(),s.inertia.data(),nullptr,nullptr,&numerical,18,0},&coefficient));
    for(std::size_t i=0;i<count;++i)
      Check(native[i]->CopyAccepted({result.history[i].data(),result.flags[i].data(),18},&result.contact[i]));
    return result;
  }
};
template<class T,std::size_t N> void Bits(const std::array<T,N>& a,const std::array<T,N>& b) {
  EXPECT_EQ(std::memcmp(a.data(),b.data(),sizeof(T)*N),0);
}
inline void Same(const Snapshot& a,const Snapshot& b,std::size_t count=2) {
  const auto& x=a.physical;const auto& y=b.physical;
  EXPECT_TRUE(fe::trial_identity::SameStamp(x.stamp,y.stamp));
  Bits(x.x,y.x);Bits(x.v,y.v);Bits(x.q,y.q);Bits(x.omega,y.omega);
  Bits(x.reaction,y.reaction);Bits(x.couple,y.couple);Bits(x.mass,y.mass);Bits(x.inertia,y.inertia);
  for(std::size_t i=0;i<count;++i) {
    SCOPED_TRACE(i);
    const auto& p=a.contact[i];const auto& q=b.contact[i];
    EXPECT_TRUE(fe::trial_identity::SameStamp(p.stamp,q.stamp));
    EXPECT_TRUE(fe::trial_identity::SameStamp(p.force_base_stamp,q.force_base_stamp));
    EXPECT_EQ(p.available,q.available);EXPECT_EQ(p.force_phase_available,q.force_phase_available);
    EXPECT_EQ(p.generation,q.generation);EXPECT_EQ(p.selectors.history,q.selectors.history);
    EXPECT_EQ(p.selectors.reference,q.selectors.reference);
    EXPECT_EQ(p.selectors.reference_generation,q.selectors.reference_generation);
    EXPECT_EQ(p.selectors.has_reference,q.selectors.has_reference);
    EXPECT_EQ(a.flags[i],b.flags[i]);
    for(unsigned row=0;row<18;++row) type25_geometry_test::Same(a.history[i][row],b.history[i][row],true);
  }
}
inline void NoTrial(Rig& rig,const Attempt& a) {
  for(std::size_t i=0;i<rig.count;++i) {
    n::runtime_qualification::Observation observed;
    EXPECT_FALSE(n::runtime_qualification::Access::Read(*rig.native[i],rig.physical.owner,
        a.token,a.assembly,&observed));
  }
}
} // namespace native_group_test
