#pragma once
#include "Probe.h"
#include "lib_utest/qualification/radioss_type25_runtime/FullLedgerRig.h"
#include "lib_utest/qualification/radioss_type25_local_geometry/Assertions.h"
#include "lib_utest/qualification/qt_mapped/TypedValues.h"
#include "lib_utest/qualification/qbat_resident/ResultValues.h"
#include "lib_src/elements/publication/PhysicalState.h"
#include "lib_src/elements/publication/PhysicalActivePrefix.h"
#include <cstring>
namespace native_group_activity_test {
namespace base=type25_source_test;namespace fe=tl::fea;namespace n=tlfea::contact::radioss_type25;
using base::Check;using Attempt=base::FullLedgerAttempt;
struct Snapshot {
 fe::NodalStamp stamp;fe::ShellPhysicalDiagnostics diagnostics;
 std::vector<double> physical;
 std::vector<std::uint64_t> material;
 std::array<std::vector<n::NativeGeometryHistory>,2> history;
 std::array<std::vector<int>,2> flags;
 std::array<fe::NativeContactPublicationSnapshot,2> native;
};
inline std::uint64_t Bits(double x){std::uint64_t b;std::memcpy(&b,&x,8);return b;}
struct Rig {
 base::FullLedgerRig physical;
 std::array<std::unique_ptr<n::Transaction>,2> contacts;
 std::array<n::Transaction*,2> members{};
 std::array<fe::NativeContactRosterEntry,2> roster;
 std::array<fe::ShellPhysicalScratchParticipationReceipt,2> receipts;
 std::array<const fe::ShellPhysicalScratchParticipationReceipt*,2> pointers{{&receipts[0],&receipts[1]}};
 std::size_t count=2;
 explicit Rig(double strain=2.5):physical(true,strain){}
 void Initialize(std::size_t size=2,bool bind=true){
  count=size;physical.Initialize(false);
  for(std::size_t i=0;i<count;++i){contacts[i]=std::make_unique<n::Transaction>();members[i]=contacts[i].get();auto source=physical.Source();source.source_id+=100*i;
   Check(contacts[i]->Initialize(physical.Config(),source,physical.owner,physical.publication,
       physical.fixture.physical,physical.Participants(),physical.Identity()));roster[i]=contacts[i]->native_roster_entry();}
  if(bind)Check(physical.publication.ConfigurePhysicalScratchParticipation(physical.owner,physical.fixture.physical,
      physical.Participants(),physical.Identity(),{{},{},{roster.data(),count}}));
 }
 void Prepare(Attempt&a){physical.Begin(a);for(std::size_t i=0;i<count;++i)Check(members[i]->AssembleAccepted(physical.owner,a.token,a.assembly));physical.Prepare(a);}
 n::TransactionGroupReport Group(Attempt&a,n::Transaction*const* chosen=nullptr,std::size_t size=0,
     fe::ShellPhysicalScratchParticipationReceipt* out=nullptr){
  return n::Transaction::SealCandidateGroup(chosen?chosen:members.data(),size?size:count,physical.publication,
      physical.owner,a.token,a.prepared,a.common,out?out:receipts.data(),size?size:count);
 }
 void Sequential(Attempt&a){for(std::size_t i=0;i<count;++i)Check(members[i]->SealCandidate(physical.owner,a.token,a.prepared,a.common,&receipts[i]));}
 void Commit(Attempt&a){Check(physical.publication.SealPhysicalScratchParticipation(physical.owner,a.token,{nullptr,nullptr,{pointers.data(),count}}));Check(physical.Commit(a));}
 void Discard(){for(auto*c:members)if(c)c->DiscardTrial();physical.Discard();}
 Snapshot Read(){
  Snapshot x;const auto nodes=physical.m.size();x.physical.resize(21*nodes+3);
  auto*v=x.physical.data();Check(physical.owner.CopyAccepted({v,v+3*nodes,nodes,v+6*nodes,v+10*nodes,v+13*nodes,v+16*nodes},&x.stamp));
  fe::NodalStamp copied;Check(physical.owner.CopyAcceptedCin({v+19*nodes,v+20*nodes,v+21*nodes,v+21*nodes+1,v+21*nodes+2,nodes,1},&copied));
  Check(physical.publication.CopyAcceptedPhysicalDiagnostics(x.stamp,&x.diagnostics));
  std::vector<fe::ShellBatchLayeredSection> q(physical.fixture.shells.qeph_count()),t(physical.fixture.shells.t3_count());
  fe::qeph::BatchDiagnostics qd;fe::t3::BatchDiagnostics td;fe::qbat::BatchDiagnostics bd;
  Check(physical.qeph.CopyAcceptedLayeredSectionHistory(x.stamp,q.data(),q.size(),&qd));Check(physical.t3.CopyAcceptedLayeredSectionHistory(x.stamp,t.data(),t.size(),&td));
  for(const auto&r:q)qt_mapped_test::Add(x.material,r);for(const auto&r:t)qt_mapped_test::Add(x.material,r);
  fe::qbat::BatchResult qb;Check(physical.qbat.CopyAcceptedResults(x.stamp,&qb,1,&bd));const auto bv=qbat_resident_test::ResultValues(qb);x.material.insert(x.material.end(),bv.begin(),bv.end());
  for(std::size_t i=0;i<count;++i){const auto rows=physical.fixture.secondary.size();x.history[i].resize(rows);x.flags[i].resize(rows);
   Check(members[i]->CopyAccepted({x.history[i].data(),x.flags[i].data(),rows},&x.native[i]));}
  return x;
 }
 const void* CaptureForce(Attempt&a,fe::t3::ForceTrial& saved){probe={};probe.capture_force=true;Check(physical.t3.CopyPreparedResults(a.common.t3,&saved,1));const auto*ptr=probe.force;Stop();if(!ptr)throw std::runtime_error("Actual T3 candidate source was not captured");return ptr;}
};
inline void Same(const Snapshot&a,const Snapshot&b,std::size_t count=2,bool same_owner=true){
 if(same_owner){EXPECT_TRUE(fe::trial_identity::SameStamp(a.stamp,b.stamp));EXPECT_TRUE(fe::shell_publication_detail::SamePhysicalDiagnostics(a.diagnostics,b.diagnostics));}
 else {EXPECT_EQ(a.stamp.epoch,b.stamp.epoch);EXPECT_EQ(a.stamp.time,b.stamp.time);}
 ASSERT_EQ(a.physical.size(),b.physical.size());EXPECT_EQ(std::memcmp(a.physical.data(),b.physical.data(),a.physical.size()*8),0);EXPECT_EQ(a.material,b.material);
 for(std::size_t i=0;i<count;++i){EXPECT_EQ(a.flags[i],b.flags[i]);EXPECT_EQ(a.native[i].generation,b.native[i].generation);
  EXPECT_EQ(a.native[i].selectors.history,b.native[i].selectors.history);EXPECT_EQ(a.native[i].selectors.reference,b.native[i].selectors.reference);
  EXPECT_EQ(a.native[i].selectors.reference_generation,b.native[i].selectors.reference_generation);
  for(std::size_t j=0;j<a.history[i].size();++j)type25_geometry_test::Same(a.history[i][j],b.history[i][j],true);}
}
template<class T>std::vector<unsigned char> Bytes(const T&x){const auto*p=reinterpret_cast<const unsigned char*>(&x);return {p,p+sizeof(x)};}
}
