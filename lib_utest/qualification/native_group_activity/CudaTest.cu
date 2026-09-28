#include "Fixture.h"
namespace native_group_activity_test {
TEST(NativeGroupedActivityCuda, ExactSequentialTrajectoryWithOneFreshTraversalPerGroup){
 Rig grouped,sequential;ASSERT_NO_THROW(grouped.Initialize());ASSERT_NO_THROW(sequential.Initialize());
 for(unsigned step=0;step<4;++step){SCOPED_TRACE(step);Attempt a,b;ASSERT_NO_THROW(grouped.Prepare(a));ASSERT_NO_THROW(sequential.Prepare(b));
  Watch();const auto result=grouped.Group(a);const auto one=Stop();ASSERT_EQ(result.report.status,n::TransactionStatus::Ok)<<result.report.message;ASSERT_GT(one.copies,0u);
  Watch();ASSERT_NO_THROW(sequential.Sequential(b));const auto two=Stop();EXPECT_EQ(two.copies,2*one.copies);EXPECT_EQ(two.bytes,2*one.bytes);EXPECT_EQ(two.syncs,2*one.syncs);
  ASSERT_NO_THROW(grouped.Commit(a));ASSERT_NO_THROW(sequential.Commit(b));ASSERT_NO_FATAL_FAILURE(Same(grouped.Read(),sequential.Read(),2,false));}
}
TEST(NativeGroupedActivityCuda, StandaloneSameAttemptCorruptionIsFreshAndWholeAttemptRollsBack){
 Rig rig;ASSERT_NO_THROW(rig.Initialize());const auto before=rig.Read();Attempt a;ASSERT_NO_THROW(rig.Prepare(a));
 ASSERT_NO_THROW(Check(rig.contacts[0]->SealCandidate(rig.physical.owner,a.token,a.prepared,a.common,&rig.receipts[0])));
 fe::t3::ForceTrial saved;const void*ptr=nullptr;ASSERT_NO_THROW(ptr=rig.CaptureForce(a,saved));auto bad=saved;bad.diagnostics.native_sound_speed=std::nan("13");
 ASSERT_EQ(cudaMemcpy(const_cast<void*>(ptr),&bad,sizeof(bad),cudaMemcpyHostToDevice),cudaSuccess);
 Watch();const auto rejected=rig.contacts[1]->SealCandidate(rig.physical.owner,a.token,a.prepared,a.common,&rig.receipts[1]);const auto watched=Stop();
 EXPECT_EQ(rejected.status,n::TransactionStatus::PublicationFailure);EXPECT_GT(watched.copies,0u);EXPECT_NE(rig.physical.Commit(a).status,fe::ShellPublicationStatus::Success);
 ASSERT_EQ(cudaMemcpy(const_cast<void*>(ptr),&saved,sizeof(saved),cudaMemcpyHostToDevice),cudaSuccess);rig.Discard();ASSERT_NO_FATAL_FAILURE(Same(before,rig.Read()));
 Attempt retry;ASSERT_NO_THROW(rig.Prepare(retry));ASSERT_EQ(rig.Group(retry).report.status,n::TransactionStatus::Ok);ASSERT_NO_THROW(rig.Commit(retry));
}
TEST(NativeGroupedActivityCuda, NewGroupCallCannotReuseEarlierPublicPrefixVerdict){
 Rig rig;ASSERT_NO_THROW(rig.Initialize());fe::PhysicalActivePrefix prefix;
 ASSERT_EQ(prefix.Initialize(rig.physical.owner,rig.physical.publication,rig.physical.fixture.physical,rig.physical.Participants(),rig.physical.Identity()).status,fe::ActivePrefixStatus::Ok);
 const auto before=rig.Read();Attempt a;ASSERT_NO_THROW(rig.Prepare(a));ASSERT_EQ(prefix.CheckPrepared(rig.physical.owner,rig.physical.publication,a.token,a.common,a.prepared).status,fe::ActivePrefixStatus::Ok);
 fe::t3::ForceTrial saved;const void*ptr=nullptr;ASSERT_NO_THROW(ptr=rig.CaptureForce(a,saved));auto bad=saved;bad.diagnostics.native_sound_speed=std::nan("17");
 ASSERT_EQ(cudaMemcpy(const_cast<void*>(ptr),&bad,sizeof(bad),cudaMemcpyHostToDevice),cudaSuccess);
 const auto held=Bytes(rig.receipts);Watch();const auto rejected=rig.Group(a);const auto watched=Stop();EXPECT_EQ(rejected.interface_index,0u);EXPECT_EQ(rejected.report.status,n::TransactionStatus::PublicationFailure);EXPECT_GT(watched.copies,0u);EXPECT_EQ(held,Bytes(rig.receipts));
 ASSERT_EQ(cudaMemcpy(const_cast<void*>(ptr),&saved,sizeof(saved),cudaMemcpyHostToDevice),cudaSuccess);rig.Discard();ASSERT_NO_FATAL_FAILURE(Same(before,rig.Read()));
}
TEST(NativeGroupedActivityCuda, FirstActivityFailurePrecedesLaterMissingMember){
 Rig rig;ASSERT_NO_THROW(rig.Initialize());Attempt a;ASSERT_NO_THROW(rig.Prepare(a));fe::t3::ForceTrial saved;const void*ptr=nullptr;ASSERT_NO_THROW(ptr=rig.CaptureForce(a,saved));
 auto bad=saved;bad.diagnostics.native_sound_speed=std::nan("19");ASSERT_EQ(cudaMemcpy(const_cast<void*>(ptr),&bad,sizeof(bad),cudaMemcpyHostToDevice),cudaSuccess);
 n::Transaction missing;std::array<n::Transaction*,2> members{{rig.contacts[0].get(),&missing}};const auto rejected=rig.Group(a,members.data());
 EXPECT_EQ(rejected.interface_index,0u);EXPECT_EQ(rejected.report.status,n::TransactionStatus::PublicationFailure);
 ASSERT_EQ(cudaMemcpy(const_cast<void*>(ptr),&saved,sizeof(saved),cudaMemcpyHostToDevice),cudaSuccess);rig.Discard();
}
TEST(NativeGroupedActivityCuda, LateFailureRevokesFirstReceiptAndRetainsExactRetry){
 Rig rig;ASSERT_NO_THROW(rig.Initialize());const auto before=rig.Read();Attempt a;ASSERT_NO_THROW(rig.Prepare(a));n::Transaction missing;
 std::array<n::Transaction*,2> members{{rig.contacts[0].get(),&missing}};const auto output=Bytes(rig.receipts);const auto rejected=rig.Group(a,members.data());
 EXPECT_EQ(rejected.interface_index,1u);EXPECT_EQ(rejected.report.status,n::TransactionStatus::NotInitialized);EXPECT_EQ(output,Bytes(rig.receipts));
 EXPECT_NE(rig.physical.Commit(a).status,fe::ShellPublicationStatus::Success);rig.Discard();ASSERT_NO_FATAL_FAILURE(Same(before,rig.Read()));
 for(unsigned retry=0;retry<2;++retry){Attempt b;ASSERT_NO_THROW(rig.Prepare(b));Watch();const auto result=rig.Group(b);const auto check=Stop();EXPECT_EQ(result.report.status,n::TransactionStatus::Ok);EXPECT_GT(check.copies,0u);rig.Discard();ASSERT_NO_FATAL_FAILURE(Same(before,rig.Read()));}
}
TEST(NativeGroupedActivityCuda, DuplicateReorderedMissingAndAliasedOutputRejectAtomically){
 Rig rig;ASSERT_NO_THROW(rig.Initialize());const auto before=rig.Read();
 for(unsigned fault=0;fault<6;++fault){SCOPED_TRACE(fault);Attempt a;ASSERT_NO_THROW(rig.Prepare(a));auto chosen=rig.members;std::size_t count=2;auto*out=rig.receipts.data();
  if(fault==0)chosen[1]=chosen[0];if(fault==1)std::swap(chosen[0],chosen[1]);if(fault==2)count=1;
  if(fault==3)out=reinterpret_cast<fe::ShellPhysicalScratchParticipationReceipt*>(&a.common);
  if(fault==4)out=reinterpret_cast<fe::ShellPhysicalScratchParticipationReceipt*>(chosen.data());
  if(fault==5)out=reinterpret_cast<fe::ShellPhysicalScratchParticipationReceipt*>(const_cast<fe::NodalCoefficientNode*>(rig.physical.fixture.ledger.nodes().data()));
  const auto held=Bytes(rig.receipts);const auto result=n::Transaction::SealCandidateGroup(chosen.data(),count,rig.physical.publication,rig.physical.owner,a.token,a.prepared,a.common,out,count);
  EXPECT_NE(result.report.status,n::TransactionStatus::Ok);EXPECT_EQ(held,Bytes(rig.receipts));rig.Discard();ASSERT_NO_FATAL_FAILURE(Same(before,rig.Read()));}
}
TEST(NativeGroupedActivityCuda, ForeignMemberNeverRevokesItsOwnPreparedOwner){
 Rig local,foreign;ASSERT_NO_THROW(local.Initialize());ASSERT_NO_THROW(foreign.Initialize());Attempt a,b;ASSERT_NO_THROW(local.Prepare(a));ASSERT_NO_THROW(foreign.Prepare(b));
 auto members=local.members;members[1]=foreign.contacts[1].get();const auto rejected=local.Group(a,members.data());EXPECT_EQ(rejected.interface_index,1u);EXPECT_NE(rejected.report.status,n::TransactionStatus::Ok);
 ASSERT_NO_THROW(foreign.Sequential(b));ASSERT_NO_THROW(foreign.Commit(b));EXPECT_EQ(foreign.physical.owner.accepted().epoch,1u);local.Discard();
}
TEST(NativeGroupedActivityCuda, OneMemberAndTransferFailureKeepExistingAuthority){
 {Rig rig;ASSERT_NO_THROW(rig.Initialize(1));Attempt a;ASSERT_NO_THROW(rig.Prepare(a));ASSERT_EQ(rig.Group(a).report.status,n::TransactionStatus::Ok);ASSERT_NO_THROW(rig.Commit(a));}
 {Rig rig;ASSERT_NO_THROW(rig.Initialize());Attempt a;ASSERT_NO_THROW(rig.Prepare(a));const auto stamp=rig.physical.owner.accepted();const auto held=Bytes(rig.receipts);
  Watch();probe.fail_copy=1;const auto failed=rig.Group(a);Stop();EXPECT_NE(failed.report.status,n::TransactionStatus::Ok);EXPECT_EQ(held,Bytes(rig.receipts));EXPECT_TRUE(fe::trial_identity::SameStamp(stamp,rig.physical.owner.accepted()));EXPECT_NE(rig.physical.Commit(a).status,fe::ShellPublicationStatus::Success);}
}
TEST(NativeGroupedActivityCuda, GenuineRemovalRejectsBothParticipantsAndExactlyRetries) {
 Rig rig(1e-3);ASSERT_NO_THROW(rig.Initialize());
 for(unsigned warm=0;warm<2;++warm){Attempt a;ASSERT_NO_THROW(rig.Prepare(a));ASSERT_EQ(rig.Group(a).report.status,n::TransactionStatus::Ok);ASSERT_NO_THROW(rig.Commit(a));}
 const auto before=rig.Read();std::vector<double> first_force;
 for(unsigned retry=0;retry<2;++retry){Attempt a;auto&p=rig.physical;ASSERT_NO_THROW(p.Begin(a));
  const auto node=p.fixture.domain.Find(14);const double load=2*p.m[node]*.001/(base::FullLedgerRig::Dt*base::FullLedgerRig::Dt);
  ASSERT_EQ(cudaMemcpyAsync(a.assembly.forces.force_x+node,&load,sizeof(load),cudaMemcpyHostToDevice,a.assembly.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(a.assembly.stream),cudaSuccess);
  for(auto*c:rig.members)ASSERT_NO_THROW(Check(c->AssembleAccepted(p.owner,a.token,a.assembly)));
  const auto force=p.Force(a);if(!retry)first_force=force;else {ASSERT_EQ(first_force.size(),force.size());EXPECT_EQ(std::memcmp(first_force.data(),force.data(),force.size()*8),0);}
  ASSERT_NO_THROW(p.Prepare(a));const auto rejected=rig.Group(a);EXPECT_EQ(rejected.interface_index,0u);EXPECT_EQ(rejected.report.status,n::TransactionStatus::ActivityChange);EXPECT_EQ(rejected.report.row,0u);
  EXPECT_NE(p.Commit(a).status,fe::ShellPublicationStatus::Success);rig.Discard();ASSERT_NO_FATAL_FAILURE(Same(before,rig.Read()));}
}

}
