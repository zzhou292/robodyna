#include "ActiveCollectionFixture.h"
#include <limits>

namespace active_shell_test {
namespace {
std::array<fe::NodalAllocationInfo,4> Allocations(const Rig& r) {
  return {r.owner.allocations(),r.qeph.allocations(),r.t3.allocations(),r.publication.allocations()};
}
void SameAllocations(const Rig& r,const std::array<fe::NodalAllocationInfo,4>& saved) {
  const auto actual=Allocations(r);
  for(unsigned i=0;i<4;++i) {
    EXPECT_EQ(actual[i].device_bytes,saved[i].device_bytes); EXPECT_EQ(actual[i].device_allocations,saved[i].device_allocations);
  }
}
void SameSnapshot(const Snapshot& a,const Snapshot& b) {
  EXPECT_EQ(a.x,b.x); EXPECT_EQ(a.v,b.v); EXPECT_EQ(a.w,b.w); EXPECT_EQ(a.orientation,b.orientation);
  EXPECT_EQ(a.stamp.epoch,b.stamp.epoch); EXPECT_EQ(a.stamp.time,b.stamp.time);
}
}
TEST_F(CudaTest, ActiveCountsCopyEveryTailAdvanceOneClockAndKeepAllocationsFixed) {
  for(bool plastic:{false,true}) {
    SCOPED_TRACE(plastic); auto r=std::make_unique<Rig>(); ASSERT_TRUE(r->Initialize(plastic));
    const auto allocated=Allocations(*r); const auto points=plastic?63u:0u;
    EXPECT_EQ(allocated[1].device_bytes,QBytes(QCount,NodeCount,points));
    EXPECT_EQ(allocated[2].device_bytes,TBytes(TCount,NodeCount,points));
    EXPECT_EQ(allocated[3].device_bytes,PublicationBytes(NodeCount));
    EXPECT_EQ(allocated[1].device_allocations,plastic?2u:1u); EXPECT_EQ(allocated[2].device_allocations,plastic?2u:1u);
    Results base,next; ASSERT_TRUE(Accepted(*r,base));
    // Fresh moving startup must read beyond the old 128-node staging boundary.
    long double k0=0; for(const auto& node:r->binding.active_nodes()) k0+=2*static_cast<long double>(node.native.mass);
    EXPECT_NEAR(base.diagnostics.kinetic.translation,double(k0),2e-12*double(k0));
    for(unsigned step=0;step<3;++step) {
      Prepared p; ASSERT_TRUE(Prepare(*r,p,step==0)); ASSERT_TRUE(Evaluate(*r,p,next));
      ASSERT_NO_FATAL_FAILURE(CheckTailHost(*r,p,base,next));
      for(const auto& result:next.qr) EXPECT_EQ(result.proposed_history.stamp().sample_index,step+1u);
      for(const auto& result:next.tr) EXPECT_EQ(result.proposed_history.stamp().sample_index,step+1u);
      if(plastic&&step==0) {
        EXPECT_GT(next.qs.back().diagnostics.maximum_plastic_strain,0);
        EXPECT_GT(next.ts.back().diagnostics.maximum_plastic_strain,0);
      }
      Results held; ASSERT_TRUE(Accepted(*r,held)); SameResults(base,held,plastic);
      ASSERT_TRUE(Publish(*r,p,next)); ASSERT_TRUE(Accepted(*r,held)); SameResults(next,held,plastic);
      EXPECT_EQ(r->owner.accepted().epoch,step+1u); EXPECT_EQ(r->owner.accepted().time,(step+1)*H);
      SameAllocations(*r,allocated); base=held;
    }
  }
}
TEST_F(CudaTest, FinalQAndTFailurePreserveYieldedStateAndRetryExactlyAtFullCounts) {
  auto r=std::make_unique<Rig>(); ASSERT_TRUE(r->Initialize(true));
  Prepared first; Results initial,base;
  ASSERT_TRUE(Accepted(*r,initial)); ASSERT_TRUE(Prepare(*r,first,true)); ASSERT_TRUE(Evaluate(*r,first,base));
  ASSERT_TRUE(Publish(*r,first,base)); ASSERT_TRUE(Accepted(*r,base));
  ASSERT_GT(base.qs.back().diagnostics.maximum_plastic_strain,0); ASSERT_GT(base.ts.back().diagnostics.maximum_plastic_strain,0);
  Snapshot before; ASSERT_TRUE(Read(*r,before)); const auto allocated=Allocations(*r);
  Prepared clean; Results expected; ASSERT_TRUE(Prepare(*r,clean)); ASSERT_TRUE(Evaluate(*r,clean,expected)); r->Discard();
  for(unsigned family=0;family<2;++family) {
    Prepared failed; ASSERT_TRUE(Prepare(*r,failed));
    auto qd=base.diagnostics.qeph; auto td=base.diagnostics.t3;
    if(!family) {
      ASSERT_EQ(r->t3.EvaluateCandidate(failed.view,&td).status,t::BatchStatus::Success);
      const auto nodes=r->binding.qeph_nodes(QCount-1); Collapse(failed,nodes[2],nodes[1]);
      const auto held=host_shell_test::Bytes(qd); const auto result=r->qeph.EvaluateCandidate(failed.view,&qd);
      EXPECT_EQ(result.status,q::BatchStatus::ElementFailure); EXPECT_EQ(result.element,QCount-1); EXPECT_EQ(host_shell_test::Bytes(qd),held);
    } else {
      ASSERT_EQ(r->qeph.EvaluateCandidate(failed.view,&qd).status,q::BatchStatus::Success);
      const auto nodes=r->binding.t3_nodes(TCount-1); Collapse(failed,nodes[2],nodes[1]);
      const auto held=host_shell_test::Bytes(td); const auto result=r->t3.EvaluateCandidate(failed.view,&td);
      EXPECT_EQ(result.status,t::BatchStatus::ElementFailure); EXPECT_EQ(result.element,TCount-1); EXPECT_EQ(host_shell_test::Bytes(td),held);
    }
    auto common=base.diagnostics; const auto held=host_shell_test::Bytes(common);
    EXPECT_EQ(r->publication.Prepare(r->owner,failed.token,qd,td,&common).status,fe::ShellPublicationStatus::StaleTrial);
    EXPECT_EQ(host_shell_test::Bytes(common),held);
    Results preserved; Snapshot after; ASSERT_TRUE(Accepted(*r,preserved)); ASSERT_TRUE(Read(*r,after));
    SameResults(base,preserved,true); SameSnapshot(before,after); SameAllocations(*r,allocated);
  }
  Prepared retry; Results actual; ASSERT_TRUE(Prepare(*r,retry)); ASSERT_TRUE(Evaluate(*r,retry,actual));
  SameResults(expected,actual,true); EXPECT_EQ(clean.endpoint.x,retry.endpoint.x);
  ASSERT_TRUE(Publish(*r,retry,actual)); EXPECT_EQ(r->owner.accepted().epoch,2u); SameAllocations(*r,allocated);
}
TEST_F(CudaTest, CrossBlockFailuresKeepFirstSourceOrderAndRetryEveryHistoryExactly) {
  for(bool plastic:{false,true}) {
    SCOPED_TRACE(plastic); auto r=std::make_unique<Rig>(); ASSERT_TRUE(r->Initialize(plastic));
    Prepared first; Results base;
    ASSERT_TRUE(Prepare(*r,first,true)); ASSERT_TRUE(Evaluate(*r,first,base));
    ASSERT_TRUE(Publish(*r,first,base)); ASSERT_TRUE(Accepted(*r,base));
    if(plastic) {
      ASSERT_GT(base.qs.back().diagnostics.maximum_plastic_strain,0);
      ASSERT_GT(base.ts.back().diagnostics.maximum_plastic_strain,0);
    }
    Snapshot before; ASSERT_TRUE(Read(*r,before)); const auto allocated=Allocations(*r);
    Prepared clean; Results expected;
    ASSERT_TRUE(Prepare(*r,clean)); ASSERT_TRUE(Evaluate(*r,clean,expected)); r->Discard();
    for(unsigned family=0;family<2;++family) for(std::size_t first_bad:{63u,64u}) {
      SCOPED_TRACE(family);
      SCOPED_TRACE(first_bad);
      Prepared failed; ASSERT_TRUE(Prepare(*r,failed));
      auto qd=base.diagnostics.qeph; auto td=base.diagnostics.t3;
      if(!family) ASSERT_EQ(r->t3.EvaluateCandidate(failed.view,&td).status,t::BatchStatus::Success);
      else ASSERT_EQ(r->qeph.EvaluateCandidate(failed.view,&qd).status,q::BatchStatus::Success);
      // Reverse mutation order, across block 0/1 and the partial final block.
      // Shared synthetic Q4 nodes also fail later parents, never an earlier one.
      const auto last=family?TCount-1:QCount-1;
      for(std::size_t e:{last,std::size_t{64},first_bad}) {
        if(!family) {const auto n=r->binding.qeph_nodes(e); Collapse(failed,n[2],n[1]);}
        else {const auto n=r->binding.t3_nodes(e); Collapse(failed,n[2],n[1]);}
      }
      if(!family) {
        const auto saved=host_shell_test::Bytes(qd);
        const auto rejected=r->qeph.EvaluateCandidate(failed.view,&qd);
        EXPECT_EQ(rejected.status,q::BatchStatus::ElementFailure); EXPECT_EQ(rejected.element,first_bad);
        EXPECT_EQ(host_shell_test::Bytes(qd),saved);
      } else {
        const auto saved=host_shell_test::Bytes(td);
        const auto rejected=r->t3.EvaluateCandidate(failed.view,&td);
        EXPECT_EQ(rejected.status,t::BatchStatus::ElementFailure); EXPECT_EQ(rejected.element,first_bad);
        EXPECT_EQ(host_shell_test::Bytes(td),saved);
      }
      auto common=base.diagnostics; const auto saved=host_shell_test::Bytes(common);
      EXPECT_EQ(r->publication.Prepare(r->owner,failed.token,qd,td,&common).status,fe::ShellPublicationStatus::StaleTrial);
      EXPECT_EQ(host_shell_test::Bytes(common),saved);
      Results held; Snapshot after; ASSERT_TRUE(Accepted(*r,held)); ASSERT_TRUE(Read(*r,after));
      SameResults(base,held,plastic); SameSnapshot(before,after); SameAllocations(*r,allocated);
      r->Discard();
      Prepared retry; Results actual;
      ASSERT_TRUE(Prepare(*r,retry)); ASSERT_TRUE(Evaluate(*r,retry,actual));
      SameResults(expected,actual,plastic); SameSnapshot(clean.endpoint,retry.endpoint);
      r->Discard();
    }
    Prepared final; Results actual;
    ASSERT_TRUE(Prepare(*r,final)); ASSERT_TRUE(Evaluate(*r,final,actual));
    SameResults(expected,actual,plastic); ASSERT_TRUE(Publish(*r,final,actual));
    EXPECT_EQ(r->owner.accepted().epoch,2u); SameAllocations(*r,allocated);
  }
}
TEST_F(CudaTest, StartupCapsPrecedeBorrowedReadsAndLateInvalidReferenceRetriesWithoutPublication) {
  auto r=std::make_unique<Rig>(); ASSERT_TRUE(r->BuildReference());
  const auto poison=reinterpret_cast<const q::QephBatchElement*>(std::uintptr_t{1});
  for(unsigned kind=0;kind<6;++kind) {
    auto config=r->QConfig(); q::QephBatch batch;
    if(kind==0) config.element_count=std::numeric_limits<std::size_t>::max();
    if(kind==1) config.owner.node_count=2049;
    if(kind==2) config.max_device_bytes=QBytes(QCount,NodeCount)-1;
    if(kind==3) config.storage_limits.max_host_bytes=1;
    if(kind==4) config.storage_limits={};
    if(kind==5) config.element_count=0;
    EXPECT_EQ(batch.Initialize(config,poison).status,q::BatchStatus::ResourceLimit);
    EXPECT_EQ(batch.allocations().device_allocations,0u);
  }
  for(unsigned family=0;family<2;++family) {
    q::QephBatch qb; t::T3Batch tb; auto qc=r->QConfig(); auto tc=r->TConfig();
    qc.max_device_bytes=QBytes(QCount,NodeCount,63)-1; tc.max_device_bytes=TBytes(TCount,NodeCount,63)-1;
    if(!family) {
      EXPECT_EQ(qb.InitializeJoined(qc,r->binding,r->catalog).status,q::BatchStatus::ResourceLimit);
      EXPECT_EQ(qb.allocations().device_allocations,0u); ++qc.max_device_bytes;
      ASSERT_EQ(qb.InitializeJoined(qc,r->binding,r->catalog).status,q::BatchStatus::Success);
    } else {
      EXPECT_EQ(tb.InitializeJoined(tc,r->binding,r->catalog).status,t::BatchStatus::ResourceLimit);
      EXPECT_EQ(tb.allocations().device_allocations,0u); ++tc.max_device_bytes;
      ASSERT_EQ(tb.InitializeJoined(tc,r->binding,r->catalog).status,t::BatchStatus::Success);
    }
  }
  std::vector<q::QephBatchElement> qinput(QCount);
  for(std::size_t e=0;e<QCount;++e) {
    qinput[e].reference=r->binding.qeph_reference(e);
    for(unsigned n=0;n<4;++n) qinput[e].nodes[n]=r->binding.qeph_nodes(e)[n];
  }
  qinput.back().reference.input.density=-1; q::QephBatch late;
  const auto rejected=late.Initialize(r->QConfig(),qinput.data());
  EXPECT_EQ(rejected.status,q::BatchStatus::ElementFailure); EXPECT_EQ(rejected.element,QCount-1);
  EXPECT_EQ(late.allocations().device_allocations,0u);
  ASSERT_EQ(late.InitializeJoined(r->QConfig(),r->binding,r->catalog).status,q::BatchStatus::Success);
}
}
