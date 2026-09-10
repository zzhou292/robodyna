#include "ResidentCollectionFixture.h"

namespace resident_collection_test {
static_assert(TCount==80 && QCount==32 && Nodes==90);
namespace {
__global__ void CollapseLastTriangle(fe::NodalPreparedView v,std::size_t from,std::size_t to) {
  auto* x=const_cast<double*>(v.kinematics.position_xyz);
  for(unsigned a=0;a<3;++a) x[3*to+a]=x[3*from+a];
}
}
TEST_F(CudaTest, TriangleTailBeyondWorkerCountMatchesEveryNativeHistoryAndRetry) {
  auto r=std::make_unique<Rig>(); auto native=std::make_unique<NativeSequence>();
  ASSERT_TRUE(r->Initialize()); ASSERT_TRUE(native->Initialize(*r));
  ASSERT_NO_FATAL_FAILURE(CheckReference(*r));
  const auto qa=r->qeph.allocations(),ta=r->t3.allocations();
  auto cache=std::make_unique<Staged>(),next=std::make_unique<Staged>();
  ASSERT_TRUE(Accepted(*r,*cache));
  for(unsigned step=0;step<4;++step) {
    SCOPED_TRACE(step); Snapshot base; Prepared p;
    ASSERT_TRUE(Read(r->owner,base));
    ASSERT_TRUE(Prepare(*r,step?Loads{}:Pulse(*r),*cache,p));
    ASSERT_TRUE(Evaluate(*r,p,*next,step%2));
    ASSERT_TRUE(native->Check(*r,p,*next));
    ASSERT_NO_FATAL_FAILURE(CheckLedgers(*r,base,*cache,p,*next));
    if(step==1) {
      ASSERT_NE(cache->t3.back().proposed_history.data().internal_work[0],0);
      auto clean=std::make_unique<Staged>(*next); const Snapshot clean_endpoint=p.endpoint;
      r->Discard();
      Prepared failed; ASSERT_TRUE(Prepare(*r,{},*cache,failed));
      q::BatchDiagnostics qd;
      ASSERT_EQ(r->qeph.EvaluateCandidate(failed.view,&qd).status,q::BatchStatus::Success);
      const auto& nodes=r->binding.t3_nodes(TCount-1);
      CollapseLastTriangle<<<1,1,0,failed.view.stream>>>(failed.view,nodes[2],nodes[1]);
      ASSERT_EQ(cudaGetLastError(),cudaSuccess);
      auto td=cache->diagnostics.t3; const auto saved_diagnostics=Bytes(td);
      const auto report=r->t3.EvaluateCandidate(failed.view,&td);
      EXPECT_EQ(report.status,t::BatchStatus::ElementFailure);
      EXPECT_EQ(report.element,TCount-1); EXPECT_EQ(Bytes(td),saved_diagnostics);
      r->Discard();
      Snapshot held; ASSERT_TRUE(Read(r->owner,held)); SameState(held,base);
      auto held_cache=std::make_unique<Staged>(); ASSERT_TRUE(Accepted(*r,*held_cache));
      ExactResults(*held_cache,*cache);
      ASSERT_TRUE(Prepare(*r,{},*cache,p)); ASSERT_TRUE(Evaluate(*r,p,*next));
      SameState(p.endpoint,clean_endpoint); ExactResults(*next,*clean);
      ASSERT_TRUE(native->Check(*r,p,*next));
    }
    ASSERT_TRUE(Publish(*r,p,*next)); native->Accept();
    ASSERT_TRUE(Accepted(*r,*cache)); ExactResults(*cache,*next);
    EXPECT_EQ(cache->t3.back().proposed_history.stamp().sample_index,step+1);
    EXPECT_EQ(r->qeph.allocations().device_bytes,qa.device_bytes);
    EXPECT_EQ(r->t3.allocations().device_bytes,ta.device_bytes);
    EXPECT_EQ(r->qeph.allocations().device_allocations,qa.device_allocations);
    EXPECT_EQ(r->t3.allocations().device_allocations,ta.device_allocations);
  }
  RecordProperty("native_qeph_intervals",4*QCount);
  RecordProperty("native_t3_intervals",4*TCount);
  RecordProperty("scope","strided candidate coverage, native histories, ordered work and tail rollback; no capacity increase");
}
} // namespace resident_collection_test
