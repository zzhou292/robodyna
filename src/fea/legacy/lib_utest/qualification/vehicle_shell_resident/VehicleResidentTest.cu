#include "VehicleResidentFixture.h"
namespace vehicle_resident_test {
namespace {
auto Allocations(const Rig& r){return std::array{r.owner.allocations(),r.qeph.allocations(),r.t3.allocations(),r.publication.allocations()};}
void SameAllocations(const Rig& r,const std::array<fe::NodalAllocationInfo,4>& expected) {
  const auto actual=Allocations(r);for(unsigned i=0;i<4;++i){EXPECT_EQ(actual[i].device_bytes,expected[i].device_bytes);
    EXPECT_EQ(actual[i].device_allocations,expected[i].device_allocations);}
}
}
TEST_F(CudaTest, VehicleSourceCountsPublishEveryNativeHistoryWithOneCompleteKineticScope) {
  for(bool plastic:{false,true}) {
    SCOPED_TRACE(plastic);auto r=std::make_unique<Rig>();ASSERT_TRUE(r->Initialize(plastic));
    ASSERT_NO_FATAL_FAILURE(ReportStorage(*r));const auto allocation=Allocations(*r);
    EXPECT_EQ(allocation[0].device_bytes,147871771u);
    EXPECT_EQ(allocation[1].device_bytes,1132577992u+(plastic?210140232u:0u));
    EXPECT_EQ(allocation[2].device_bytes,72889780u+(plastic?13632712u:0u));
    EXPECT_EQ(allocation[3].device_bytes,11513280u);
    Results base(r->nq,r->nt,plastic),next(r->nq,r->nt,plastic),visible(r->nq,r->nt,plastic);
    ASSERT_TRUE(Accepted(*r,base));Prepared p(r->n);ASSERT_TRUE(Prepare(*r,p,true));ASSERT_TRUE(Evaluate(*r,p,next));
    ASSERT_NO_FATAL_FAILURE(CheckNativeTail(*r,p,base,next));
    ASSERT_NO_FATAL_FAILURE(CheckCompleteFields(*r,p,next));
    if(plastic){EXPECT_GT(next.qs.back().diagnostics.maximum_plastic_strain,0);EXPECT_GT(next.ts.back().diagnostics.maximum_plastic_strain,0);}
    ASSERT_TRUE(Accepted(*r,visible));SameResults(base,visible);
    EXPECT_EQ(r->qeph.CopyPreparedResults(next.diagnostics.qeph,reinterpret_cast<q::ForceTrial*>(1),r->nq-1).status,q::BatchStatus::ResourceLimit);
    EXPECT_EQ(r->t3.CopyPreparedResults(next.diagnostics.t3,reinterpret_cast<t::ForceTrial*>(1),r->nt-1).status,t::BatchStatus::ResourceLimit);
    // Joined participants cannot independently publish this common candidate.
    EXPECT_NE(q::CommitQephTrial(r->owner,p.token,r->qeph,next.diagnostics.qeph,
      {next.diagnostics.qeph.owner_id,next.diagnostics.qeph.base_epoch,next.diagnostics.qeph.attempt,Qualification,true}).status,q::BatchStatus::Success);
    // The standalone rejection deliberately discards the attempt. A clean
    // retry must recover exactly the same complete native history/section data.
    r->Discard();Prepared retry(r->n);ASSERT_TRUE(Prepare(*r,retry,true));ASSERT_TRUE(Evaluate(*r,retry,visible));
    SameResults(next,visible);ASSERT_TRUE(Publish(*r,retry,visible));
    ASSERT_TRUE(Accepted(*r,base));SameResults(visible,base);
    EXPECT_EQ(r->owner.accepted().epoch,1u);EXPECT_EQ(r->owner.accepted().time,H);SameAllocations(*r,allocation);
  }
}
TEST_F(CudaTest, VehicleLastQAndTFailurePreserveYieldedCachesAndAllNodeStateThenRetryExactly) {
  auto r=std::make_unique<Rig>();ASSERT_TRUE(r->Initialize(true));const auto allocation=Allocations(*r);
  Results base(r->nq,r->nt,true),expected(r->nq,r->nt,true),visible(r->nq,r->nt,true);
  Prepared first(r->n);ASSERT_TRUE(Prepare(*r,first,true));ASSERT_TRUE(Evaluate(*r,first,base));ASSERT_TRUE(Publish(*r,first,base));
  ASSERT_GT(base.qs.back().diagnostics.maximum_plastic_strain,0);ASSERT_GT(base.ts.back().diagnostics.maximum_plastic_strain,0);
  Snapshot before(r->n),after(r->n);ASSERT_TRUE(Read(*r,before));
  Prepared clean(r->n);ASSERT_TRUE(Prepare(*r,clean));ASSERT_TRUE(Evaluate(*r,clean,expected));r->Discard();
  for(unsigned family=0;family<2;++family){Prepared failed(r->n);ASSERT_TRUE(Prepare(*r,failed));
    auto qd=base.diagnostics.qeph;auto td=base.diagnostics.t3;
    if(!family){
      ASSERT_EQ(r->t3.EvaluateCandidate(r->owner,failed.token,failed.view,&td).status,t::BatchStatus::Success);
      const auto nodes=r->binding.qeph_nodes(r->nq-1);Collapse(failed,nodes[2],nodes[1]);
      const auto held=vehicle_shell_test::Bytes(qd);const auto report=r->qeph.EvaluateCandidate(r->owner,failed.token,failed.view,&qd);
      EXPECT_EQ(report.status,q::BatchStatus::ElementFailure);EXPECT_EQ(report.element,r->nq-1);EXPECT_EQ(vehicle_shell_test::Bytes(qd),held);
    } else {
      ASSERT_EQ(r->qeph.EvaluateCandidate(r->owner,failed.token,failed.view,&qd).status,q::BatchStatus::Success);
      const auto nodes=r->binding.t3_nodes(r->nt-1);Collapse(failed,nodes[2],nodes[1]);
      const auto held=vehicle_shell_test::Bytes(td);const auto report=r->t3.EvaluateCandidate(r->owner,failed.token,failed.view,&td);
      EXPECT_EQ(report.status,t::BatchStatus::ElementFailure);EXPECT_EQ(report.element,r->nt-1);EXPECT_EQ(vehicle_shell_test::Bytes(td),held);
    }
    auto common=base.diagnostics;const auto held=vehicle_shell_test::Bytes(common);
    EXPECT_EQ(r->publication.Prepare(r->owner,failed.token,qd,td,&common).status,fe::ShellPublicationStatus::StaleTrial);
    EXPECT_EQ(vehicle_shell_test::Bytes(common),held);ASSERT_TRUE(Accepted(*r,visible));SameResults(base,visible);
    ASSERT_TRUE(Read(*r,after));SameSnapshot(before,after);SameAllocations(*r,allocation);
  }
  Prepared retry(r->n);ASSERT_TRUE(Prepare(*r,retry));ASSERT_TRUE(Evaluate(*r,retry,visible));
  SameResults(expected,visible);EXPECT_EQ(clean.endpoint.x,retry.endpoint.x);EXPECT_EQ(clean.endpoint.orientation,retry.endpoint.orientation);
  ASSERT_TRUE(Publish(*r,retry,visible));EXPECT_EQ(r->owner.accepted().epoch,2u);SameAllocations(*r,allocation);
}
} // namespace vehicle_resident_test
