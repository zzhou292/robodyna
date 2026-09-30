// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CudaFixture.h"
#include <limits>

namespace type25_batch_test {
namespace {
void AgainstHost(Rig& rig,const fe::NodalTrialToken& token,const fe::NodalPreparedView& prepared,
                 const std::vector<spring::Evaluation>& original,const std::vector<spring::Evaluation>& actual) {
  temporal::Snapshot fields;fe::NodalPreparedView stamp;
  ASSERT_EQ(rig.owner.CopyPrepared(token,fields.buffer(),&stamp).status,fe::NodalStatus::Ok);
  ASSERT_TRUE(fe::trial_identity::SamePrepared(stamp,prepared));
  for(std::size_t e=0;e<actual.size();++e) {
    SCOPED_TRACE(e);spring::EndpointKinematics nodes[2];
    for(unsigned i=0;i<2;++i) {
      const auto n=rig.input.model.connections()[e].global_node[i];
      nodes[i]={{fields.x[3*n],fields.x[3*n+1],fields.x[3*n+2]},
                {fields.v[3*n],fields.v[3*n+1],fields.v[3*n+2]},
                {fields.omega[3*n],fields.omega[3*n+1],fields.omega[3*n+2]}};
    }
    spring::Evaluation expected;
    ASSERT_EQ(spring::Evaluate(rig.input.model.source_units(),rig.input.property.property,rig.input.model.references()[e],
      original[e].history,nodes,rig.initial.h,expected),spring::Status::Success);
    const auto a=type25_test::EvaluationValues(actual[e]),b=type25_test::EvaluationValues(expected);
    for(std::size_t i=0;i<a.size();++i) {
      SCOPED_TRACE(i);EXPECT_NEAR(a[i],b[i],2e-12*std::max(std::abs(b[i]),1e-20));
    }
    EXPECT_EQ(actual[e].history.active,expected.history.active);
  }
}
}
TEST_F(Type25BatchCuda, CompleteCandidateHasActualEndpointFieldsAndNoAcceptedMutation) {
  for(std::size_t count:{129u,1024u}) {
    Rig rig;ASSERT_TRUE(rig.Initialize(count));std::vector<spring::Evaluation> before,after,trial(count);
    spring::BatchDiagnostics accepted,after_d,candidate;ASSERT_TRUE(rig.Read(before,accepted));
    const auto allocation=rig.batch.allocations();const auto host=rig.batch.host_bytes();
    fe::NodalTrialToken token;fe::NodalPreparedView prepared;ASSERT_TRUE(rig.Prepare(token,prepared));
    ASSERT_EQ(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&candidate).status,spring::BatchStatus::Success);
    ASSERT_EQ(rig.batch.CopyPreparedResults(candidate,trial.data(),trial.size()).status,spring::BatchStatus::Success);
    EXPECT_EQ(candidate.element_count,count);EXPECT_EQ(candidate.active_count,count);EXPECT_EQ(candidate.phase,spring::BatchPhase::Prepared);
    EXPECT_EQ(candidate.internal_kick_work,0);EXPECT_EQ(candidate.internal_drift_work,0);EXPECT_TRUE(candidate.accepted_force_assembled);
    AgainstHost(rig,token,prepared,before,trial);
    EXPECT_GT(tl::math::fixed3::Norm(trial.back().endpoints[0].force_N),0);
    ASSERT_TRUE(rig.Read(after,after_d));EXPECT_TRUE(spring::batch_detail::SameDiagnostics(accepted,after_d));
    for(std::size_t e=0;e<count;++e)Exact(before[e],after[e]);
    EXPECT_EQ(rig.batch.allocations().device_bytes,allocation.device_bytes);EXPECT_EQ(rig.batch.allocations().device_allocations,1u);EXPECT_EQ(rig.batch.host_bytes(),host);
    rig.Discard();
  }
}
TEST_F(Type25BatchCuda, FinalConnectionFailurePreservesOutputsAndAcceptedCacheThenRetriesExactly) {
  Rig rig;ASSERT_TRUE(rig.Initialize());std::vector<spring::Evaluation> before,after,first(129),retry(129);
  spring::BatchDiagnostics initial,observed,clean,good;ASSERT_TRUE(rig.Read(before,initial));
  temporal::Snapshot nodal_before,nodal_after;ASSERT_TRUE(temporal::Read(rig.owner,nodal_before));
  fe::NodalTrialToken token;fe::NodalPreparedView prepared;ASSERT_TRUE(rig.Prepare(token,prepared));
  ASSERT_EQ(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&clean).status,spring::BatchStatus::Success);
  ASSERT_EQ(rig.batch.CopyPreparedResults(clean,first.data(),first.size()).status,spring::BatchStatus::Success);rig.Discard();
  ASSERT_TRUE(rig.Prepare(token,prepared));const double bad=std::numeric_limits<double>::quiet_NaN();
  ASSERT_EQ(cudaMemcpyAsync(const_cast<double*>(prepared.kinematics.position_xyz)+12,&bad,sizeof(bad),cudaMemcpyHostToDevice,prepared.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(prepared.stream),cudaSuccess);
  observed.attempt=887;const auto rejected=rig.batch.EvaluateCandidate(rig.owner,token,prepared,&observed);
  EXPECT_EQ(rejected.status,spring::BatchStatus::ElementFailure);EXPECT_EQ(rejected.element,128u);EXPECT_EQ(observed.attempt,887u);
  ASSERT_TRUE(rig.Read(after,observed));EXPECT_TRUE(spring::batch_detail::SameDiagnostics(initial,observed));
  for(std::size_t e=0;e<before.size();++e)Exact(before[e],after[e]);
  rig.Discard();ASSERT_TRUE(temporal::Read(rig.owner,nodal_after));temporal::SameState(nodal_before,nodal_after);
  ASSERT_TRUE(rig.Prepare(token,prepared));ASSERT_EQ(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&good).status,spring::BatchStatus::Success);
  ASSERT_EQ(rig.batch.CopyPreparedResults(good,retry.data(),retry.size()).status,spring::BatchStatus::Success);
  for(std::size_t e=0;e<retry.size();++e)Exact(first[e],retry[e]);
  EXPECT_GT(good.attempt,clean.attempt);EXPECT_EQ(good.time,clean.time);EXPECT_EQ(good.minimum_native_dt,clean.minimum_native_dt);rig.Discard();
}
} // namespace type25_batch_test
