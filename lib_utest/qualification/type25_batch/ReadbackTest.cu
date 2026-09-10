// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CudaFixture.h"

namespace type25_batch_test {
TEST_F(Type25BatchCuda, ReadbackAndCandidateAliasesRejectAtomicallyAndKeepValidRetry) {
  Rig rig;ASSERT_TRUE(rig.Initialize());std::vector<spring::Evaluation> before,output(129);
  spring::BatchDiagnostics initial,diagnostics;ASSERT_TRUE(rig.Read(before,initial));
  diagnostics.attempt=991;output.back().critical_dt_s=771;
  EXPECT_EQ(rig.batch.CopyAcceptedResults(rig.owner.accepted(),output.data(),128,&diagnostics).status,spring::BatchStatus::ResourceLimit);
  EXPECT_EQ(diagnostics.attempt,991u);EXPECT_EQ(output.back().critical_dt_s,771);
  EXPECT_EQ(rig.batch.CopyAcceptedResults(rig.owner.accepted(),output.data(),output.size(),
    reinterpret_cast<spring::BatchDiagnostics*>(output.data())).status,spring::BatchStatus::InvalidInput);
  EXPECT_EQ(output.back().critical_dt_s,771);
  auto* alias=reinterpret_cast<spring::Evaluation*>(const_cast<spring::Reference*>(rig.input.model.references()));
  EXPECT_EQ(rig.batch.CopyAcceptedResults(rig.owner.accepted(),alias,129,&diagnostics).status,spring::BatchStatus::InvalidInput);
  EXPECT_EQ(diagnostics.attempt,991u);
  fe::NodalTrialToken token;fe::NodalPreparedView prepared;ASSERT_TRUE(rig.Prepare(token,prepared));const auto view=prepared;
  EXPECT_EQ(rig.batch.EvaluateCandidate(rig.owner,token,prepared,reinterpret_cast<spring::BatchDiagnostics*>(&prepared)).status,spring::BatchStatus::InvalidInput);
  EXPECT_TRUE(fe::trial_identity::SamePrepared(prepared,view));
  ASSERT_EQ(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&diagnostics).status,spring::BatchStatus::Success);
  auto stale=diagnostics;++stale.attempt;
  EXPECT_EQ(rig.batch.CopyPreparedResults(stale,output.data(),output.size()).status,spring::BatchStatus::StaleTrial);
  EXPECT_EQ(output.back().critical_dt_s,771);
  ASSERT_EQ(rig.batch.CopyPreparedResults(diagnostics,output.data(),output.size()).status,spring::BatchStatus::Success);
  rig.Discard();spring::BatchDiagnostics accepted;std::vector<spring::Evaluation> after;ASSERT_TRUE(rig.Read(after,accepted));
  EXPECT_TRUE(spring::batch_detail::SameDiagnostics(initial,accepted));for(std::size_t e=0;e<after.size();++e)Exact(before[e],after[e]);
}
} // namespace type25_batch_test
