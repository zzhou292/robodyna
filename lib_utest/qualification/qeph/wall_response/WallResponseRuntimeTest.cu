#include "WallResponseTransaction.h"
#include "../wall_incoming/WallIncomingFixture.h"
#include "../wall_coupled/WallCoupledResultChecks.h"

namespace tl::qualification::qeph::wall_response {
namespace rt=runtime_detail;
namespace c=qeph_wall_test;
using WallResponseCuda=tl_test::nodal_temporal::NodalTemporalCuda;
TEST_F(WallResponseCuda, ObserverAndReceiptRejectionKeepAcceptedResponseThenRetryEntryExactly) {
  qeph_wall_incoming_test::ScreenBinding binding; std::string error;
  ASSERT_TRUE(qeph_wall_incoming_test::ReadScreenBinding(binding,error))<<error;
  ::tl::qualification::qeph::wall_response::Run run;
  run.config={2,1,binding.selected_h,binding.decision_sha256};
  ASSERT_TRUE(BuildModel(2,run.model,error))<<error; run.samples.reserve(SampleCount);
  rt::Participants p(run.config,run.model); auto& r=p.wall.shell;
  ASSERT_TRUE(rt::Initialize(run,p,error))<<error;
  EXPECT_FALSE(rt::Initialize(run,p,error));
  const auto schedule=wr::AnalyzeWallSwitchingSchedule(run.model.screened(),Step(run.config));
  ASSERT_TRUE(schedule.passed); ASSERT_GT(schedule.entry_base_epoch,1u);
  while(run.accepted_steps+1<schedule.entry_base_epoch) {
    rt::StepStage stage;
    ASSERT_TRUE(rt::PrepareStep(run,p,stage,error))<<error;
    ASSERT_TRUE(rt::ObserveStep(run,stage,error))<<error;
    ASSERT_TRUE(rt::PublishStep(run,p,stage,error))<<error;
  }
  ASSERT_FALSE(run.summary.entry.observed);
  c::Snapshot base; ASSERT_TRUE(c::Read(r.owner,base));
  const auto cache_bytes=c::Bytes(p.cache);
  const auto native_cache_bytes=c::Bytes(p.native.cache);
  const auto native_state=p.native.state;
  const auto published_bytes=c::Bytes(p.published);
  const auto last_bytes=c::Bytes(run.last_accepted);
  const auto summary_bytes=c::Bytes(run.summary);
  const auto samples=run.samples;
  const auto ledgers=run.ledger_maxima;
  const auto accepted_epoch=run.accepted_steps;
  c::PortResults cache{}; port::BatchDiagnostics accepted;
  ASSERT_TRUE(c::ReadAccepted(r,cache,accepted)); const auto accepted_bytes=c::Bytes(accepted);
  rt::StepStage clean;
  ASSERT_TRUE(rt::PrepareStep(run,p,clean,error))<<error;
  ASSERT_TRUE(rt::ObserveStep(run,clean,error))<<error;
  ASSERT_TRUE(clean.summary.entry.observed);
  ASSERT_EQ(clean.trial.contact_base.potential.upper,0);
  ASSERT_GT(clean.components.contact.potential.lower,0);
  const auto sample_bytes=c::Bytes(clean.sample);
  const auto staged_summary_bytes=c::Bytes(clean.summary);
  const auto original_input=clean.endpoint;
  // Exercise the actual host observer, not just a private ready flag. Its
  // output remains intact, but this prospective observation cannot publish.
  clean.endpoint.contact.potential.error=-1;
  EXPECT_FALSE(rt::ObserveStep(run,clean,error)); EXPECT_FALSE(error.empty());
  EXPECT_FALSE(clean.observation_ready);
  EXPECT_EQ(c::Bytes(clean.sample),sample_bytes); EXPECT_EQ(c::Bytes(clean.summary),staged_summary_bytes);
  EXPECT_FALSE(rt::PublishStep(run,p,clean,error));
  clean.endpoint=original_input;
  ASSERT_TRUE(rt::ObserveStep(run,clean,error))<<error;
  auto receipt=c::Receipt(clean.components.shell); receipt.passed=false;
  EXPECT_EQ(port::CommitQephTrial(r.owner,clean.trial.nodal.token,r.batch,clean.components.shell,receipt).status,
            port::BatchStatus::StaleTrial);
  p.Discard();
  c::Snapshot after; ASSERT_TRUE(c::Read(r.owner,after)); c::SameState(base,after);
  ASSERT_TRUE(c::ReadAccepted(r,cache,accepted)); EXPECT_EQ(c::Bytes(accepted),accepted_bytes);
  EXPECT_EQ(c::Bytes(cache),cache_bytes); EXPECT_EQ(c::Bytes(p.cache),cache_bytes);
  EXPECT_EQ(c::Bytes(p.native.cache),native_cache_bytes); c::SameState(p.native.state,native_state);
  EXPECT_EQ(p.native.epoch,accepted_epoch); EXPECT_EQ(c::Bytes(p.published),published_bytes);
  EXPECT_EQ(run.accepted_steps,accepted_epoch); EXPECT_EQ(c::Bytes(run.last_accepted),last_bytes);
  EXPECT_EQ(run.attempted_steps,accepted_epoch+1);
  EXPECT_EQ(run.native_cell_intervals,2*run.attempted_steps);
  EXPECT_EQ(c::Bytes(run.summary),summary_bytes); EXPECT_EQ(run.ledger_maxima,ledgers);
  ASSERT_EQ(run.samples.size(),samples.size());
  for(unsigned i=0;i<samples.size();++i) EXPECT_EQ(c::Bytes(run.samples[i]),c::Bytes(samples[i]));
  rt::StepStage retry;
  ASSERT_TRUE(rt::PrepareStep(run,p,retry,error,true))<<error;
  ASSERT_TRUE(rt::ObserveStep(run,retry,error))<<error;
  EXPECT_NE(retry.trial.nodal.view.attempt,clean.trial.nodal.view.attempt);
  EXPECT_EQ(retry.trial.nodal.state.x,clean.trial.nodal.state.x);
  EXPECT_EQ(retry.trial.nodal.state.v,clean.trial.nodal.state.v);
  EXPECT_EQ(c::Bytes(retry.components.elements),c::Bytes(clean.components.elements));
  c::SameNumerics(retry.components.contact_result,clean.components.contact_result);
  EXPECT_EQ(retry.sample.values,clean.sample.values); EXPECT_EQ(retry.sample.errors,clean.sample.errors);
  EXPECT_EQ(retry.sample.synchronous_velocity,clean.sample.synchronous_velocity);
  EXPECT_EQ(retry.sample.synchronous_omega,clean.sample.synchronous_omega);
  EXPECT_EQ(c::Bytes(retry.summary),staged_summary_bytes);
  ASSERT_FALSE(::testing::Test::HasFailure());
  ASSERT_TRUE(rt::PublishStep(run,p,retry,error))<<error;
  ASSERT_TRUE(run.summary.entry.observed); ASSERT_EQ(run.accepted_steps,accepted_epoch+1);
  rt::StepStage applied;
  ASSERT_TRUE(rt::PrepareStep(run,p,applied,error))<<error;
  ASSERT_GT(applied.trial.contact_base.resultant.lower,0);
  ASSERT_TRUE(rt::ObserveStep(run,applied,error))<<error;
  ASSERT_GT(applied.endpoint.wall_impulse,run.last_accepted.wall_impulse);
  ASSERT_TRUE(rt::PublishStep(run,p,applied,error))<<error;
  EXPECT_EQ(run.accepted_steps,accepted_epoch+2);
  EXPECT_EQ(run.attempted_steps,accepted_epoch+3);
  EXPECT_EQ(run.native_cell_intervals,2*run.attempted_steps);
  RecordProperty("screen_index_sha256",binding.decision_sha256);
  RecordProperty("scope","short-observer-transaction-only");
  RecordProperty("native_cell_intervals",static_cast<int>(run.native_cell_intervals));
  RecordProperty("accepted_steps",static_cast<int>(run.accepted_steps));
  RecordProperty("attempted_steps",static_cast<int>(run.attempted_steps));
  RecordProperty("full_response_qualified","false");
}
} // namespace tl::qualification::qeph::wall_response
