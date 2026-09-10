#include "WallResponseRuntime.h"
#include "WallResponseTransaction.h"
#include <chrono>
#include <iostream>

namespace tl::qualification::qeph::wall_response {
namespace {
struct Scope {
  Run& run; runtime_detail::Participants& participants;
  std::chrono::steady_clock::time_point started=std::chrono::steady_clock::now();
  ~Scope() {
    participants.Discard();
    run.elapsed_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
    if(!run.completed) std::cerr<<"Wall response stopped: "<<run.failure<<"; accepted "<<run.accepted_steps<<'\n';
  }
};
}
void Execute(Run& run) {
  run.failure="Response config/model/capacity preflight";
  ASSERT_TRUE(ValidConfig(run.config)); ASSERT_TRUE(run.model.prepared());
  ASSERT_TRUE(run.samples.empty()); ASSERT_EQ(run.samples.capacity(),SampleCount);
  ASSERT_EQ(run.model.fields().cells,run.config.cells);
  int devices=0; ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess); ASSERT_GT(devices,0);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  runtime_detail::Participants participants(run.config,run.model); Scope scope{run,participants};
  auto& r=participants.wall.shell; std::string error;
  run.failure="Screened startup/initial observation";
  ASSERT_TRUE(runtime_detail::Initialize(run,participants,error))<<error;
  auto progress=std::chrono::steady_clock::now();
  for(unsigned step=0;step<Steps(run.config);++step) {
    run.failure="Native accepted-base proposal/owner/contact/cache/ledger checks";
    runtime_detail::StepStage stage;
    ASSERT_TRUE(runtime_detail::PrepareStep(run,participants,stage,error))<<error;
    run.failure="Endpoint observer/event/energy staging";
    ASSERT_TRUE(runtime_detail::ObserveStep(run,stage,error))<<error;
    EXPECT_EQ(r.owner.allocations().device_bytes,run.owner_device_bytes);
    EXPECT_EQ(r.owner.allocations().device_allocations,run.owner_allocations);
    EXPECT_EQ(r.batch.allocations().device_bytes,run.batch_device_bytes);
    EXPECT_EQ(r.batch.allocations().device_allocations,run.batch_allocations);
    EXPECT_EQ(participants.wall.wall.allocations().device_bytes,run.wall_device_bytes);
    EXPECT_EQ(participants.wall.wall.allocations().device_allocations,run.wall_allocations);
    ASSERT_FALSE(::testing::Test::HasFailure())<<"No wall-response receipt after a failed scientific check";
    run.failure="Joint owner/history/contact-output/observer publication";
    ASSERT_TRUE(runtime_detail::PublishStep(run,participants,stage,error))<<error;
    const auto now=std::chrono::steady_clock::now();
    if(now-progress>std::chrono::seconds(30)) {
      std::cout<<"Accepted "<<run.accepted_steps<<'/'<<Steps(run.config)<<" wall endpoints; time "
               <<run.last_accepted.time<<" s\n"<<std::flush;
      progress=now;
    }
  }
  run.failure="Completed wall-response event/analytic/energy validation";
  EXPECT_EQ(run.samples.size(),SampleCount); EXPECT_EQ(run.accepted_steps,Steps(run.config));
  EXPECT_EQ(run.attempted_steps,run.accepted_steps); EXPECT_EQ(run.last_accepted.time,Horizon);
  EXPECT_EQ(r.owner.accepted().time,Horizon); EXPECT_EQ(participants.native.time,Horizon);
  EXPECT_EQ(run.native_cell_intervals,run.config.cells*run.accepted_steps);
  ASSERT_TRUE(CompleteSummary(run.model,run.config,run.summary,error))<<error;
  ASSERT_FALSE(::testing::Test::HasFailure()); run.completed=true; run.failure.clear();
}
} // namespace tl::qualification::qeph::wall_response
