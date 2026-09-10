#include "ForceStageCaptureFixture.h"
#include "PreparedSnapshotCudaProbe.h"

namespace force_stage_capture_test {
TEST_F(Cuda,CompletedCaptureDeviceCopyThenFailurePreservesAllOutputAndAcceptedState) {
  Fixture fixture;fe::FENodalState owner;ASSERT_EQ(Initialize(fixture,owner,Config(fixture)).status,Code::Ok);
  const auto accepted=owner.accepted();fe::NodalTrialToken token;fe::NodalAssemblyView view;
  ASSERT_TRUE(Prepare(owner,fixture.Load(),token,view));fe::NodalPreparedView actual;
  ASSERT_EQ(owner.BorrowPrepared(token,&actual).status,Code::Ok);
  std::array<double,19*nt::Capacity> before{},after{};const auto bytes=(19*fixture.input.n+18*fixture.count)*sizeof(double);
  ASSERT_EQ(cudaMemcpyAsync(before.data(),actual.base_kinematics.position_xyz,bytes,cudaMemcpyDeviceToHost,actual.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(actual.stream),cudaSuccess);
  Capture out;const auto untouched=out;const auto token_bytes=Image(token);
  prepared_snapshot_probe::FailAfterNextDeviceRead();
  EXPECT_EQ(owner.CopyPreparedForceStage(token,out.buffer(),&out.prepared).status,Code::DeviceFailure);
  EXPECT_TRUE(prepared_snapshot_probe::CompletedReadBeforeFailure());SameCapture(out,untouched);EXPECT_EQ(Image(token),token_bytes);
  EXPECT_TRUE(fe::trial_identity::SameStamp(owner.accepted(),accepted));EXPECT_EQ(owner.Commit(token).status,Code::DeviceFailure);
  ASSERT_EQ(cudaMemcpyAsync(after.data(),actual.base_kinematics.position_xyz,bytes,cudaMemcpyDeviceToHost,actual.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(actual.stream),cudaSuccess);EXPECT_EQ(before,after);
}
} // namespace force_stage_capture_test
