#include "PreparedSnapshotFixture.h"
#include "PreparedSnapshotCudaProbe.h"
namespace prepared_snapshot_test {
TEST_F(Cuda,CompletedPrivateDeviceCopyThenReportedFailureCannotPublishPreparedOrAcceptedOutputs) {
  for(bool prepared:{false,true}) {
    SCOPED_TRACE(prepared);Fixture fixture;fe::FENodalState owner;
    ASSERT_EQ(fixture.Initialize(owner).status,Code::Ok);const auto accepted_stamp=owner.accepted();
    fe::NodalTrialToken token;fe::NodalAssemblyView assembly;ASSERT_TRUE(Prepare(owner,fixture.Load(),token,assembly));
    fe::NodalPreparedView actual;ASSERT_EQ(owner.BorrowPrepared(token,&actual).status,Code::Ok);
    // Privileged fixture-only inspection verifies physical accepted bytes even
    // after the injected CUDA error correctly poisons ordinary owner APIs.
    std::array<double,19*nt::Capacity> before{},after{};
    const auto bytes=(19*fixture.input.n+18*fixture.count)*sizeof(double);
    ASSERT_EQ(cudaMemcpyAsync(before.data(),actual.base_kinematics.position_xyz,bytes,cudaMemcpyDeviceToHost,actual.stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(actual.stream),cudaSuccess);
    Output output;const auto untouched=output;const auto token_bytes=Image(token);
    prepared_snapshot_probe::FailAfterNextDeviceRead();
    const auto report=prepared?owner.CopyPrepared(token,output.nodes.buffer(),&output.prepared):
      owner.CopyAccepted(output.nodes.buffer(),&output.nodes.stamp);
    EXPECT_EQ(report.status,Code::DeviceFailure);EXPECT_TRUE(prepared_snapshot_probe::CompletedReadBeforeFailure());
    SameOutput(output,untouched);EXPECT_EQ(Image(token),token_bytes);
    EXPECT_TRUE(fe::trial_identity::SameStamp(owner.accepted(),accepted_stamp));
    EXPECT_EQ(owner.Commit(token).status,Code::DeviceFailure);
    ASSERT_EQ(cudaMemcpyAsync(after.data(),actual.base_kinematics.position_xyz,bytes,cudaMemcpyDeviceToHost,actual.stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(actual.stream),cudaSuccess);EXPECT_EQ(before,after);
  }
}
} // namespace prepared_snapshot_test
