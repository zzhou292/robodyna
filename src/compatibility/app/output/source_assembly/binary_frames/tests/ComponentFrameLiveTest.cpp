#include "ComponentFrameLiveSupport.h"
#include "lib_utest/qualification/nodal_rigid_group/PreparedSnapshotCudaProbe.h"

namespace crash::output::assembly::binary::test {
TEST(ComponentBinaryLive, ActualSixPartAcceptedBinaryAndLegacyFieldsHaveCompleteBitParity) {
    int devices=0;ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);ASSERT_GT(devices,0);
    dynamics::SourceAssemblyWallCase run;
    const auto result=run.Initialize(fixture::prepared::WallAssembly(),fixture::PreparedWall(),fixture::Configuration());
    ASSERT_TRUE(result)<<result.message;ActualParity(run,1030,915);
}
TEST(ComponentBinaryLive, ActualReadbackFailurePreservesTheSelectedRecordAndOwner) {
    int devices=0;ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);ASSERT_GT(devices,0);
    dynamics::SourceAssemblyWallCase run;
    const auto result=run.Initialize(fixture::prepared::WallAssembly(),fixture::PreparedWall(),fixture::Configuration());
    ASSERT_TRUE(result)<<result.message;
    SourceAssemblyBinaryFrames producer(run,{run.owner()->accepted().owner_id,23,31},5);
    producer.Capture(run);const auto held=*producer.frame();const auto* pointer=producer.frame();
    ASSERT_TRUE(run.Step());const auto accepted=run.owner()->accepted();
    prepared_snapshot_probe::FailAfterNextDeviceRead();
    EXPECT_THROW(producer.Capture(run),std::runtime_error);
    EXPECT_TRUE(prepared_snapshot_probe::CompletedReadBeforeFailure());
    EXPECT_EQ(producer.frame(),pointer);Same(held,*producer.frame());
    EXPECT_TRUE(tl::fea::trial_identity::SameStamp(accepted,run.owner()->accepted()));
    EXPECT_THROW(producer.Capture(run),std::runtime_error);Same(held,*producer.frame());
}
TEST(ComponentBinaryLive, StartupByteCapAndForeignOwnerLeaveExistingRecordUnchanged) {
    int devices=0;ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);ASSERT_GT(devices,0);
    dynamics::SourceAssemblyWallCase run,other;
    ASSERT_TRUE(run.Initialize(fixture::prepared::WallAssembly(),fixture::PreparedWall(),fixture::Configuration()));
    ASSERT_TRUE(other.Initialize(fixture::prepared::WallAssembly(),fixture::PreparedWall(),fixture::Configuration()));
    const visual::Identity identity{run.owner()->accepted().owner_id,23,31};
    SourceAssemblyBinaryFrames producer(run,identity,5);producer.Capture(run);const auto held=*producer.frame();
    Limits cap;cap.frame_bytes=producer.frame_bytes()-1;
    EXPECT_THROW((SourceAssemblyBinaryFrames(run,identity,5,cap)),std::runtime_error);
    EXPECT_THROW(producer.Capture(other),std::runtime_error);Same(held,*producer.frame());
    cap.frame_bytes=producer.frame_bytes();SourceAssemblyBinaryFrames exact(run,identity,5,cap);
    exact.Capture(run);Same(held,*exact.frame());
}
} // namespace crash::output::assembly::binary::test
