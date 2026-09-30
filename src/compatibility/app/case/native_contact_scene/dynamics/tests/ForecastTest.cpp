#include "Fixture.h"
#include <gtest/gtest.h>
namespace crash::cases::native_scene::test {
TEST(NativeSceneDynamicsHost, CompleteRuntimeBoundRejectsShortCapsBeforeCreatingOwner) {
    SourceFixture source;auto c=source.config();const auto f=NativeSceneDynamics::Preflight(source.contact,c);
    EXPECT_GT(f.owner.device_bytes,0u);EXPECT_GT(f.qeph.device_bytes,0u);EXPECT_GT(f.t3.device_bytes,0u);
    EXPECT_GT(f.roster.publication_host_bytes,0u);EXPECT_EQ(f.transaction_device_reservation,c.limits.contact.max_device_bytes);
    c.limits.host_bytes=f.peak_host_bytes;c.limits.device_bytes=f.device_bytes;
    EXPECT_NO_THROW(NativeSceneDynamics::Preflight(source.contact,c));
    --c.limits.host_bytes;EXPECT_THROW(NativeSceneDynamics::Preflight(source.contact,c),std::exception);
    c=source.config();c.limits.device_bytes=f.device_bytes-1;EXPECT_THROW(NativeSceneDynamics::Preflight(source.contact,c),std::exception);
    c=source.config();c.fixed_dt=4e-7;EXPECT_THROW(NativeSceneDynamics::Preflight(source.contact,c),std::exception);
}
TEST(NativeSceneDynamicsHost, RunPlanUsesActualSourceAndSharedExactHorizonWithoutDevice) {
    SourceFixture source;RunConfig c;c.dynamics=source.config();c.run_id=905;c.steps=1000;c.samples=31;
    const auto run=PreparedNativeSceneRun::Prepare(source.contact,source.archive,c);
    EXPECT_EQ(run.horizon().intervals,1000u);EXPECT_EQ(run.horizon().fixed_dt_s,3e-7);
    EXPECT_EQ(run.forecast().archive.archive.archive.frame_epochs.front(),0u);
    EXPECT_EQ(run.forecast().archive.archive.archive.frame_epochs.back(),1000u);
    c.steps=1001;EXPECT_THROW(PreparedNativeSceneRun::Prepare(source.contact,source.archive,c),std::exception);
    c.steps=1000;c.host_bytes=run.forecast().host_upper_bound-1;
    EXPECT_THROW(PreparedNativeSceneRun::Prepare(source.contact,source.archive,c),std::exception);
}
}
