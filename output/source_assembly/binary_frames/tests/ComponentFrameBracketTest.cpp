#include "ComponentFrameLiveSupport.h"
#include "case/source_assembly/tests/SourceBracketTestSupport.h"

namespace crash::output::assembly::binary::test {
TEST(ComponentBinaryLive, ActualSevenPartConnectorOwnerRetainsEverySourcePointAndAcceptedAttempt) {
    int devices=0;ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess);ASSERT_GT(devices,0);
    const auto bindings=fixture::prepared::BracketBindings();fixture::prepared::WallInput wall;
    cases::source_assembly::SourceAssemblyWallSetup setup;
    const auto ready=setup.Initialize(bindings,wall.canonical,wall.bytes,fixture::prepared::WallSettings());ASSERT_TRUE(ready)<<ready.message;
    dynamics::SourceAssemblyWallCase run;auto config=fixture::Configuration();config.fixed_dt*=4;config.observe_force_stage=true;
    const auto initialized=run.Initialize(bindings,setup,config);ASSERT_TRUE(initialized)<<initialized.message;
    ASSERT_NE(run.bindings()->connectors(),nullptr);ActualParity(run,1093,959);
}
} // namespace crash::output::assembly::binary::test
