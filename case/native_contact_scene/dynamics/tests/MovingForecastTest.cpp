#include "Fixture.h"
#include <type_traits>
namespace crash::cases::native_scene::test {
TEST(NativeMovingDynamicsHost, DeclaredScopeSelectsTypedSourceAndRetainsExactRunIdentity) {
    SourceFixture fixed;SourceFixture moving("ROBO_DYNA_NATIVE_MOVING_SCENE_EXPORT");
    EXPECT_EQ(fixed.contact.kind(),ContactSelection::Kind::FixedWall);
    EXPECT_EQ(moving.contact.kind(),ContactSelection::Kind::MovingShells);
    EXPECT_TRUE(moving.contact.Visit([](const auto& selected) {
        return std::is_same_v<std::decay_t<decltype(selected.source())>,native::MovingMainSource>;
    }));
    const auto copied=moving.contact;EXPECT_EQ(copied.source().selection.mains,moving.contact.source().selection.mains);
    RunConfig config;config.dynamics=moving.config();config.run_id=908;config.steps=1000;config.samples=101;
    const auto run=PreparedNativeSceneRun::Prepare(moving.contact,moving.archive,config);
    const auto forecast=run.ForecastDocument();
    EXPECT_EQ(std::string(forecast["native_profile"].GetString()),moving.contact.profile_name());
    EXPECT_EQ(forecast["primary_mains"].GetUint64(),12u);EXPECT_EQ(forecast["expanded_mains"].GetUint64(),24u);
    EXPECT_EQ(forecast["secondaries"].GetUint64(),18u);EXPECT_EQ(forecast["fixed_dt_s"].GetDouble(),3e-7);
    EXPECT_EQ(std::string(forecast["declared_export_sha256"].GetString()),moving.physical.declared().data().export_sha256);
    const auto old=PreparedNativeSceneRun::Prepare(fixed.contact,fixed.archive,config).ForecastDocument();
    EXPECT_EQ(std::string(old["native_profile"].GetString()),"ordinary_fixed_main_all_active_ready_normals_local_single_worker");
}
TEST(NativeMovingDynamicsHost, SharedForecastIncludesCompleteSourceAndRejectsShortReservations) {
    SourceFixture moving("ROBO_DYNA_NATIVE_MOVING_SCENE_EXPORT");auto config=moving.config();
    const auto forecast=NativeSceneDynamics::Preflight(moving.contact,config);
    EXPECT_GT(forecast.source_host_bytes,moving.contact.forecast().owned_bytes);
    config.limits.host_bytes=forecast.peak_host_bytes;config.limits.device_bytes=forecast.device_bytes;
    EXPECT_NO_THROW(NativeSceneDynamics::Preflight(moving.contact,config));
    --config.limits.host_bytes;EXPECT_THROW(NativeSceneDynamics::Preflight(moving.contact,config),std::exception);
    config=moving.config();config.limits.device_bytes=forecast.device_bytes-1;
    EXPECT_THROW(NativeSceneDynamics::Preflight(moving.contact,config),std::exception);
}
}
