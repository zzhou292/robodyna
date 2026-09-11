#include "../EnvelopeWall.h"
#include "../RuntimeBudget.h"
#include <gtest/gtest.h>
#include <limits>
namespace crash::cases::vehicle_wall::test {
namespace c=tlfea::contact;
TEST(VehicleWallValues, PlacementUsesRepresentedOperationAndCompleteDeclaredEnvelope) {
    const std::array<c::Vec3,2> bounds{{{-2,-1,.01},{2,1,2}}};
    Settings settings;
    const auto p=Place(bounds,settings);
    EXPECT_EQ(p.translation_x_m,(bounds[1].x+settings.leading_gap_m)-.05);
    EXPECT_EQ(p.represented_wall_x_m,.05+p.translation_x_m);
    const auto gap=static_cast<long double>(p.represented_wall_x_m)-bounds[1].x;
    EXPECT_LE(static_cast<long double>(p.leading_gap.lower),gap);
    EXPECT_GE(static_cast<long double>(p.leading_gap.upper),gap);
    const auto travel=static_cast<long double>(settings.initial_speed_mps)*settings.requested_duration_s;
    EXPECT_LE(static_cast<long double>(p.nominal_forward_travel.lower),travel);
    EXPECT_GE(static_cast<long double>(p.nominal_forward_travel.upper),travel);
    EXPECT_LT(p.declared_world_envelope.minimum.y,bounds[0].y);
    EXPECT_GT(p.declared_world_envelope.maximum.z,bounds[1].z);
    EXPECT_GT(p.declared_world_envelope.maximum.x,bounds[1].x+double(travel));
    EXPECT_EQ(p.projected_wall_box.minimum.x,p.represented_wall_x_m);
    EXPECT_EQ(p.projected_wall_box.maximum.x,p.represented_wall_x_m);
}
TEST(VehicleWallValues, GeneratedMeshCoversWiderAndEdgeOnEnvelopesWithDistinctFeatureIdentity) {
    for (auto bounds : {std::array<c::Vec3,2>{{{-2,-2,-.2},{2,2,3}}},
                        std::array<c::Vec3,2>{{{0,0,0},{.05,.02,0}}}}) {
        Settings settings;
        const auto placement=Place(bounds,settings);
        auto mesh=EnvelopeWall::Prepare(placement,settings);
        auto moved=std::move(mesh);
        const auto view=moved.view();
        ASSERT_EQ(view.vertex_count,4);
        ASSERT_EQ(view.triangle_count,2);
        for (std::size_t n=0;n<view.vertex_count;++n) {
            EXPECT_EQ(view.vertices[n].position.x,placement.represented_wall_x_m);
            EXPECT_GE(view.vertices[n].source_node_id,0xd700000000000000ULL);
        }
        c::PlanarWallBoxCoverage out;
        ASSERT_EQ(c::CheckPlanarWallBox(moved.geometry(),placement.projected_wall_box,settings.exposed_clearance_m,
            settings.wall_binding_id,c::PlanarWallBoxMode::ConservativeExpansion,&out).status,c::PlanarContactStatus::Ok);
        EXPECT_TRUE(out.covered);
        EXPECT_EQ(view.triangles[0].source_quad_id,view.triangles[1].source_quad_id);
        for (const auto& face : moved.geometry().faces()) {
            const auto& v=face.geometry.vertices;
            EXPECT_LT((v[1].y-v[0].y)*(v[2].z-v[0].z)-(v[1].z-v[0].z)*(v[2].y-v[0].y),0);
        }
    }
}
TEST(VehicleWallValues, InvalidSettingsAndLateOverflowPreserveExistingMesh) {
    const std::array<c::Vec3,2> bounds{{{0,-1,0},{2,1,2}}};
    Settings settings;
    auto mesh=EnvelopeWall::Prepare(Place(bounds,settings),settings);
    const auto before=mesh.view().vertices[3].position;
    settings.mesh_profile=static_cast<WallMeshProfile>(99);
    EXPECT_THROW(CheckSettings(settings),std::runtime_error);
    settings={};settings.initial_speed_mps=35;
    EXPECT_THROW(CheckSettings(settings),std::runtime_error);
    settings={};settings.stiffness_per_area=std::numeric_limits<double>::infinity();
    EXPECT_THROW(CheckSettings(settings),std::runtime_error);
    settings={};auto bad=Place(bounds,settings);
    bad.projected_wall_box.maximum.y=std::numeric_limits<double>::max();
    settings.transverse_margin_m=std::numeric_limits<double>::max();
    EXPECT_THROW(mesh=EnvelopeWall::Prepare(bad,settings),std::runtime_error);
    EXPECT_EQ(mesh.view().vertices[3].position.y,before.y);
    EXPECT_EQ(mesh.view().vertices[3].position.z,before.z);
}
TEST(VehicleWallValues, CompleteBudgetChargesRetainedOnceAndMaximumScratchWithExactRetry) {
    vehicle_dynamics::Forecast dynamics;
    dynamics.startup.retained_source_upper_bound=1000;
    dynamics.startup.retained_host_upper_bound=1400;
    dynamics.startup.device_bytes=100;
    dynamics.startup.peak_temporary_bytes=300;
    dynamics.workspace_bytes=20;
    SetupForecast setup;
    setup.shared_source_upper_bound=1000;
    setup.retained_setup_bytes=50;
    setup.geometry.temporary_bytes=500;
    tl::fea::ShellMappedFootprint contact{10,900,60,200,1160};
    auto limits=RuntimeLimits{};
    const auto result=detail::ComposeForecast(dynamics,setup,contact,7,limits);
    EXPECT_EQ(result.retained_host_upper_bound,1537);
    EXPECT_EQ(result.peak_host_upper_bound,2037);
    EXPECT_EQ(result.device_bytes,110);
    limits.host_bytes=2037;limits.device_bytes=110;
    EXPECT_EQ(detail::ComposeForecast(dynamics,setup,contact,7,limits).device_bytes,110);
    --limits.host_bytes;
    EXPECT_THROW(detail::ComposeForecast(dynamics,setup,contact,7,limits),std::runtime_error);
    ++limits.host_bytes;--limits.device_bytes;
    EXPECT_THROW(detail::ComposeForecast(dynamics,setup,contact,7,limits),std::runtime_error);
    ++limits.device_bytes;
    EXPECT_EQ(detail::ComposeForecast(dynamics,setup,contact,7,limits).peak_host_upper_bound,2037);
    ++setup.shared_source_upper_bound;
    EXPECT_THROW(detail::ComposeForecast(dynamics,setup,contact,7,limits),std::runtime_error);
}
} // namespace crash::cases::vehicle_wall::test
