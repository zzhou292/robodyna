#include "chrono/full_shell/FrameGeometryState.h"
#include "chrono/ReplayVisuals.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/physics/ChBody.h"
#include "output/full_shell/tests/TestSupport.h"
#include <gtest/gtest.h>
#include <limits>
namespace crash::visual::physical_run::test {
namespace records=output::full_shell;
TEST(PhysicalSceneValues, VariableFaceShapeRetainsZeroOneThreeFourPointRolesAndLateRollback) {
    const records::ParentPoints rows[]{{11,101,2,1,0,records::PlasticField::NotApplicable},
        {12,102,3,1,1,records::PlasticField::NativeEquivalentPlasticStrain},
        {13,103,2,1,3,records::PlasticField::NativeEquivalentPlasticStrain},
        {14,104,9,1,4,records::PlasticField::NativeEquivalentPlasticStrain}};
    const auto context=records::Context::Create(records::test::Id(),4,rows,4,.125);
    full_shell::detail::FrameGeometryState state(context);
    state.options.colors=ReplayColorMode::PartId;
    state.options.parent_activity=true;
    state.options.plastic_strain_maximum=1;
    state.budget=full_shell::FrameGeometryBudget(4,4,7,state.options);
    full_shell::detail::InitializeTopology(state,{0,1,2,0,2,3, 0,1,2, 0,1,2,0,2,3, 0,1,2,0,2,3},
        {0,0,1,2,2,3,3});
    records::FrameRecord frame{{},{-0.,0,0,1,0,0,1,1,0,0,1,0},std::vector<double>(8)};
    const auto active=[&](std::vector<std::uint8_t> flags) {
        return records::activity::ActivityRecord::Create(context,
            {context.identity(),context.point_layout_sha256(),frame.stamp,flags.data(),flags.size()},frame.stamp);
    };
    auto flags=active({1,1,1,1});
    ASSERT_EQ(full_shell::detail::UpdateFrame(state,frame,&flags,frame.stamp).status,ReplaySceneStatus::Ok);
    auto shape=MakeReplayShape(state.mesh,true,false,true,false);
    auto body=MakeReplayCarrier("physical display",shape);
    EXPECT_FALSE(shape->IsFixedConnectivity());EXPECT_TRUE(shape->IsMutable());
    EXPECT_TRUE(body->IsFixed());EXPECT_EQ(state.fields.size(),4u);
    const auto legend=state.part_colors.legend();
    frame.stamp={1,0,4,.125,0,.0625,.0625};
    frame.plastic_points.back()=.42;const auto subset=active({1,0,1,0});
    ASSERT_EQ(full_shell::detail::UpdateFrame(state,frame,&subset,frame.stamp).status,ReplaySceneStatus::Ok);
    EXPECT_EQ(state.mesh.get(),shape->GetMesh().get());
    EXPECT_EQ(state.mesh->GetNumTriangles(),4u);
    EXPECT_EQ(state.triangle_parents,(std::vector<std::uint64_t>{11,11,13,13}));
    EXPECT_EQ(state.part_colors.legend().size(),legend.size());
    const auto positions=state.mesh->GetCoordsVertices();
    const auto stamp=state.stamp;
    frame.plastic_points.back()=std::numeric_limits<double>::quiet_NaN();
    EXPECT_NE(full_shell::detail::UpdateFrame(state,frame,&subset,frame.stamp).status,ReplaySceneStatus::Ok);
    EXPECT_EQ(state.stamp.attempt,stamp.attempt);EXPECT_EQ(state.mesh->GetNumTriangles(),4u);
    EXPECT_EQ(state.mesh->GetCoordsVertices(),positions);
    frame.plastic_points.back()=.42;const auto none=active({0,0,0,0});
    ASSERT_EQ(full_shell::detail::UpdateFrame(state,frame,&none,frame.stamp).status,ReplaySceneStatus::Ok);
    EXPECT_EQ(state.mesh->GetNumTriangles(),0u);
    const auto restored=active({1,1,1,1});
    ASSERT_EQ(full_shell::detail::UpdateFrame(state,frame,&restored,frame.stamp).status,ReplaySceneStatus::Ok);
    EXPECT_EQ(state.mesh->GetNumTriangles(),7u);
    EXPECT_TRUE(MakeReplayShape(state.mesh,true,false,true)->IsFixedConnectivity());
}
} // namespace crash::visual::physical_run::test
