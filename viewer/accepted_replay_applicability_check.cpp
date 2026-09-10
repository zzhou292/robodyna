#include "ReplayPartColorTestSupport.h"
#include "chrono/ReplayScalarApplicability.h"

namespace {
using namespace replay_color_test;
using namespace crash::visual;
using A=crash::output::ReplayScalarApplicability;
TEST(ReplayScalarScene,TypedMissingFieldsKeepDistinctColorsAndRejectChangedApplicability) {
    for(auto missing:{A::NotApplicable,A::Unavailable})for(auto mode:{ReplayColorMode::PartId,ReplayColorMode::PlasticStrain}) {
        auto first=Frame(0);first.parent_plastic_strain.front().applicability=missing;
        AcceptedReplayScene scene;
        ASSERT_EQ(scene.Initialize(Info(),first,first.mesh,false,1,ReplayView::IncidentSide,mode).status,ReplaySceneStatus::Ok);
        ASSERT_NE(scene.scalar_legend(),nullptr);EXPECT_EQ(scene.scalar_legend()->native,1u);
        EXPECT_EQ(scene.scalar_legend()->not_applicable,missing==A::NotApplicable?1u:0u);
        EXPECT_EQ(scene.scalar_legend()->unavailable,missing==A::Unavailable?1u:0u);
        const auto mesh=scene.moving_mesh();const auto colors=mesh->GetCoordsColors();const auto positions=mesh->GetCoordsVertices();
        if(mode==ReplayColorMode::PlasticStrain)SameColor(colors.back(),ReplayMissingScalarColor(missing));
        else SameColor(colors.back(),ReplayPartColor(2000145));
        auto next=Frame(1);next.parent_plastic_strain.front()={200,0,missing};
        auto wrong=next;wrong.parent_plastic_strain.back().applicability=A::Unavailable;wrong.parent_plastic_strain.back().value=0;
        EXPECT_EQ(scene.Publish(wrong).status,ReplaySceneStatus::InvalidFrame);
        SameColors(mesh->GetCoordsColors(),colors);EXPECT_EQ(scene.stamp()->epoch,0u);
        for(std::size_t n=0;n<positions.size();++n)for(unsigned j=0;j<3;++j)EXPECT_EQ(mesh->GetCoordsVertices()[n][j],positions[n][j]);
        ASSERT_EQ(scene.Publish(next).status,ReplaySceneStatus::Ok);
        EXPECT_EQ(scene.moving_mesh().get(),mesh.get());EXPECT_EQ(scene.stamp()->epoch,1u);
    }
}
TEST(ReplayScalarScene,ExistingPlasticReplayCannotClaimAnAllMissingNativeField) {
    auto first=Frame(0);
    for(auto& f:first.parent_plastic_strain)f.applicability=A::Unavailable;
    auto info=Info();info.plastic_strain_color_max=0;
    AcceptedReplayScene scene;
    EXPECT_EQ(scene.Initialize(info,first,first.mesh).status,ReplaySceneStatus::InvalidFrame);
    EXPECT_EQ(scene.stamp(),nullptr);
    EXPECT_EQ(Initialize(scene).status,ReplaySceneStatus::Ok);
}
} // namespace
