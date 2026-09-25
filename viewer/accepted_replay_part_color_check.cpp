#include "ReplayPartColorTestSupport.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include <array>
#include <limits>

namespace {
using namespace replay_color_test;
using namespace crash::visual;

TEST(ReplayPartColor, VersionedPaletteUsesAllPidBitsAndStableKnownValues) {
    const std::array<std::uint64_t,5> ids{1,2000145,2000157,2000204,UINT64_MAX};
    // Independent Python colorsys HSV construction, rounded to binary32.
    const std::array<chrono::ChColor,5> expected{{
        {.5673576593399048f,.27250978350639343f,.8936470746994019f},
        {.827294111251831f,.1814207285642624f,.45098286867141724f},
        {.9225882291793823f,.7550219297409058f,.2865450382232666f},
        {.8392941355705261f,.15561500191688538f,.1765434592962265f},
        {.8527058959007263f,.1653246283531189f,.5257989764213562f}}};
    EXPECT_STREQ(ReplayPartPaletteName,"robo-dyna.original-part-id.v1");
    EXPECT_EQ(ReplayPartPaletteSeed,1u);
    for(std::size_t i=0;i<ids.size();++i)SameColor(ReplayPartColor(ids[i]),expected[i]);
    const auto low=ReplayPartColor(1),high=ReplayPartColor(UINT64_C(0x100000001));
    EXPECT_TRUE(low.R!=high.R||low.G!=high.G||low.B!=high.B);
}
TEST(ReplayPartColor, ExplicitSeedRetainsDefaultBitsAndAlternateColorsAreSourceStable) {
    for(auto id:{UINT64_C(1),UINT64_C(2),UINT64_C(2000145),UINT64_MAX})
        SameColor(ReplayPartColor(id),ReplayPartColor(id,1));
    // Independent Python colorsys construction using the same declared integer
    // mixing, then one binary32 rounding. No source-ID special case in product.
    SameColor(ReplayPartColor(1,2),{.9480000138282776f,.15524893999099731f,.2931748032569885f});
    SameColor(ReplayPartColor(2,2),{.175646111369133f,.49066266417503357f,.9148235321044922f});
    for(auto seed:{UINT64_C(0),UINT64_C(2),UINT64_MAX}) {
        ReplayPartColors first,subset;std::vector<chrono::ChColor> full,part;
        ASSERT_TRUE(first.Initialize({2,2000145,1,2},full,seed));
        ASSERT_TRUE(subset.Initialize({1,2},part,seed));
        SameColor(part[0],full[2]);SameColor(part[1],full[0]);SameColor(full[0],full[3]);
        const auto before=full;
        EXPECT_FALSE(first.Initialize({1,0},full,seed));SameColors(full,before);
        ASSERT_TRUE(first.Initialize({2,2000145,1,2},full,seed));SameColors(full,before);
    }
}
TEST(ReplayPartColor, SubsetsOrderRepeatedParentsAndInvalidRetryKeepPidMeaning) {
    ReplayPartColors colors;std::vector<chrono::ChColor> full;
    ASSERT_TRUE(colors.Initialize({2000157,2000145,2000157,2000204},full));
    ASSERT_EQ(colors.legend().size(),3u);EXPECT_EQ(colors.legend()[0].part_id,2000145u);
    std::vector<chrono::ChColor> subset;ReplayPartColors fewer;
    ASSERT_TRUE(fewer.Initialize({2000204,2000157},subset));
    SameColor(subset[0],full[3]);SameColor(subset[1],full[0]);SameColor(full[0],full[2]);
    const auto before=full;
    EXPECT_FALSE(colors.Initialize({2000157,0},full));SameColors(full,before);
    EXPECT_FALSE(colors.Initialize(std::vector<std::uint64_t>(ReplayPartColorTriangleLimit+1,1),full));SameColors(full,before);
    EXPECT_FALSE(colors.Initialize({},full));EXPECT_EQ(colors.legend().size(),3u);
    ASSERT_TRUE(colors.Initialize({2000145},full));SameColor(full[0],before[1]);
}
TEST(ReplayPartColor, FullShellTriangleCountAndMissingLastPidPreserveCompleteOutput) {
    // Actual full-shell render count, synthetic categorical IDs: this qualifies
    // palette capacity/mapping, not full-vehicle geometry or replay admission.
    constexpr std::size_t triangles=2*336976+21481;
    std::vector<std::uint64_t> ids(triangles);
    for(std::size_t i=0;i<triangles;++i)ids[i]=2000001+i%875;
    ReplayPartColors palette;std::vector<chrono::ChColor> colors;
    ASSERT_TRUE(palette.Initialize(ids,colors));ASSERT_EQ(colors.size(),triangles);
    ASSERT_EQ(palette.legend().size(),875u);
    for(std::size_t i=0;i<triangles;++i) {
        const auto expected=ReplayPartColor(ids[i]);
        if(colors[i].R!=expected.R||colors[i].G!=expected.G||colors[i].B!=expected.B)
            FAIL()<<"Source-scale mapping changed at triangle "<<i;
    }
    const auto before=colors;ids.back()=0;
    EXPECT_FALSE(palette.Initialize(ids,colors));ASSERT_EQ(colors.size(),before.size());
    for(std::size_t i=0;i<triangles;++i)
        if(colors[i].R!=before[i].R||colors[i].G!=before[i].G||colors[i].B!=before[i].B)
            FAIL()<<"Late invalid PID changed output at triangle "<<i;
    EXPECT_EQ(palette.legend().size(),875u);
}
TEST(ReplayPartColor, StrictOptionsAndUnsupportedMissingIdsFailWithoutScenePublication) {
    auto mode=ReplayColorMode::Automatic;
    for(const auto text:{"part-id","plastic-strain","uniform","auto"}) {
        ASSERT_TRUE(ParseReplayColorMode(text,mode));EXPECT_STREQ(ReplayColorModeName(mode),text);
    }
    for(const auto text:{"","parts","PART-ID","part-id ","plastic"}) {
        EXPECT_FALSE(ParseReplayColorMode(text,mode));EXPECT_EQ(mode,ReplayColorMode::Automatic);
    }
    for(unsigned fault=0;fault<4;++fault) {
        auto info=Info();AcceptedReplayScene scene;
        if(fault==0)info.triangle_source_part.clear();
        if(fault==1)info.triangle_source_part.back()=0;
        if(fault==2)info.triangle_source_part.push_back(2000145);
        if(fault==3)info.kind=crash::output::ReplayKind::ElasticCoupon;
        EXPECT_EQ(scene.Initialize(info,Frame(0),Frame(0).mesh,false,1,ReplayView::IncidentSide,
                                  ReplayColorMode::PartId).status,ReplaySceneStatus::InvalidFrame);
        EXPECT_EQ(scene.stamp(),nullptr);ASSERT_EQ(Initialize(scene).status,ReplaySceneStatus::Ok);
    }
    AcceptedReplayScene invalid;
    EXPECT_EQ(Initialize(invalid,static_cast<ReplayColorMode>(99)).status,ReplaySceneStatus::InvalidFrame);
}
TEST(ReplayPartColor, DefaultPlasticRemainsExactAndPartColorsRetainAllPhysicalGeometry) {
    AcceptedReplayScene automatic,plastic,part;
    ASSERT_EQ(Initialize(automatic,ReplayColorMode::Automatic).status,ReplaySceneStatus::Ok);
    ASSERT_EQ(Initialize(plastic,ReplayColorMode::PlasticStrain).status,ReplaySceneStatus::Ok);
    ASSERT_EQ(Initialize(part).status,ReplaySceneStatus::Ok);
    EXPECT_EQ(automatic.color_mode(),ReplayColorMode::PlasticStrain);
    const auto fixed=part.moving_mesh()->GetCoordsColors();const auto shape=part.moving_shape();
    EXPECT_EQ(shape->GetNumMaterials(),0);
    for(std::size_t i=0;i<3;++i) {
        auto frame=Frame(i);
        if(i)for(auto* scene:{&automatic,&plastic,&part})ASSERT_EQ(scene->Publish(frame).status,ReplaySceneStatus::Ok);
        SameColors(automatic.moving_mesh()->GetCoordsColors(),plastic.moving_mesh()->GetCoordsColors());
        SameColors(part.moving_mesh()->GetCoordsColors(),fixed);
        EXPECT_EQ(part.moving_mesh()->GetCoordsVertices(),frame.mesh->GetCoordsVertices());
        EXPECT_EQ(part.moving_mesh()->GetIndicesVertices(),frame.mesh->GetIndicesVertices());
        EXPECT_EQ(part.wall_mesh()->GetCoordsVertices(),Frame(0).mesh->GetCoordsVertices());
        EXPECT_EQ(part.camera()->position,automatic.camera()->position);
        EXPECT_EQ(part.moving_shape(),shape);EXPECT_EQ(part.stamp()->epoch,i);
    }
}
TEST(ReplayPartColor, HiddenPlasticAndLateGeometryFailuresStillPreserveAcceptedDisplay) {
    AcceptedReplayScene scene;ASSERT_EQ(Initialize(scene).status,ReplaySceneStatus::Ok);
    const auto positions=scene.moving_mesh()->GetCoordsVertices();
    const auto colors=scene.moving_mesh()->GetCoordsColors();
    for(unsigned fault=0;fault<3;++fault) {
        auto frame=Frame(1);
        if(fault==0)frame.parent_plastic_strain.back().value=std::numeric_limits<double>::quiet_NaN();
        if(fault==1)frame.parent_plastic_strain.back().source_parent=999;
        if(fault==2) {
            auto mesh=std::make_shared<chrono::ChTriangleMeshConnected>(*frame.mesh);
            mesh->GetCoordsVertices().back().x()=std::numeric_limits<double>::infinity();frame.mesh=mesh;
        }
        EXPECT_EQ(scene.Publish(frame).status,ReplaySceneStatus::InvalidFrame);
        EXPECT_EQ(scene.stamp()->epoch,0u);EXPECT_EQ(scene.moving_mesh()->GetCoordsVertices(),positions);
        SameColors(scene.moving_mesh()->GetCoordsColors(),colors);
    }
    ASSERT_EQ(scene.Publish(Frame(1)).status,ReplaySceneStatus::Ok);
}
} // namespace
