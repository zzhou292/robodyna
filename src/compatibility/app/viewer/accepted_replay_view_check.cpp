#include "chrono/AcceptedReplayScene.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "output/AcceptedReplay.h"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>

namespace {
using crash::output::ReplayKind;
using crash::visual::AcceptedReplayScene;
using crash::visual::ReplaySceneStatus;
using crash::visual::ReplayView;

struct Fixture {
    crash::output::ReplayInfo info;
    crash::output::ReplayFrame initial;
    std::shared_ptr<chrono::ChTriangleMeshConnected> wall;
    explicit Fixture(ReplayKind kind) {
        auto mesh = std::make_shared<chrono::ChTriangleMeshConnected>();
        mesh->GetCoordsVertices() = {{0,-.05,0},{.2,-.05,0},{.2,.05,0},{0,.05,0},{-.1,0,0}};
        mesh->GetIndicesVertices() = {{0,1,2},{0,2,3},{0,3,4}};
        info.kind=kind; info.owner_id=9; info.final_epoch=1; info.final_time=.01;
        info.frame_count=2; info.node_count=5; info.triangle_count=3;
        info.bounds_min={-.1,-.05,-.002}; info.bounds_max={.2,.05,.002};
        initial={0,9,0,0,mesh};
        if (kind!=ReplayKind::ElasticCoupon && kind!=ReplayKind::SourcePartElastic) {
            wall=std::make_shared<chrono::ChTriangleMeshConnected>();
            wall->GetCoordsVertices()={{.201,-1,-1},{.201,1,-1},{.201,1,1},{.201,-1,1}};
            wall->GetIndicesVertices()={{0,2,1},{0,3,2}};
        }
    }
};
void Exact(const std::array<double,3>& actual,const std::array<double,3>& expected) {
    EXPECT_EQ(std::memcmp(actual.data(),expected.data(),3*sizeof(double)),0);
}

TEST(AcceptedReplayView, StrictNamesRejectInvalidOptionsWithoutChangingSelection) {
    ReplayView view=ReplayView::IncidentSide;
    EXPECT_STREQ(crash::visual::ReplayViewName(view),"incident-side");
    ASSERT_TRUE(crash::visual::ParseReplayView("wall-side",view));
    EXPECT_EQ(view,ReplayView::WallSide);
    EXPECT_STREQ(crash::visual::ReplayViewName(view),"wall-side");
    for (const char* invalid:{"","Wall-side","wall","wall-side "," incident-side","1"}) {
        EXPECT_FALSE(crash::visual::ParseReplayView(invalid,view));
        EXPECT_EQ(view,ReplayView::WallSide);
    }
    ASSERT_TRUE(crash::visual::ParseReplayView("incident-side",view));
    EXPECT_EQ(view,ReplayView::IncidentSide);
    EXPECT_EQ(crash::visual::ReplayViewName(static_cast<ReplayView>(99)),nullptr);
}

TEST(AcceptedReplayView, DefaultAndExplicitIncidentCamerasMatchOriginalArithmeticExactly) {
    for (auto kind:{ReplayKind::NormalImpact,ReplayKind::ElasticCoupon,ReplayKind::GuidedPlate,
                   ReplayKind::SourcePartElastic,ReplayKind::SourcePartWall,ReplayKind::SourceAssemblyWall}) {
        SCOPED_TRACE(static_cast<int>(kind));
        Fixture f(kind); AcceptedReplayScene implicit,explicit_view;
        ASSERT_EQ(implicit.Initialize(f.info,f.initial,f.wall).status,ReplaySceneStatus::Ok);
        ASSERT_EQ(explicit_view.Initialize(f.info,f.initial,f.wall,false,1,ReplayView::IncidentSide).status,ReplaySceneStatus::Ok);
        // Frozen pre-option camera formula: compare binary64 bytes, including
        // the original midpoint and multiply/add evaluation order.
        std::array<double,3> target{},extent{},position{},direction{};
        for (unsigned a=0;a<3;++a) {
            target[a]=f.info.bounds_min[a]*.5+f.info.bounds_max[a]*.5;
            extent[a]=f.info.bounds_max[a]-f.info.bounds_min[a];
        }
        const bool source=kind==ReplayKind::SourcePartWall||kind==ReplayKind::SourceAssemblyWall;
        direction=source?std::array<double,3>{-1,-1,.15}:kind==ReplayKind::GuidedPlate
            ?std::array<double,3>{-1,-.15,-1}:kind==ReplayKind::ElasticCoupon
            ?std::array<double,3>{-.3,-1.25,.18}:std::array<double,3>{-1.5,-1.25,1};
        const double diagonal=std::hypot(extent[0],extent[1],extent[2]);
        for (unsigned a=0;a<3;++a) position[a]=target[a]+(source?1.25:1.6)*diagonal*direction[a];
        for (const auto* scene:{&implicit,&explicit_view}) {
            Exact(scene->camera()->target,target); Exact(scene->camera()->position,position);
            EXPECT_EQ(scene->camera()->vertical,kind==ReplayKind::GuidedPlate
                ?crash::visual::ReplayVertical::Y:crash::visual::ReplayVertical::Z);
            EXPECT_EQ(scene->camera()->vertical_fov_degrees,40);
            EXPECT_EQ(scene->camera()->view,ReplayView::IncidentSide);
            EXPECT_EQ(scene->moving_mesh()->GetCoordsVertices(),f.initial.mesh->GetCoordsVertices());
        }
    }
}

TEST(AcceptedReplayView, OppositeSourceViewRetainsEveryPhysicalVertexTriangleAndPlasticColor) {
    for (auto kind:{ReplayKind::SourcePartWall,ReplayKind::SourceAssemblyWall}) {
        SCOPED_TRACE(static_cast<int>(kind)); Fixture f(kind);
        f.info.source_plasticity=true; f.info.triangle_source_parent={100,100,200};
        f.info.plastic_strain_color_max=.02; f.initial.parent_plastic_strain={{200,0},{100,0}};
        AcceptedReplayScene incident,opposite;
        ASSERT_EQ(incident.Initialize(f.info,f.initial,f.wall).status,ReplaySceneStatus::Ok);
        ASSERT_EQ(opposite.Initialize(f.info,f.initial,f.wall,false,1,ReplayView::WallSide).status,ReplaySceneStatus::Ok);
        const auto a=*incident.camera(),b=*opposite.camera();
        EXPECT_EQ(b.view,ReplayView::WallSide); Exact(a.target,b.target);
        EXPECT_LT(a.position[0],a.target[0]); EXPECT_GT(b.position[0],b.target[0]);
        EXPECT_EQ(b.position[0]-b.target[0],a.target[0]-a.position[0]);
        EXPECT_GT(b.position[0],f.wall->GetCoordsVertices()[0].x());
        EXPECT_EQ(a.position[1],b.position[1]); EXPECT_EQ(a.position[2],b.position[2]);
        EXPECT_EQ(a.vertical,b.vertical); EXPECT_EQ(a.vertical_fov_degrees,b.vertical_fov_degrees);
        // The complete trajectory bounding sphere fits the vertical field of
        // view (and thus the wider 1280x720 horizontal field) from either side.
        const auto& lo=f.info.bounds_min; const auto& hi=f.info.bounds_max;
        const double radius=.5*std::hypot(hi[0]-lo[0],hi[1]-lo[1],hi[2]-lo[2]);
        for (const auto* camera:{&a,&b}) {
            const double distance=std::hypot(camera->position[0]-camera->target[0],
                camera->position[1]-camera->target[1],camera->position[2]-camera->target[2]);
            EXPECT_LT(std::asin(radius/distance),camera->vertical_fov_degrees*std::acos(-1.)/360);
        }
        auto deformed=std::make_shared<chrono::ChTriangleMeshConnected>(*f.initial.mesh);
        deformed->GetCoordsVertices().back().z()=.002;
        auto next=f.initial; next.index=1; next.epoch=1; next.time=.01; next.mesh=deformed;
        next.parent_plastic_strain={{200,.02},{100,.01}};
        for (auto* scene:{&incident,&opposite}) {
            const auto moving=scene->moving_mesh(); const auto fixed=scene->wall_mesh();
            EXPECT_EQ(moving->GetCoordsVertices(),f.initial.mesh->GetCoordsVertices());
            ASSERT_EQ(scene->Publish(next).status,ReplaySceneStatus::Ok);
            EXPECT_EQ(scene->moving_mesh(),moving); EXPECT_EQ(scene->wall_mesh(),fixed);
            EXPECT_EQ(moving->GetCoordsVertices(),deformed->GetCoordsVertices());
            EXPECT_EQ(moving->GetIndicesVertices(),f.initial.mesh->GetIndicesVertices());
            EXPECT_EQ(fixed->GetCoordsVertices(),f.wall->GetCoordsVertices());
            EXPECT_EQ(fixed->GetIndicesVertices(),f.wall->GetIndicesVertices());
            EXPECT_EQ(scene->deformation_scale(),1); EXPECT_EQ(scene->stamp()->epoch,1u);
        }
        const auto& ca=incident.moving_mesh()->GetCoordsColors();
        const auto& cb=opposite.moving_mesh()->GetCoordsColors();
        ASSERT_EQ(ca.size(),3u); ASSERT_EQ(cb.size(),ca.size());
        for (std::size_t i=0;i<ca.size();++i) {
            EXPECT_EQ(ca[i].R,cb[i].R); EXPECT_EQ(ca[i].G,cb[i].G); EXPECT_EQ(ca[i].B,cb[i].B);
        }
        Exact(incident.camera()->position,a.position); Exact(opposite.camera()->position,b.position);
    }
}

TEST(AcceptedReplayView, UnsupportedKindsAndInvalidEnumRejectBeforePublicationAndPermitRetry) {
    for (auto kind:{ReplayKind::NormalImpact,ReplayKind::ElasticCoupon,ReplayKind::GuidedPlate,ReplayKind::SourcePartElastic}) {
        Fixture f(kind); AcceptedReplayScene scene;
        EXPECT_EQ(scene.Initialize(f.info,f.initial,f.wall,false,1,ReplayView::WallSide).status,ReplaySceneStatus::InvalidFrame);
        EXPECT_EQ(scene.camera(),nullptr); EXPECT_EQ(scene.stamp(),nullptr); EXPECT_FALSE(scene.moving_mesh());
        ASSERT_EQ(scene.Initialize(f.info,f.initial,f.wall).status,ReplaySceneStatus::Ok);
    }
    Fixture f(ReplayKind::SourceAssemblyWall); AcceptedReplayScene scene;
    EXPECT_EQ(scene.Initialize(f.info,f.initial,f.wall,false,1,static_cast<ReplayView>(99)).status,ReplaySceneStatus::InvalidFrame);
    EXPECT_EQ(scene.camera(),nullptr); EXPECT_EQ(scene.stamp(),nullptr);
    ASSERT_EQ(scene.Initialize(f.info,f.initial,f.wall,false,1,ReplayView::WallSide).status,ReplaySceneStatus::Ok);
}
}  // namespace
