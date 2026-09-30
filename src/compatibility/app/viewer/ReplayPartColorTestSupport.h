#pragma once
#include "chrono/AcceptedReplayScene.h"
#include "output/AcceptedReplay.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <gtest/gtest.h>

namespace replay_color_test {
// Presentation-only synthetic fixture: two source parents and three triangles.
inline crash::output::ReplayInfo Info() {
    crash::output::ReplayInfo info;
    info.kind=crash::output::ReplayKind::SourceAssemblyWall;
    info.owner_id=9;info.final_epoch=2;info.final_time=.02;info.frame_count=3;
    info.node_count=5;info.triangle_count=3;info.bounds_min={-.1,-.05,0};info.bounds_max={.2,.05,.02};
    info.source_plasticity=true;info.plastic_strain_color_max=.02;
    info.triangle_source_parent={100,100,200};info.triangle_source_part={2000157,2000157,2000145};
    return info;
}
inline crash::output::ReplayFrame Frame(std::size_t index) {
    auto mesh=std::make_shared<chrono::ChTriangleMeshConnected>();
    const double z=.001*index;
    mesh->GetCoordsVertices()={{0,-.05,0},{.2,-.05,z},{.2,.05,z},{0,.05,0},{-.1,0,0}};
    mesh->GetIndicesVertices()={{0,1,2},{0,2,3},{0,3,4}};
    crash::output::ReplayFrame frame{index,9,index,.01*index,mesh};
    frame.parent_plastic_strain={{200,.01*index},{100,.005*index}};
    return frame;
}
inline void SameColor(const chrono::ChColor& a,const chrono::ChColor& b) {
    EXPECT_EQ(a.R,b.R);EXPECT_EQ(a.G,b.G);EXPECT_EQ(a.B,b.B);
}
inline void SameColors(const std::vector<chrono::ChColor>& a,const std::vector<chrono::ChColor>& b) {
    ASSERT_EQ(a.size(),b.size());for(std::size_t i=0;i<a.size();++i)SameColor(a[i],b[i]);
}
inline auto Initialize(crash::visual::AcceptedReplayScene& scene,
                       crash::visual::ReplayColorMode mode=crash::visual::ReplayColorMode::PartId) {
    return scene.Initialize(Info(),Frame(0),Frame(0).mesh,false,1,crash::visual::ReplayView::IncidentSide,mode);
}
} // namespace replay_color_test
