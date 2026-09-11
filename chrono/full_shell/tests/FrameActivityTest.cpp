#include "../FrameGeometryState.h"
#include "ColorChecks.h"
#include "output/full_shell/tests/TestSupport.h"
#include <limits>

namespace crash::visual::full_shell::test {
namespace fs = output::full_shell;
namespace activity = fs::activity;
namespace {
const std::vector<std::uint32_t> Triangles{0,1,2, 0,2,3, 4,5,6, 0,1,2, 0,2,3};
const std::vector<std::uint32_t> Parents{2,2,1,0,0};
fs::Context Context() {
    const fs::ParentPoints parents[]{
        {9,800,2,1,3,fs::PlasticField::NativeEquivalentPlasticStrain},
        {1,90,2,2,0,fs::PlasticField::NotApplicable},
        {7,77,2,1,0,fs::PlasticField::Unavailable}};
    return fs::Context::Create(fs::test::Id(),7,parents,3,.125);
}
fs::FrameRecord Frame(bool later = false) {
    return {later ? fs::FrameStamp{1,0,7,.125,0,.0625,.0625} : fs::FrameStamp{},
        {-0.,0,0, 1,0,0, 1,1,0, 0,1,0, 2,0,0, 3,0,0, 2,1,0},
        {0,.2,later ? .7 : .4}};
}
activity::ActivityRecord Activity(const fs::Context& c, const fs::FrameStamp& stamp,
        const std::vector<std::uint8_t>& flags) {
    return activity::ActivityRecord::Create(c,
        {c.identity(),c.point_layout_sha256(),stamp,flags.data(),flags.size()},stamp);
}
void Initialize(detail::FrameGeometryState& state, ReplayColorMode mode, bool enabled = true) {
    state.options.colors = mode;
    state.options.parent_activity = enabled;
    state.options.plastic_strain_maximum = 1;
    state.budget = FrameGeometryBudget(7,3,5,state.options);
    detail::InitializeTopology(state,Triangles,Parents);
}
void ExactVertices(const std::vector<chrono::ChVector3d>& a,const std::vector<chrono::ChVector3d>& b) {
    ASSERT_EQ(a.size(),b.size());
    for(std::size_t i=0;i<a.size();++i) for(unsigned j=0;j<3;++j) {
        EXPECT_EQ(a[i][j],b[i][j]);
        EXPECT_EQ(std::signbit(a[i][j]),std::signbit(b[i][j]));
    }
}
void ExactIndices(const std::vector<chrono::ChVector3i>& a,const std::vector<chrono::ChVector3i>& b) {
    ASSERT_EQ(a.size(),b.size());
    for(std::size_t i=0;i<a.size();++i) for(unsigned j=0;j<3;++j) EXPECT_EQ(a[i][j],b[i][j]);
}
}
TEST(FullShellFrameActivity, MixedQuadsTriangleAndCoincidentLayersKeepCompletePaletteAndSourceOrder) {
    const auto context=Context();
    const auto initial=Frame(),later=Frame(true);
    const auto all=Activity(context,initial.stamp,{1,1,1});
    const auto subset=Activity(context,later.stamp,{0,1,1});
    for(auto mode:{ReplayColorMode::PartId,ReplayColorMode::PlasticStrain,ReplayColorMode::Uniform}) {
        detail::FrameGeometryState state(context);
        Initialize(state,mode);
        ASSERT_EQ(detail::UpdateFrame(state,initial,&all,initial.stamp).status,ReplaySceneStatus::Ok);
        const auto mesh=state.mesh;
        const auto colors=mesh->GetCoordsColors();
        const auto legend=state.part_colors.legend();
        ASSERT_EQ(detail::UpdateFrame(state,later,&subset,later.stamp).status,ReplaySceneStatus::Ok);
        EXPECT_EQ(state.mesh.get(),mesh.get());
        EXPECT_EQ(state.triangle_parents,(std::vector<std::uint64_t>{7,7,1}));
        EXPECT_EQ(state.triangle_parts,(std::vector<std::uint64_t>{77,77,90}));
        ASSERT_EQ(mesh->GetIndicesVertices().size(),3u);
        if(mode!=ReplayColorMode::Uniform) {
            ASSERT_EQ(mesh->GetIndicesColors().size(),3u);
            EXPECT_EQ(mesh->GetCoordsColors().size(),5u);
            for(unsigned t=0;t<3;++t) EXPECT_EQ(mesh->GetIndicesColors()[t][0],int(t));
        }
        if(mode==ReplayColorMode::PartId) {
            ExpectColors(mesh->GetCoordsColors(),colors);
            ASSERT_EQ(state.part_colors.legend().size(),legend.size());
            for(std::size_t i=0;i<legend.size();++i) {
                EXPECT_EQ(state.part_colors.legend()[i].part_id,legend[i].part_id);
                ExpectColor(state.part_colors.legend()[i].color,legend[i].color);
            }
        }
        EXPECT_EQ(state.fields[0].value,.7); // Hidden native field remains a checked source value.
        const auto last_only=Activity(context,later.stamp,{1,0,0});
        ASSERT_EQ(detail::UpdateFrame(state,later,&last_only,later.stamp).status,ReplaySceneStatus::Ok);
        EXPECT_EQ(state.triangle_parents,(std::vector<std::uint64_t>{9,9}));
        if(mode!=ReplayColorMode::Uniform) {
            ASSERT_EQ(mesh->GetIndicesColors().size(),2u);
            EXPECT_EQ(mesh->GetIndicesColors()[0][0],3);
            EXPECT_EQ(mesh->GetIndicesColors()[1][0],4);
            const auto expected=mode==ReplayColorMode::PartId ? ReplayPartColor(800) : ReplayScalarColor(.7);
            ExpectColor(mesh->GetCoordsColors()[3],expected);
            ExpectColor(mesh->GetCoordsColors()[4],expected);
        }
        ASSERT_EQ(detail::UpdateFrame(state,initial,&all,initial.stamp).status,ReplaySceneStatus::Ok);
        EXPECT_EQ(state.triangle_parents,(std::vector<std::uint64_t>{7,7,1,9,9}));
        EXPECT_EQ(mesh->GetIndicesVertices().size(),5u);
        ExpectColors(mesh->GetCoordsColors(),colors);
    }
}
TEST(FullShellFrameActivity, AllInactiveAllowsEmptyFacesAndCollapsedHiddenParentThenSeekRestores) {
    const auto context=Context();
    detail::FrameGeometryState state(context);
    Initialize(state,ReplayColorMode::PartId);
    const auto initial=Frame();
    const auto all=Activity(context,initial.stamp,{1,1,1});
    ASSERT_EQ(detail::UpdateFrame(state,initial,&all,initial.stamp).status,ReplaySceneStatus::Ok);
    auto collapsed=Frame(true);
    std::fill(collapsed.position_xyz.begin(),collapsed.position_xyz.end(),0);
    const auto hidden=Activity(context,collapsed.stamp,{0,0,0});
    ASSERT_EQ(detail::UpdateFrame(state,collapsed,&hidden,collapsed.stamp).status,ReplaySceneStatus::Ok);
    EXPECT_TRUE(state.visible);
    EXPECT_TRUE(state.mesh->GetIndicesVertices().empty());
    EXPECT_TRUE(state.mesh->GetIndicesColors().empty());
    EXPECT_TRUE(state.triangle_parents.empty());
    EXPECT_EQ(state.part_colors.legend().size(),3u);
    collapsed.position_xyz.back()=std::numeric_limits<double>::max();
    EXPECT_EQ(detail::UpdateFrame(state,collapsed,&hidden,collapsed.stamp).status,ReplaySceneStatus::InvalidFrame);
    EXPECT_TRUE(state.mesh->GetIndicesVertices().empty());
    ASSERT_EQ(detail::UpdateFrame(state,initial,&all,initial.stamp).status,ReplaySceneStatus::Ok);
    EXPECT_EQ(state.mesh->GetIndicesVertices().size(),5u);
}
TEST(FullShellFrameActivity, ActivityIdentityPhaseAndLateGeometryFailurePreserveEveryVisibleValueThenRetry) {
    const auto context=Context();
    detail::FrameGeometryState state(context);
    Initialize(state,ReplayColorMode::PlasticStrain);
    const auto first=Frame();
    const auto all=Activity(context,first.stamp,{1,1,1});
    ASSERT_EQ(detail::UpdateFrame(state,first,&all,first.stamp).status,ReplaySceneStatus::Ok);
    const auto faces=state.mesh->GetIndicesVertices(),color_indices=state.mesh->GetIndicesColors();
    const auto positions=state.mesh->GetCoordsVertices();
    const auto colors=state.mesh->GetCoordsColors();
    auto next=Frame(true);
    const auto active=Activity(context,next.stamp,{0,1,1});
    for(unsigned fault=0;fault<5;++fault) {
        auto parents=context.parents();
        auto id=context.identity();
        if(fault==0) ++id.owner;
        if(fault==1) id.source_mapping_sha256[0]='c';
        if(fault==2) std::swap(parents[0],parents[2]);
        const auto other=fs::Context::Create(id,context.nodes(),parents.data(),parents.size(),fault==3?.25:.125);
        const auto wrong=Activity(other,fault==3?fs::FrameStamp{}:next.stamp,{1,0,1});
        EXPECT_EQ(detail::UpdateFrame(state,next,fault==4?&all:&wrong,next.stamp).status,ReplaySceneStatus::InvalidFrame);
    }
    next.position_xyz[18]=3; next.position_xyz[19]=0; // Last active T3 vertex duplicates its second vertex.
    EXPECT_EQ(detail::UpdateFrame(state,next,&active,next.stamp).status,ReplaySceneStatus::InvalidFrame);
    next=Frame(true);
    next.plastic_points.back()=std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(detail::UpdateFrame(state,next,&active,next.stamp).status,ReplaySceneStatus::InvalidFrame);
    ExactIndices(state.mesh->GetIndicesVertices(),faces);
    ExactIndices(state.mesh->GetIndicesColors(),color_indices);
    ExactVertices(state.mesh->GetCoordsVertices(),positions);
    ExpectColors(state.mesh->GetCoordsColors(),colors);
    EXPECT_EQ(state.triangle_parents,(std::vector<std::uint64_t>{7,7,1,9,9}));
    EXPECT_EQ(state.triangle_parts,(std::vector<std::uint64_t>{77,77,90,800,800}));
    EXPECT_EQ(state.fields[0].value,.4);
    EXPECT_TRUE(fs::SameStamp(state.stamp,first.stamp));
    next=Frame(true);
    ASSERT_EQ(detail::UpdateFrame(state,next,&active,next.stamp).status,ReplaySceneStatus::Ok);
}
TEST(FullShellFrameActivity, ExplicitChannelAndExactFullCountBudgetPreserveLegacyAdmission) {
    FrameGeometryOptions options;
    options.geometry=ReplayGeometryLimits::Vehicle();
    const auto legacy=FrameGeometryBudget(359785,349645,677989,options);
    options.parent_activity=true;
    const auto bytes=FrameGeometryBudget(359785,349645,677989,options);
    EXPECT_EQ(bytes,legacy+64u*677989);
    EXPECT_LT(bytes,384u*1024*1024);
    options.host_bytes=bytes-1;
    EXPECT_THROW(FrameGeometryBudget(359785,349645,677989,options),std::exception);
    ++options.host_bytes;
    EXPECT_EQ(FrameGeometryBudget(359785,349645,677989,options),bytes);
    RecordProperty("activity_geometry_budget_bytes",std::to_string(bytes));
    const auto context=Context();
    const auto frame=Frame();
    const auto active=Activity(context,frame.stamp,{1,0,1});
    FullShellFrameGeometry uninitialized;
    EXPECT_EQ(uninitialized.Update(frame,active,frame.stamp).status,ReplaySceneStatus::NotInitialized);
    detail::FrameGeometryState legacy_state(context),enabled(context);
    Initialize(legacy_state,ReplayColorMode::PartId,false);
    Initialize(enabled,ReplayColorMode::PartId);
    EXPECT_EQ(detail::UpdateFrame(legacy_state,frame,&active,frame.stamp).status,ReplaySceneStatus::InvalidFrame);
    EXPECT_EQ(detail::UpdateFrame(enabled,frame,nullptr,frame.stamp).status,ReplaySceneStatus::InvalidFrame);
    EXPECT_FALSE(enabled.visible);
    ASSERT_EQ(detail::UpdateFrame(legacy_state,frame,nullptr,frame.stamp).status,ReplaySceneStatus::Ok);
    EXPECT_EQ(legacy_state.mesh->GetIndicesVertices().size(),5u);
    EXPECT_FALSE(legacy_state.activity);
}
} // namespace crash::visual::full_shell::test
