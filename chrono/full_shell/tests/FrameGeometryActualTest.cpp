#include "FrameGeometryActualSupport.h"
#include "ColorChecks.h"
#include "chrono/ReplayScalarApplicability.h"
#include <limits>

namespace crash::visual::full_shell::test {
TEST(FullShellGeometryActual,OriginalFullSourceBinaryFramesKeepExactCoordinatesSourceIdsAndBothColors) {
    const auto& f=Actual();
    const auto initial=f.Frame(), next=f.Frame(true);
    ft::Directory directory;
    const auto first=fsout::WriteFrame(directory.path,"initial",f.context,ft::View(initial));
    const auto later=fsout::WriteFrame(directory.path,"later",f.context,ft::View(next));
    const auto read0=fsout::ReadFrame(directory.path,f.context,first,initial.stamp);
    const auto read1=fsout::ReadFrame(directory.path,f.context,later,next.stamp);
    for(auto mode:{ReplayColorMode::PartId,ReplayColorMode::PlasticStrain}) {
        FullShellFrameGeometry geometry;
        ASSERT_EQ(geometry.Initialize(f.mapping,f.context,f.Options(mode)).status,ReplaySceneStatus::Ok);
        EXPECT_EQ(geometry.mesh(),nullptr);EXPECT_EQ(geometry.fields(),nullptr);
        ASSERT_EQ(geometry.Update(read0,read0.stamp).status,ReplaySceneStatus::Ok);
        const auto mesh=geometry.mesh();
        ASSERT_EQ(mesh->GetCoordsVertices().size(),359785u);ASSERT_EQ(mesh->GetIndicesVertices().size(),677989u);
        ExactPositions(mesh->GetCoordsVertices(),initial.position_xyz);
        ASSERT_EQ(geometry.Update(read1,read1.stamp).status,ReplaySceneStatus::Ok);
        EXPECT_EQ(geometry.mesh().get(),mesh.get());ExactPositions(mesh->GetCoordsVertices(),next.position_xyz);
        const auto& fields=*geometry.fields();
        ASSERT_EQ(fields.size(),349645u);EXPECT_EQ(fields.front().value,read1.plastic_points[2]);EXPECT_EQ(fields.back().value,read1.plastic_points[5]);
        EXPECT_EQ(fields[1].applicability,output::ReplayScalarApplicability::NotApplicable);
        EXPECT_EQ(fields[2].applicability,output::ReplayScalarApplicability::Unavailable);
        EXPECT_EQ(geometry.scalar_legend()->native,2u);EXPECT_EQ(geometry.scalar_legend()->not_applicable,1u);
        EXPECT_EQ(geometry.scalar_legend()->unavailable,349642u);
        const auto& indices=f.mapping.arrays()[source::detail::TriangleParents];
        const auto parents=output::arrays::Decode<std::uint32_t>(indices.descriptor,indices.bytes);
        for(std::size_t t=0;t<parents.size();++t) {
            const auto p=parents[t];const auto& source_parent=f.mapping.parents()[p];
            ASSERT_EQ((*geometry.triangle_source_parents())[t],source_parent.source_element);
            ASSERT_EQ((*geometry.triangle_source_parts())[t],source_parent.source_part);
            const auto expected=mode==ReplayColorMode::PartId?ReplayPartColor(source_parent.source_part)
                : fields[p].applicability==output::ReplayScalarApplicability::NativeValue
                    ?ReplayScalarColor(fields[p].value/.001):ReplayMissingScalarColor(fields[p].applicability);
            ASSERT_TRUE(SameColor(mesh->GetCoordsColors()[t],expected));
        }
        if(mode==ReplayColorMode::PartId)EXPECT_EQ(geometry.part_legend()->size(),867u);
    }
}
TEST(FullShellGeometryActual,WrongContextPhaseAndLateTailPreservePresentationAndAllowRetry) {
    const auto& f=Actual();
    auto parents=f.context.parents();parents.back().source_element ^= UINT64_C(1)<<63;
    const auto wrong=fsout::Context::Create(f.context.identity(),f.context.nodes(),parents.data(),parents.size(),f.context.fixed_dt());
    FullShellFrameGeometry geometry;
    EXPECT_EQ(geometry.Initialize(f.mapping,wrong,f.Options(ReplayColorMode::PartId)).status,ReplaySceneStatus::InvalidFrame);
    ASSERT_EQ(geometry.Initialize(f.mapping,f.context,f.Options(ReplayColorMode::PartId)).status,ReplaySceneStatus::Ok);
    const auto initial=f.Frame();auto next=f.Frame(true);
    ASSERT_EQ(geometry.Update(initial,initial.stamp).status,ReplaySceneStatus::Ok);
    const auto mesh=geometry.mesh();const auto colors=mesh->GetCoordsColors();
    auto phase=next.stamp;++phase.attempt;
    EXPECT_EQ(geometry.Update(next,phase).status,ReplaySceneStatus::InvalidFrame);
    const auto value=next.plastic_points.back();next.plastic_points.back()=std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(geometry.Update(next,next.stamp).status,ReplaySceneStatus::InvalidFrame);
    next.plastic_points.back()=value;
    const auto position=next.position_xyz.back();next.position_xyz.back()=std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(geometry.Update(next,next.stamp).status,ReplaySceneStatus::InvalidFrame);
    EXPECT_TRUE(fsout::SameStamp(*geometry.stamp(),initial.stamp));ExactPositions(mesh->GetCoordsVertices(),initial.position_xyz);
    ExpectColors(mesh->GetCoordsColors(),colors);EXPECT_EQ(geometry.fields()->back().value,0);
    next.position_xyz.back()=position;
    ASSERT_EQ(geometry.Update(next,next.stamp).status,ReplaySceneStatus::Ok);
    EXPECT_EQ(geometry.mesh().get(),mesh.get());ExpectColors(mesh->GetCoordsColors(),colors);
    ExactPositions(mesh->GetCoordsVertices(),next.position_xyz);
    EXPECT_TRUE(fsout::SameStamp(*geometry.stamp(),next.stamp));
}
TEST(FullShellGeometryActual,AllUnavailableSourceFieldsKeepPartColorsWithoutAStrainScale) {
    const auto& mapping=st::ActualMapping();
    const auto context=mapping.MakeFrameContext(fsout::Identity{1,2,3,4,5,6},0x1p-26);
    auto frame=Actual().Frame();frame.plastic_points.clear();
    FrameGeometryOptions options;options.geometry=ReplayGeometryLimits::Vehicle();
    options.colors=ReplayColorMode::PlasticStrain;
    FullShellFrameGeometry geometry;
    EXPECT_EQ(geometry.Initialize(mapping,context,options).status,ReplaySceneStatus::InvalidFrame);
    options.colors=ReplayColorMode::Automatic;
    ASSERT_EQ(geometry.Initialize(mapping,context,options).status,ReplaySceneStatus::Ok);
    EXPECT_EQ(geometry.color_mode(),ReplayColorMode::PartId);
    ASSERT_EQ(geometry.Update(frame,frame.stamp).status,ReplaySceneStatus::Ok);
    EXPECT_EQ(geometry.scalar_legend()->native,0u);EXPECT_EQ(geometry.scalar_legend()->maximum,0);
    EXPECT_EQ(geometry.scalar_legend()->unavailable,349645u);
    for(const auto& field:*geometry.fields())ASSERT_EQ(field.applicability,output::ReplayScalarApplicability::Unavailable);
    EXPECT_EQ(geometry.part_legend()->size(),867u);
}
} // namespace crash::visual::full_shell::test
