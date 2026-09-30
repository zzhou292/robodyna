#include "SourceAssemblyOutputTestSupport.h"
#include "output/MeshArchive.h"
#include "chrono/ReplayParentScalarColors.h"
#include <limits>

namespace crash::output::assembly::test {
TEST(SourceAssemblySurface, EveryOriginalNodeParentFamilyAndDisplayFaceSurvives) {
    const auto surface=Surface();const auto& data=surface.source().data();const auto& b=surface.binding();
    ASSERT_EQ(b.vertices.size(),1030u);ASSERT_EQ(b.triangles.size(),1719u);ASSERT_EQ(surface.parents().size(),915u);
    ASSERT_EQ(surface.triangle_parents().size(),1719u);
    for(std::size_t n=0;n<data.nodes.size();++n) {
        EXPECT_EQ(b.vertices[n].tl_node,n);EXPECT_EQ(b.vertices[n].source.node,data.nodes[n].source_id);
        EXPECT_EQ(b.vertices[n].source.asset,5u);EXPECT_EQ(b.vertices[n].source.instance,0x5941524953u);
    }
    std::size_t triangles=0;
    for(std::size_t i=0;i<data.parents.size();++i) {
        SCOPED_TRACE(i);const auto& input=data.parents[i];const auto& p=surface.parents()[i];
        EXPECT_EQ(p.source_index,i);EXPECT_EQ(p.element,input.source_id);EXPECT_EQ(p.part,input.part_id);
        EXPECT_EQ(p.material,input.material_id);EXPECT_EQ(p.section,input.section_id);EXPECT_EQ(p.curve,input.curve_id);
        EXPECT_EQ(p.source_elform,input.source_elform);EXPECT_EQ(p.family,input.family);EXPECT_EQ(p.family_index,input.family_index);
        EXPECT_EQ(p.first_triangle,triangles);EXPECT_EQ(p.triangle_count,input.arity==4?2u:1u);
        for(unsigned piece=0;piece<p.triangle_count;++piece) {
            const auto& face=b.triangles[triangles];EXPECT_EQ(face.element,p.element);EXPECT_EQ(face.part,p.part);
            EXPECT_EQ(face.subtriangle,piece);EXPECT_EQ(face.local_face,0u);EXPECT_EQ(surface.triangle_parents()[triangles],p.element);
            EXPECT_EQ(face.vertices[0],input.nodes[0]);EXPECT_EQ(face.vertices[1],input.nodes[piece?2:1]);
            EXPECT_EQ(face.vertices[2],input.nodes[piece?3:2]);++triangles;
        }
    }
    EXPECT_EQ(triangles,1719u);
}
TEST(SourceAssemblySurface, PhysicalScaleChronoArchiveAndLateGeometryFailureAreAtomic) {
    const auto surface=Surface();auto x=Positions(surface);visual::AcceptedSurfaceMesh mesh;
    ASSERT_EQ(mesh.Initialize(surface.binding()).status,visual::Status::Ok);
    ASSERT_EQ(mesh.Publish({x.data(),nullptr,nullptr,1030},{surface.binding().identity,0,0}).status,visual::Status::Ok);
    const auto saved=mesh.mesh()->GetCoordsVertices();
    for(std::size_t n=0;n<1030;++n)for(unsigned c=0;c<3;++c)EXPECT_EQ(Bits(saved[n][c]),Bits(x[3*n+c]));
    source::test::Scratch scratch;const auto directory=scratch.Write("host geometry mapping test; not a simulation frame").parent_path();
    ASSERT_NO_THROW(WriteMeshFiles(directory,"source-physical-scale",*mesh.mesh()));
    EXPECT_THROW(WriteMeshFiles(directory,"source-physical-scale",*mesh.mesh()),std::runtime_error);
    x.back()=std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(mesh.Publish({x.data(),nullptr,nullptr,1030},{surface.binding().identity,1,.001}).status,visual::Status::InvalidFrame);
    EXPECT_EQ(mesh.frame()->epoch,0u);EXPECT_EQ(mesh.mesh()->GetCoordsVertices(),saved);
    EXPECT_EQ(surface.source().data().identity.sha256,source::PinnedYarisSixPartInventory().sha256);
}
TEST(SourceAssemblySurface, BoundedStartupKeepsCompleteMappingOnFailedReplacement) {
    auto surface=Surface();const auto before=surface.binding().triangles.size();const auto bytes=surface.host_bytes();
    auto source=source::test::Load();SurfaceLimits limits;limits.max_host_bytes=bytes-1;
    EXPECT_THROW(surface=SourceAssemblySurface::Prepare(source,{17,23,31},5,7,limits),std::runtime_error);
    EXPECT_EQ(surface.host_bytes(),bytes);EXPECT_EQ(surface.binding().triangles.size(),before);
    limits.max_host_bytes=bytes;EXPECT_NO_THROW(SourceAssemblySurface::Prepare(source,{17,23,31},5,7,limits));
    limits.max_nodes=1029;EXPECT_THROW(SourceAssemblySurface::Prepare(source,{17,23,31},5,7,limits),std::runtime_error);
    limits={};limits.max_parents=914;EXPECT_THROW(SourceAssemblySurface::Prepare(source,{17,23,31},5,7,limits),std::runtime_error);
    limits={};limits.max_nodes=std::numeric_limits<std::size_t>::max();
    EXPECT_THROW(SourceAssemblySurface::Prepare(source,{17,23,31},5,7,limits),std::runtime_error);
    EXPECT_THROW(SourceAssemblySurface::Prepare(source,{17,23,31},0,7),std::runtime_error);
}
TEST(SourceAssemblySurface, ExistingParentColorsCoverEveryDisplayTriangleWithoutInterpolation) {
    const auto surface=Surface();Fields fields(surface);std::vector<ReplayParentScalar> values(915);
    ASSERT_TRUE(CopyParentScalars(surface,fields.Q(),fields.T(),values));
    visual::ReplayParentScalarColors color_map;std::vector<chrono::ChColor> colors;
    ASSERT_TRUE(color_map.Initialize(surface.triangle_parents(),values,1,colors));ASSERT_EQ(colors.size(),1719u);
    for(const auto& p:surface.parents()) {
        EXPECT_EQ(values[p.source_index].source_parent,p.element);
        const auto expected=visual::ReplayScalarColor(values[p.source_index].value);
        for(unsigned i=0;i<p.triangle_count;++i) {
            const auto& c=colors[p.first_triangle+i];EXPECT_EQ(c.R,expected.R);EXPECT_EQ(c.G,expected.G);EXPECT_EQ(c.B,expected.B);
        }
    }
    const auto saved=colors;values.back().value=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(color_map.Stage(values,colors));
    ASSERT_EQ(colors.size(),saved.size());for(std::size_t i=0;i<colors.size();++i) {
        EXPECT_EQ(colors[i].R,saved[i].R);EXPECT_EQ(colors[i].G,saved[i].G);EXPECT_EQ(colors[i].B,saved[i].B);
    }
}
} // namespace crash::output::assembly::test
