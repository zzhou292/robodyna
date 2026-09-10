#include "SourcePartContactGeometry.h"
#include "case/PlacedCanonicalWall.h"
#include "case/CanonicalWallArtifacts.h"
#include "qualification/source_contact/SourceNodalWallFixture.h"
#include "qualification/source_contact/SourceShellCollection.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cstdlib>
#include <limits>
#include <set>
#include <sstream>

namespace crash::cases::source_part_wall {
namespace {
namespace legacy=qualification::source_contact::nodal;
namespace fs=std::filesystem;
std::string SourcePath,WallPath;
source::SourcePartContactFixture Source() {
    source::SourcePartContactFixture input;
    const auto loaded=source::LoadPinnedSourcePartContact(SourcePath,&input);
    output::Require(loaded.status==source::FixtureStatus::Ok,loaded.diagnostic.c_str());return input;
}
struct WallInput {
    case_data::CanonicalWall wall;
    std::string bytes;
    WallInput():bytes(case_data::ReadPinnedWallManifest(WallPath)) {
        std::istringstream stream(bytes);
        output::Require(wall.Load(stream).status==case_data::WallStatus::Ok,"Pinned wall failed its owning loader");
    }
};
void SameCertificate(const contact::Q4CertifiedIntegral& a,const contact::Q4CertifiedIntegral& b) {
    EXPECT_EQ(output::Bits(a.value),output::Bits(b.value));EXPECT_EQ(output::Bits(a.error),output::Bits(b.error));
    EXPECT_EQ(output::Bits(a.lower),output::Bits(b.lower));EXPECT_EQ(output::Bits(a.upper),output::Bits(b.upper));
}
struct Directory {
    fs::path path;
    Directory() {
        auto pattern=(fs::temp_directory_path()/"placed-source-wall-XXXXXX").string();
        std::vector<char> text(pattern.begin(),pattern.end());text.push_back(0);
        const auto p=::mkdtemp(text.data());output::Require(p,"Cannot create placed-wall test directory");path=p;
    }
    ~Directory(){std::error_code error;fs::remove_all(path,error);}
};
output::Document Read(const fs::path& path) {
    const auto bytes=output::ReadBounded(path,1024*1024);output::Document d;
    d.Parse<rapidjson::kParseFullPrecisionFlag>(bytes.data(),bytes.size());
    output::Require(!d.HasParseError()&&d.IsObject(),"Invalid placed-wall test artifact");return d;
}
TEST(SourcePartWallGeometry, AllOriginalParentsAndNodesRetainAreasWithoutChoosingStructuralMass) {
    const auto input=Source();SourcePartContactGeometry geometry;legacy::PreparedSource old;std::string diagnostic;
    ASSERT_TRUE(geometry.Initialize(input,diagnostic))<<diagnostic;
    ASSERT_TRUE(old.Initialize(input,diagnostic))<<diagnostic;
    const auto& weights=*geometry.weights();ASSERT_EQ(weights.node_count(),117u);ASSERT_EQ(weights.parent_count(),94u);
    SameCertificate(weights.total_area(),old.weights().total_area());
    std::set<std::uint64_t> elements;unsigned quads=0,triangles=0;
    for(unsigned p=0;p<source::ParentCount;++p) {
        const auto& a=weights.parent(p);const auto& b=old.weights().parent(p);
        EXPECT_EQ(a.parent_element_id,b.parent_element_id);EXPECT_EQ(a.feature_id,b.feature_id);
        EXPECT_EQ(a.parent_face_id,b.parent_face_id);EXPECT_EQ(a.arity,b.arity);EXPECT_EQ(a.family,b.family);
        SameCertificate(a.area,b.area);SameCertificate(a.share,b.share);elements.insert(a.parent_element_id);
        for(unsigned n=0;n<a.arity;++n)EXPECT_EQ(a.nodes[n],b.nodes[n]);
        contact::NodalWallWeights single;
        ASSERT_TRUE(geometry.ParentWeights(p,&single));ASSERT_EQ(single.parent_count(),1u);
        const auto& selected=single.parent(0);const auto& original=input.parents()[p];
        ASSERT_LT(geometry.weight_index(p),94u);
        EXPECT_EQ(weights.parent(geometry.weight_index(p)).parent_element_id,original.source_id);
        EXPECT_EQ(geometry.source_parent_index(geometry.weight_index(p)),p);
        EXPECT_EQ(selected.parent_element_id,original.source_id);EXPECT_EQ(selected.arity,original.arity);
        for(unsigned n=0;n<original.arity;++n)EXPECT_EQ(selected.nodes[n],original.local_node_indices[n]);
        quads+=original.arity==4;triangles+=original.arity==3;
    }
    EXPECT_EQ(elements.size(),94u);EXPECT_EQ(quads,88u);EXPECT_EQ(triangles,6u);
    for(unsigned n=0;n<source::NodeCount;++n) {EXPECT_EQ(weights.node(n).node,n);SameCertificate(weights.node(n).area,old.weights().node(n).area);}
    source::SourceShellCollection collection;ASSERT_EQ(collection.Initialize(input).status,source::FixtureStatus::Ok);
    tl::fea::ShellBatchBinding structural;ASSERT_EQ(structural.Initialize(collection.input()).status,tl::fea::ShellBindingStatus::Success);
    // The old equal-native-node proxy and actual native structural startup are
    // deliberately different. Contact geometry cannot silently select either.
    bool different=false;
    for(unsigned n=0;n<source::NodeCount;++n)
        different|=output::Bits(structural.nodes()[n].native.mass)!=output::Bits(old.mass().mass[n]);
    EXPECT_TRUE(different);EXPECT_GT(structural.totals().isotropic_inertia,0);
}
TEST(SourcePartWallGeometry, FailedPreparationAndParentSelectionPreservePublishedGeometry) {
    SourcePartContactGeometry geometry;std::string diagnostic;source::SourcePartContactFixture empty;
    EXPECT_FALSE(geometry.Initialize(empty,diagnostic));EXPECT_FALSE(geometry.prepared());EXPECT_EQ(geometry.weights(),nullptr);
    EXPECT_EQ(geometry.weight_index(0),UINT32_MAX);EXPECT_EQ(geometry.source_parent_index(0),UINT32_MAX);
    const auto input=Source();ASSERT_TRUE(geometry.Initialize(input,diagnostic));
    const auto* weights=geometry.weights();const auto total=weights->total_area();
    EXPECT_FALSE(geometry.Initialize(input,diagnostic));EXPECT_EQ(geometry.weights(),weights);SameCertificate(weights->total_area(),total);
    contact::NodalWallWeights selected;ASSERT_TRUE(geometry.ParentWeights(0,&selected));const auto area=selected.total_area();
    EXPECT_FALSE(geometry.ParentWeights(source::ParentCount,&selected));EXPECT_FALSE(geometry.ParentWeights(0,nullptr));
    SameCertificate(selected.total_area(),area);EXPECT_EQ(selected.parent_count(),1u);
    EXPECT_EQ(geometry.weight_index(94),UINT32_MAX);EXPECT_EQ(geometry.source_parent_index(94),UINT32_MAX);
}
TEST(SourcePartWallGeometry, ExplicitPlacementPreservesAllOriginalIdentityAndArchivesActualMesh) {
    WallInput source_wall;case_data::PlacedCanonicalWall placed;
    constexpr double shift=-.34817618-.05;
    const auto prepared=placed.Initialize(source_wall.wall,source_wall.bytes,shift);
    ASSERT_EQ(prepared.status,case_data::PlacedWallStatus::Ok)<<prepared.diagnostic;
    const auto view=placed.view();ASSERT_EQ(view.vertex_count,62u);ASSERT_EQ(view.triangle_count,100u);
    const double expected=.05+shift;EXPECT_EQ(output::Bits(placed.placement()->translation_x_m),output::Bits(shift));
    EXPECT_EQ(output::Bits(placed.geometry()->wall_x()),output::Bits(expected));
    EXPECT_EQ(placed.placement()->source_manifest_sha256,case_data::kCanonicalWallManifestSha256);
    for(unsigned n=0;n<62;++n) {
        const auto& a=view.vertices[n];const auto& b=source_wall.wall.vertices()[n];
        EXPECT_EQ(output::Bits(a.position.x),output::Bits(expected));
        EXPECT_EQ(output::Bits(a.position.y),output::Bits(b.position_m[1]));EXPECT_EQ(output::Bits(a.position.z),output::Bits(b.position_m[2]));
        EXPECT_EQ(a.source_node_id,b.source_node_id);EXPECT_EQ(a.assembled_source_node_id,b.assembled_source_node_id);
        EXPECT_EQ(output::Bits(b.position_m[0]),output::Bits(.05));
    }
    for(unsigned p=0;p<100;++p) {
        const auto& a=view.triangles[p];const auto& b=source_wall.wall.triangles()[p];
        EXPECT_EQ(a.triangle_id,b.triangle_id);EXPECT_EQ(a.source_quad_id,b.source_quad_id);EXPECT_EQ(a.assembled_source_quad_id,b.assembled_source_quad_id);
        for(unsigned n=0;n<3;++n)EXPECT_EQ(a.nodes[n],b.vertex_indices[n]);
    }
    Directory directory;ASSERT_NO_THROW(case_data::WritePlacedCanonicalWallArtifacts(directory.path,placed));
    const auto metadata=Read(directory.path/"placed-wall-placement.json");const auto mesh=Read(directory.path/"placed-wall.mesh.json");
    EXPECT_EQ(metadata["declared_translation_x_binary64"].GetUint64(),output::Bits(shift));
    EXPECT_EQ(metadata["represented_wall_x_binary64"].GetUint64(),output::Bits(expected));
    EXPECT_EQ(metadata["vertex_identity"].Size(),62u);EXPECT_EQ(metadata["triangle_identity"].Size(),100u);
    EXPECT_EQ(metadata["placed_mesh_sha256"].GetString(),output::Sha256(output::ReadBounded(directory.path/"placed-wall.mesh.json",1024*1024)));
    for(const auto& vertex:mesh["mesh"]["m_vertices"].GetArray())EXPECT_EQ(output::Bits(vertex["x"].GetDouble()),output::Bits(expected));
    EXPECT_EQ(output::ReadBounded(directory.path/"original-canonical-wall.manifest.json",1024*1024),source_wall.bytes);
    EXPECT_THROW(case_data::WritePlacedCanonicalWallArtifacts(directory.path,placed),std::runtime_error);
}
TEST(SourcePartWallGeometry, CompleteMotionBoxProjectsBothXEndpointsAndChecksActualFiniteBoundary) {
    const auto input=Source();SourcePartContactGeometry geometry;std::string diagnostic;ASSERT_TRUE(geometry.Initialize(input,diagnostic));
    WallInput source_wall;case_data::PlacedCanonicalWall placed;
    ASSERT_EQ(placed.Initialize(source_wall.wall,source_wall.bytes,-.34817618-.05).status,case_data::PlacedWallStatus::Ok);
    const auto bounds=geometry.reference_bounds();auto lo=bounds[0],hi=bounds[1];
    lo.x-=.05;hi.x+=.05;lo.y-=.01;hi.y+=.01;lo.z-=.01;hi.z+=.01;
    contact::PlanarWallBoxCoverage coverage;
    const auto checked=geometry.CheckWallCoverage(*placed.geometry(),lo,hi,1e-6,source::PartId,&coverage);
    ASSERT_EQ(checked.status,contact::PlanarContactStatus::Ok)<<checked.message;ASSERT_TRUE(coverage.covered);
    const auto x=placed.geometry()->wall_x();EXPECT_EQ(output::Bits(coverage.physical.minimum.x),output::Bits(x));
    EXPECT_EQ(output::Bits(coverage.physical.maximum.x),output::Bits(x));
    EXPECT_DOUBLE_EQ(coverage.physical.minimum.y,lo.y);EXPECT_DOUBLE_EQ(coverage.physical.maximum.z,hi.z);
    const auto before=coverage;auto outside=hi;outside.y=2;
    EXPECT_NE(geometry.CheckWallCoverage(*placed.geometry(),lo,outside,1e-6,source::PartId,&coverage).status,contact::PlanarContactStatus::Ok);
    EXPECT_EQ(output::Bits(coverage.physical.maximum.y),output::Bits(before.physical.maximum.y));EXPECT_TRUE(coverage.covered);
    auto incomplete=lo;incomplete.z=bounds[0].z+.001;
    EXPECT_EQ(geometry.CheckWallCoverage(*placed.geometry(),incomplete,hi,1e-6,source::PartId,&coverage).status,contact::PlanarContactStatus::InvalidInput);
    EXPECT_EQ(output::Bits(coverage.physical.minimum.z),output::Bits(before.physical.minimum.z));
}
TEST(SourcePartWallGeometry, InvalidPlacementIsAtomicAndCanonicalLoaderStillRejectsMovedCoordinates) {
    WallInput input;case_data::PlacedCanonicalWall placed;
    for(double invalid:{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
        EXPECT_EQ(placed.Initialize(input.wall,input.bytes,invalid).status,case_data::PlacedWallStatus::InvalidInput);
        EXPECT_FALSE(placed.initialized());EXPECT_EQ(placed.view().vertices,nullptr);
    }
    EXPECT_EQ(placed.Initialize(input.wall,input.bytes+" ",.1).status,case_data::PlacedWallStatus::InvalidInput);
    ASSERT_EQ(placed.Initialize(input.wall,input.bytes,.1).status,case_data::PlacedWallStatus::Ok);
    const auto view=placed.view();const auto* metadata=placed.placement();
    EXPECT_EQ(placed.Initialize(input.wall,input.bytes,.2).status,case_data::PlacedWallStatus::AlreadyInitialized);
    EXPECT_EQ(placed.view().vertices,view.vertices);EXPECT_EQ(placed.placement(),metadata);
    EXPECT_EQ(output::Bits(placed.view().vertices[0].position.x),output::Bits(.05+.1));
    output::Document changed;changed.Parse(input.bytes.data(),input.bytes.size());
    for(auto& vertex:changed["vertices"].GetArray())vertex["position_m"][0].SetDouble(.15);
    Directory directory;output::WriteJson(directory.path/"moved-original-schema.json",changed);
    case_data::CanonicalWall still_original;
    EXPECT_EQ(still_original.LoadFile((directory.path/"moved-original-schema.json").string()).status,case_data::WallStatus::InvalidData);
    EXPECT_FALSE(still_original.loaded());
}
} // namespace
} // namespace crash::cases::source_part_wall
int main(int argc,char** argv) {
    if(argc<3)return 2;
    crash::cases::source_part_wall::SourcePath=argv[1];crash::cases::source_part_wall::WallPath=argv[2];
    ::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
