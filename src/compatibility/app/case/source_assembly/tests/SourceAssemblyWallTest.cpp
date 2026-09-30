#include "SourceAssemblyBindingTestSupport.h"
#include "case/shell_collection/ShellCollectionContactGeometry.h"
#include "case/CanonicalWallArtifacts.h"
#include "case/PlacedCanonicalWall.h"
#include <cstdlib>
#include <sstream>

namespace crash::cases::source_assembly::test {
namespace sc=tlfea::contact;
TEST(SourceAssemblyWall, CompleteOriginalMeshCoversTheAssemblyAtItsOwnLeadingGap) {
    const auto assembly=SourceAssemblyBindings::Prepare(Load(),Options());
    ShellCollectionContactGeometry geometry;ASSERT_TRUE(geometry.Initialize(assembly.shells()));
    const auto* path=std::getenv("ROBO_DYNA_SOURCE_ASSEMBLY_WALL");ASSERT_NE(path,nullptr);
    const auto bytes=case_data::ReadPinnedWallManifest(path);case_data::CanonicalWall source_wall;
    std::istringstream input(bytes);ASSERT_EQ(source_wall.Load(input).status,case_data::WallStatus::Ok);
    const auto bounds=geometry.reference_bounds();constexpr double gap=.02;
    case_data::PlacedCanonicalWall placed;const double shift=(bounds[1].x+gap)-.05;
    ASSERT_EQ(placed.Initialize(source_wall,bytes,shift).status,case_data::PlacedWallStatus::Ok);
    const auto wall=placed.view();ASSERT_EQ(wall.vertex_count,62u);ASSERT_EQ(wall.triangle_count,100u);
    const double wall_x=placed.geometry()->wall_x();sc::Q4IntegralInterval represented_gap;
    ASSERT_TRUE(sc::q4_bounds::Difference(wall_x,bounds[1].x,&represented_gap));
    EXPECT_GT(represented_gap.lower,0);EXPECT_NEAR(wall_x-bounds[1].x,gap,1e-15);
    EXPECT_NE(wall_x,-.34867618+gap); // The earlier connector is not the leading part.
    for(unsigned i=0;i<wall.vertex_count;++i) {
        const auto& current=wall.vertices[i];const auto& original=source_wall.vertices()[i];
        SameBits(current.position.x,wall_x);SameBits(current.position.y,original.position_m[1]);
        SameBits(current.position.z,original.position_m[2]);EXPECT_EQ(current.source_node_id,original.source_node_id);
    }
    for(unsigned p=0;p<wall.triangle_count;++p) {
        const auto& current=wall.triangles[p];const auto& original=source_wall.triangles()[p];
        EXPECT_EQ(current.triangle_id,original.triangle_id);EXPECT_EQ(current.source_quad_id,original.source_quad_id);
        for(unsigned l=0;l<3;++l)EXPECT_EQ(current.nodes[l],original.vertex_indices[l]);
    }
    sc::PlanarWallBox motion{bounds[0],bounds[1]};
    motion.minimum.x-=.05;motion.maximum.x+=.05;
    motion.minimum.y-=.01;motion.minimum.z-=.01;motion.maximum.y+=.01;motion.maximum.z+=.01;
    sc::PlanarWallBoxCoverage coverage;
    const auto result=geometry.CheckWallCoverage(*placed.geometry(),motion,1e-6,Options().source_instance_id,&coverage);
    ASSERT_EQ(result.status,sc::PlanarContactStatus::Ok)<<result.message;EXPECT_TRUE(coverage.covered);
    SameBits(coverage.physical.minimum.x,wall_x);SameBits(coverage.physical.maximum.x,wall_x);
    const auto before=coverage;auto outside=motion;outside.maximum.y=2;
    EXPECT_NE(geometry.CheckWallCoverage(*placed.geometry(),outside,1e-6,Options().source_instance_id,&coverage).status,
        sc::PlanarContactStatus::Ok);
    SameBits(coverage.physical.maximum.y,before.physical.maximum.y);
    auto incomplete=motion;incomplete.minimum.z=bounds[0].z+.001;
    EXPECT_EQ(geometry.CheckWallCoverage(*placed.geometry(),incomplete,1e-6,Options().source_instance_id,&coverage).status,
        sc::PlanarContactStatus::InvalidInput);
    SameBits(coverage.physical.minimum.z,before.physical.minimum.z);
    RecordProperty("represented_wall_x_m",Number(wall_x));RecordProperty("declared_leading_gap_m",Number(gap));
    RecordProperty("source_manifest_sha256",placed.placement()->source_manifest_sha256);
}
} // namespace crash::cases::source_assembly::test
