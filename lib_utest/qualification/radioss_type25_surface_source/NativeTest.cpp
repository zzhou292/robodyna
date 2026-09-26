#include "Fixture.h"
#include "NativeOracle.h"
namespace type25_surface_source_test {
namespace {
void Compare(const Case& c) {
  const auto reference=Oracle(c.Input());Built actual(c);
  ASSERT_EQ(actual.report.status,s::Status::Ok);
  ASSERT_EQ(actual.result.face_count,reference.faces.size());
  ASSERT_EQ(actual.result.solid_count,reference.surface_solid_flags.size());
  for(std::size_t i=0;i<reference.faces.size();++i) {
    SCOPED_TRACE(i);SameFace(actual.result.faces[i],reference.faces[i]);
  }
  for(std::size_t i=0;i<reference.surface_solid_flags.size();++i)
    EXPECT_EQ(actual.result.surface_solid_flags[i],reference.surface_solid_flags[i]);
}
}
TEST(Type25SurfaceSourceNative, CompleteHexPentaAndMixedShellOutputsMatchTheOriginalRoutines) {
  for(const auto mode:{s::SurfaceMode::Exterior,s::SurfaceMode::ExteriorShellEdges,s::SurfaceMode::All}) {
    auto c=SingleHex();c.mode=mode;Compare(c);c.solids={Penta()};Compare(c);
    c.solids={Hex(),Penta(102,10,8)};
    c.quads={{201,10,{14,15,16,17}}};c.triangles={{202,10,{18,19,20,20}}};Compare(c);
    c.reverse=true;Compare(c);
    c.parts.clear();Compare(c);
  }
}
TEST(Type25SurfaceSourceNative, SharedFacesSelectedScopeAndOverlappingCellsFollowNativeCriteria) {
  auto c=Adjacent();Compare(c);c.parts={10};Compare(c);
  c.kind=s::ClauseKind::Solids;c.parts.clear();c.selected={0};Compare(c);
  c.selected={0,1};Compare(c);c.mode=s::SurfaceMode::All;Compare(c);
  c=SingleHex();c.solids.push_back(Hex(102));Compare(c);
  c.mode=s::SurfaceMode::All;Compare(c);
  // Raw node numbering changes the native rotated-edge comparison. Preserve
  // the complete source criterion, not a graphics same-face expectation.
  std::swap(c.solids[1].nodes[1],c.solids[1].nodes[3]);c.mode=s::SurfaceMode::Exterior;Compare(c);
}
TEST(Type25SurfaceSourceNative, FirstShellAndCornerOrderAreObservedForQuadAndTriangleSuppression) {
  for(const bool tri:{false,true}) {
    auto c=SingleHex();c.parts={10,30};
    if(tri) {c.solids={Penta()};c.triangles={{501,20,{0,2,1,1}},{502,30,{0,2,1,1}}};}
    else c.quads={{501,20,{4,5,6,7}},{502,30,{4,5,6,7}}};
    Compare(c);auto& rows=tri?c.triangles:c.quads;std::swap(rows[0],rows[1]);Compare(c);
    rows[0].part_id=20;rows[1].part_id=30;
    if(tri)rows[0].nodes[0]=2,rows[0].nodes[1]=1,rows[0].nodes[2]=0,rows[0].nodes[3]=0;
    else rows[0].nodes[0]=5,rows[0].nodes[1]=6,rows[0].nodes[2]=7,rows[0].nodes[3]=4;
    Compare(c);
  }
}
TEST(Type25SurfaceSourceNative, GeneratedCorpusPreservesEveryFaceAndStableFiveWordOrder) {
  for(unsigned seed=0;seed<32;++seed) {
    SCOPED_TRACE(seed);auto c=Adjacent();c.nodes=32;
    c.solids.push_back(Penta(301,30,16));
    c.quads={{501,20,{4,5,6,7}},{502,40,{4,5,6,7}}};
    c.triangles={{503,30,{16,18,17,17}}};
    c.parts={10};if(seed&1)c.parts.push_back(20);if(seed&2)c.parts.push_back(30);if(seed&4)c.parts.push_back(40);
    if(seed&8)std::swap(c.quads[0],c.quads[1]);
    c.mode=seed&16?s::SurfaceMode::All:s::SurfaceMode::Exterior;c.reverse=bool(seed&2);
    Compare(c);
  }
}
}
