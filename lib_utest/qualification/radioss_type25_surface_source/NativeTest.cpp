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
TEST(Type25SurfaceSourceNative, ExplicitRawBrickSlotsRetainNativeCompactionAndClauseBehavior) {
  // Native BRICK packet with two collapsed top edges, unlike declared PENTA's
  // slots4=1/8=5. Reordering it into a different family would change face IDs.
  const s::Solid source{601,10,s::SolidTopology::NativeRaw8,{0,1,2,3,4,4,5,5}};
  for(unsigned rotate=0;rotate<8;++rotate) {
    SCOPED_TRACE(rotate);
    auto c=SingleHex();
    c.solids={source};
    for(unsigned k=0;k<8;++k)c.solids[0].nodes[k]=source.nodes[(k+rotate)%8];
    for(const auto mode:{s::SurfaceMode::Exterior,s::SurfaceMode::ExteriorShellEdges,s::SurfaceMode::All}) {
      c.mode=mode;Compare(c);
      c.kind=s::ClauseKind::Solids;c.parts.clear();c.selected={0};Compare(c);
      c.kind=s::ClauseKind::Parts;c.parts={10};c.selected.clear();
    }
  }
  auto c=SingleHex();c.solids={source};
  c.quads={{701,20,{0,1,2,3}},{702,10,{0,1,2,3}}};
  c.triangles={{703,20,{0,1,4,4}},{704,10,{0,1,4,4}}};
  Compare(c);
  std::reverse(c.quads.begin(),c.quads.end());
  std::reverse(c.triangles.begin(),c.triangles.end());
  Compare(c);
  c.quads.clear();c.triangles.clear();
  Built compact(c);ASSERT_EQ(compact.report.status,s::Status::Ok);
  EXPECT_EQ(compact.result.face_count,5u);
  EXPECT_EQ(compact.result.counts.degenerate_faces,1u);
  std::size_t triangles=0;
  for(std::size_t i=0;i<compact.result.face_count;++i)
    triangles+=compact.result.faces[i].nodes[2]==compact.result.faces[i].nodes[3];
  EXPECT_EQ(triangles,2u);
}
TEST(Type25SurfaceSourceNative, RawBrickProfileIsExplicitAndPreservesStrictLegacyRejection) {
  auto c=SingleHex();
  c.solids[0].nodes[5]=c.solids[0].nodes[4];
  c.solids[0].nodes[7]=c.solids[0].nodes[6];
  Built strict(c);EXPECT_EQ(strict.report.status,s::Status::UnsupportedProfile);
  EXPECT_THROW(Oracle(c.Input()),std::invalid_argument);
  c.solids[0].topology=s::SolidTopology::DeclaredPenta6;
  Built wrong_family(c);EXPECT_EQ(wrong_family.report.status,s::Status::InvalidInput);
  c.solids[0].topology=s::SolidTopology::NativeRaw8;
  Compare(c);
  Built admitted(c);ASSERT_EQ(admitted.report.status,s::Status::Ok);
  const auto prior=admitted.result;
  const auto bytes=Bytes(admitted.output);
  c.solids[0].nodes[7]=static_cast<std::uint32_t>(c.nodes);
  EXPECT_EQ(s::Build(c.Input(),{},admitted.output,admitted.scratch,&admitted.result).status,s::Status::InvalidInput);
  EXPECT_EQ(Bytes(admitted.output),bytes);SameSnapshot(admitted.result,prior);
  // No repeat removal or node rewriting occurs even when a whole raw face
  // degenerates below three distinct nodes; native handles that face locally.
}

}
