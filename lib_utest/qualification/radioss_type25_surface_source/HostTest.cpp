#include "Fixture.h"
namespace type25_surface_source_test {
TEST(Type25SurfaceSource, HexAndPentaKeepPhysicalParentAndDistinctRawFaceIdentity) {
  auto h=SingleHex();Built a(h);ASSERT_EQ(a.report.status,s::Status::Ok);
  ASSERT_EQ(a.result.face_count,6u);
  const unsigned order[]{3,6,5,4,1,2};
  for(unsigned i=0;i<6;++i) {
    EXPECT_EQ(a.result.faces[i].source.element_id,101u);
    EXPECT_EQ(a.result.faces[i].source.solid_face,order[i]);
    EXPECT_EQ(a.result.faces[i].raw_role,1);
  }
  auto p=h;p.solids={Penta()};Built b(p);ASSERT_EQ(b.report.status,s::Status::Ok);
  ASSERT_EQ(b.result.face_count,5u);EXPECT_EQ(b.result.counts.degenerate_faces,1u);
  std::size_t triangles=0;
  for(std::size_t i=0;i<b.result.face_count;++i) {
    const auto& face=b.result.faces[i];
    EXPECT_EQ(face.source.element_id,102u);EXPECT_NE(face.source.solid_face,6u);
    triangles+=face.nodes[2]==face.nodes[3]?1:0;
  }
  EXPECT_EQ(triangles,2u);
}
TEST(Type25SurfaceSource, ExteriorCancellationStaysInsideTheSelectedClause) {
  auto c=Adjacent();Built both(c);ASSERT_EQ(both.report.status,s::Status::Ok);
  EXPECT_EQ(both.result.face_count,10u);EXPECT_EQ(both.result.counts.internal_faces,2u);
  c.parts={10};Built one(c);ASSERT_EQ(one.report.status,s::Status::Ok);
  EXPECT_EQ(one.result.face_count,6u);EXPECT_EQ(one.result.counts.internal_faces,0u);
  c.kind=s::ClauseKind::Solids;c.parts.clear();c.selected={0};Built direct(c);
  ASSERT_EQ(direct.report.status,s::Status::Ok);EXPECT_EQ(direct.result.face_count,6u);
  c.selected={0,1};c.mode=s::SurfaceMode::All;Built all(c);
  ASSERT_EQ(all.report.status,s::Status::Ok);EXPECT_EQ(all.result.face_count,12u);
  EXPECT_EQ(all.result.counts.internal_faces,0u);
}
TEST(Type25SurfaceSource, FirstWholeModelMatchingShellControlsSuppressionEvenWhenUnselected) {
  auto c=SingleHex();c.parts={10,30};
  c.quads={{501,20,{4,5,6,7}},{502,30,{4,5,6,7}}};
  Built first_unselected(c);ASSERT_EQ(first_unselected.report.status,s::Status::Ok);
  EXPECT_EQ(first_unselected.result.face_count,7u);
  EXPECT_EQ(first_unselected.result.counts.shell_suppressed_faces,0u);
  std::swap(c.quads[0],c.quads[1]);Built first_selected(c);
  ASSERT_EQ(first_selected.report.status,s::Status::Ok);EXPECT_EQ(first_selected.result.face_count,6u);
  EXPECT_EQ(first_selected.result.counts.shell_suppressed_faces,1u);
  // Earlier node-corner incidence beats the family's array row order.
  c.quads={{501,20,{5,6,7,4}},{502,30,{4,5,6,7}}};Built corners(c);
  ASSERT_EQ(corners.report.status,s::Status::Ok);EXPECT_EQ(corners.result.face_count,6u);
}
TEST(Type25SurfaceSource, OnlyActuallyEmittedSolidSegmentsSetTheInterfaceSolidFlags) {
  auto c=SingleHex();c.parts={10,30};
  c.quads={{201,30,{3,2,1,0}},{202,30,{4,5,6,7}},{203,30,{0,1,5,4}},
      {204,30,{2,3,7,6}},{205,30,{1,2,6,5}},{206,30,{0,4,7,3}}};
  Built b(c);ASSERT_EQ(b.report.status,s::Status::Ok);
  EXPECT_EQ(b.result.face_count,6u);EXPECT_EQ(b.result.counts.selected_solids,1u);
  EXPECT_EQ(b.result.counts.solid_faces,0u);EXPECT_EQ(b.result.counts.shell_suppressed_faces,6u);
  ASSERT_EQ(b.result.solid_count,1u);EXPECT_EQ(b.result.surface_solid_flags[0],0);
  c.parts.clear();Built empty(c);ASSERT_EQ(empty.report.status,s::Status::Ok);
  EXPECT_EQ(empty.result.face_count,0u);EXPECT_EQ(empty.result.faces,nullptr);
  EXPECT_EQ(empty.result.surface_solid_flags[0],0);EXPECT_TRUE(empty.report.count_complete);
}
TEST(Type25SurfaceSource, ForecastChecksDescriptorsBeforeBorrowedDataAndCapsAreExact) {
  auto c=SingleHex();const auto in=c.Input();s::Forecast f;
  ASSERT_EQ(s::Preflight(in,{},f).status,s::Status::Ok);
  s::Limits cap;cap.output_bytes=f.output_bytes;cap.scratch_bytes=f.scratch_bytes;
  Built exact(c,cap);ASSERT_EQ(exact.report.status,s::Status::Ok);
  auto short_output=cap;--short_output.output_bytes;s::Forecast unchanged=f;
  EXPECT_EQ(s::Preflight(in,short_output,unchanged).status,s::Status::ResourceLimit);
  EXPECT_EQ(unchanged.output_bytes,f.output_bytes);
  auto short_scratch=cap;--short_scratch.scratch_bytes;
  EXPECT_EQ(s::Preflight(in,short_scratch,unchanged).status,s::Status::ResourceLimit);
  auto excessive=in;excessive.solid_count=SIZE_MAX;excessive.solids=reinterpret_cast<const s::Solid*>(1);
  EXPECT_EQ(s::Preflight(excessive,{},unchanged).status,s::Status::ResourceLimit);
}
TEST(Type25SurfaceSource, LateInvalidPhysicalRowsAndAliasFailuresPreserveAllPublicOutput) {
  auto c=Adjacent();Built b(c);ASSERT_EQ(b.report.status,s::Status::Ok);
  const auto before=Bytes(b.output);const auto snapshot=b.result;
  c.solids.back().nodes[7]=UINT32_MAX;
  EXPECT_EQ(s::Build(c.Input(),{},b.output,b.scratch,&b.result).status,s::Status::InvalidInput);
  EXPECT_EQ(Bytes(b.output),before);SameSnapshot(b.result,snapshot);
  c=Adjacent();auto input=c.Input();input.solids=static_cast<const s::Solid*>(b.output.data());
  EXPECT_EQ(s::Build(input,{},b.output,b.scratch,&b.result).status,s::Status::InvalidInput);
  EXPECT_EQ(Bytes(b.output),before);SameSnapshot(b.result,snapshot);
  input=c.Input();
  EXPECT_EQ(s::Build(input,{},b.output,b.output,&b.result).status,s::Status::InvalidInput);
  EXPECT_EQ(Bytes(b.output),before);
  EXPECT_EQ(s::Build(input,{},b.output,b.scratch,reinterpret_cast<s::Snapshot*>(b.scratch.data())).status,s::Status::InvalidInput);
  EXPECT_EQ(Bytes(b.output),before);
  ASSERT_EQ(s::Build(input,{},b.output,b.scratch,&b.result).status,s::Status::Ok);
  EXPECT_EQ(b.result.face_count,snapshot.face_count);
}
TEST(Type25SurfaceSource, UnsupportedPhaseAndTopologyNeverBecomeAnImplicitRepair) {
  auto c=SingleHex();Built b(c);ASSERT_EQ(b.report.status,s::Status::Ok);
  const auto before=Bytes(b.output);auto in=c.Input();in.phase=s::ReaderPhase::Unspecified;
  EXPECT_EQ(s::Build(in,{},b.output,b.scratch,&b.result).status,s::Status::UnsupportedProfile);
  EXPECT_EQ(Bytes(b.output),before);
  c.solids[0].nodes[1]=c.solids[0].nodes[0];
  EXPECT_EQ(s::Build(c.Input(),{},b.output,b.scratch,&b.result).status,s::Status::UnsupportedProfile);
  EXPECT_EQ(Bytes(b.output),before);
  c.solids[0]=Penta();c.solids[0].nodes[7]=2;
  EXPECT_EQ(s::Build(c.Input(),{},b.output,b.scratch,&b.result).status,s::Status::InvalidInput);
  EXPECT_EQ(Bytes(b.output),before);
}
TEST(Type25SurfaceSource, ForecastAndArenaObjectAliasesRejectBeforeAnyOwnerBytesChange) {
  auto c=SingleHex();auto input=c.Input();Built b(c);
  ASSERT_EQ(b.report.status,s::Status::Ok);
  const auto source=c.solids[0];const auto header=input;
  EXPECT_EQ(s::Preflight(input,{},*reinterpret_cast<s::Forecast*>(&input)).status,s::Status::InvalidInput);
  EXPECT_EQ(std::memcmp(&input,&header,sizeof(input)),0);
  EXPECT_EQ(s::Preflight(input,{},*reinterpret_cast<s::Forecast*>(c.solids.data())).status,s::Status::InvalidInput);
  EXPECT_EQ(std::memcmp(&c.solids[0],&source,sizeof(source)),0);
  const auto output_before=Bytes(b.output);
  std::array<unsigned char,sizeof(tl::util::HostArena)> handle{};
  std::memcpy(handle.data(),&b.scratch,handle.size());
  EXPECT_EQ(s::Build(input,{},b.output,b.scratch,reinterpret_cast<s::Snapshot*>(&b.scratch)).status,s::Status::InvalidInput);
  EXPECT_EQ(std::memcmp(handle.data(),&b.scratch,handle.size()),0);
  EXPECT_EQ(Bytes(b.output),output_before);
  // A valid separately allocated arena handle itself can reside in another
  // arena payload. Publishing there would corrupt the handle unless guarded.
  tl::util::HostArena outer;
  ASSERT_TRUE(outer.Initialize(b.forecast.output_bytes));
  auto* embedded=::new(outer.data()) tl::util::HostArena;
  ASSERT_TRUE(embedded->Initialize(b.forecast.scratch_bytes));
  const auto outer_before=Bytes(outer);s::Snapshot result;
  EXPECT_EQ(s::Build(input,{},outer,*embedded,&result).status,s::Status::InvalidInput);
  EXPECT_EQ(Bytes(outer),outer_before);
  embedded->~HostArena();
}

}
