// SPDX-License-Identifier: AGPL-3.0-or-later
#include "PhysicalMainFixture.h"
namespace type25_physical_main_test {
using Status=n::TransactionStatus;
TEST(NativeMixedPhysicalMains, RealSourcesKeepEveryOriginAndSeparateFinalSupport) {
  Fixture f;const auto report=f.Validate();
  ASSERT_EQ(report.report.status,Status::Ok)<<report.report.message;
  EXPECT_EQ(f.sides.main_count,5u);EXPECT_EQ(f.sides.primary_to_partner[2],0u);
  EXPECT_EQ(f.sides.primary_identities[2].physical_parent_id,0u);
  EXPECT_EQ(f.post.final_support[2].first.source_element_id,9101u);
  // Incidence accepts a real coincident QEPH support independently of the
  // QBAT raw origin. This does not choose or certify native INCOQ3 ranking.
  f.supports[0].first.source_element_id=100;
  EXPECT_EQ(f.Validate().report.status,Status::Ok);
}
TEST(NativeMixedPhysicalMains, GenuineQ4SupportsTriangularSubsetAndPentaKeepsRawSlots) {
  Fixture f;pm::Face q;bool triangle=false;
  ASSERT_TRUE(f.index.Shell(103,q,triangle));ASSERT_FALSE(triangle);
  const pm::Face subset{q[0],q[1],q[2],q[2]};
  EXPECT_TRUE(pm::Support(f.index,{{s::PhysicalSupportKind::ShellQuad,103},0},subset));
  EXPECT_FALSE(pm::Support(f.index,{{s::PhysicalSupportKind::ShellTriangle,103},0},subset));
  std::array<std::uint32_t,8> nodes;ASSERT_TRUE(f.index.Solid(9102,nodes));
  EXPECT_EQ(nodes[0],nodes[3]);EXPECT_EQ(nodes[4],nodes[7]);
  n::source_surfaces::Solid solid;for(unsigned k=0;k<8;++k)solid.nodes[k]=nodes[k];
  for(unsigned face=0;face<6;++face) {
    pm::Face polygon{};const auto count=n::source_surfaces::detail::CompactFace(solid,face,polygon.data());
    if(count<3)continue;
    if(count==3)polygon[3]=polygon[2];
    EXPECT_TRUE(pm::Origin(f.index,{s::PrimaryFaceKind::Solid,9102,std::uint8_t(face+1)},polygon));
  }
}
TEST(NativeMixedPhysicalMains, WrongOwnerMissingOriginAndFakeRepresentativeAreRejected) {
  Fixture f;
  f.supports[2].first.source_element_id=103;
  EXPECT_EQ(f.Validate().report.status,Status::SourceMismatch);
  f.supports[2].first.source_element_id=9101;
  auto origins=f.origins;origins[3].physical_parent_id=9919;
  auto bad=f.sides;bad.raw_origins=origins.data();
  EXPECT_EQ(rd::ValidateMixedPhysicalMains(f.physical.physical,bad,f.post,1u<<20).report.status,Status::SourceMismatch);
  auto identities=f.identities;identities[2].physical_parent_id=9100;
  bad=f.sides;bad.primary_identities=identities.data();
  EXPECT_EQ(rd::ValidateMixedPhysicalMains(f.physical.physical,bad,f.post,1u<<20).report.status,Status::SourceMismatch);
  bad=f.sides;--bad.raw_origin_count;
  EXPECT_EQ(rd::ValidateMixedPhysicalMains(f.physical.physical,bad,f.post,1u<<20).report.status,Status::SourceMismatch);
  EXPECT_EQ(f.Validate().report.status,Status::Ok);
}
TEST(NativeMixedPhysicalMains, NonfaceDiagonalAndExactResourceBoundaryDoNotPublish) {
  Fixture f;const auto valid=f.Validate();ASSERT_EQ(valid.report.status,Status::Ok);
  EXPECT_EQ(f.Validate(valid.startup_host_bytes).report.status,Status::Ok);
  EXPECT_EQ(f.Validate(valid.startup_host_bytes-1).report.status,Status::ResourceLimit);
  std::array<s::Main,5> mains;
  std::copy_n(f.sides.mains,5,mains.data());std::swap(mains[2].nodes[1],mains[2].nodes[2]);
  auto bad=f.sides;bad.mains=mains.data();
  EXPECT_EQ(rd::ValidateMixedPhysicalMains(f.physical.physical,bad,f.post,1u<<20).report.status,Status::SourceMismatch);
  auto foreign=f.post;foreign.source_generation++;
  EXPECT_EQ(rd::ValidateMixedPhysicalMains(f.physical.physical,f.sides,foreign,1u<<20).report.status,Status::InvalidInput);
  EXPECT_EQ(f.Validate().report.status,Status::Ok);
}
} // namespace type25_physical_main_test
