#include "Assertions.h"
namespace type25_interface_surface_test {
TEST(Type25InterfaceNative, CompleteClassificationFilterAndTrueMixedSideCountsMatch) {
  auto solid=SolidOnly();Compare(solid);
  Built only(solid);ASSERT_EQ(only.report.status,f::Status::Ok);
  EXPECT_EQ(only.result.shell_primary_count,0u);EXPECT_EQ(only.result.main_count,only.result.primary_count);
  auto mixed=Mixed();Compare(mixed);Compare(Mixed(true));
  Built b(mixed);ASSERT_EQ(b.report.status,f::Status::Ok);
  EXPECT_EQ(b.result.shell_primary_count,3u);
  EXPECT_NE(b.result.main_count,2*b.result.primary_count);
  Case triangles;triangles.physical.triangles={{301,10,{0,1,2,2}}};triangles.Extract();Compare(triangles);
}
TEST(Type25InterfaceNative, SuppliedPentaIncidenceOrderChangesTheObservedFirstCoatingSign) {
  auto c=FirstSolid();
  const auto first=Oracle(c.Input());Compare(c);
  ASSERT_EQ(first.classifications.size(),1u);
  ASSERT_EQ(first.classifications[0].matched_solid,0u);
  std::reverse(c.physical.solids.begin(),c.physical.solids.end());c.Extract();
  const auto second=Oracle(c.Input());Compare(c);
  ASSERT_EQ(second.classifications.size(),1u);
  EXPECT_EQ(second.classifications[0].matched_solid,0u);
  EXPECT_EQ(first.classifications[0].role,-second.classifications[0].role);
  EXPECT_NE(first.classifications[0].role,0);
}
TEST(Type25InterfaceNative, UnsignedRoleKeysAndSavedOrientationFollowActualNativeFilter) {
  auto c=SolidOnly();
  c.physical.quads={{201,20,{0,1,2,3}},{202,30,{0,1,2,3}}};c.physical.parts={10,30};c.Extract();
  // Explicit numerical reader order: the first unselected physical shell lets
  // both raw solid and selected shell occurrences reach native I25SURFI.
  const auto native=Oracle(c.Input());Compare(c);
  ASSERT_GE(native.primary.size(),2u);
  EXPECT_EQ(native.identities[0].kind,s::PrimaryFaceKind::Solid);
  EXPECT_EQ(native.primary[1].source_id,202u);
  EXPECT_EQ(native.primary[1].side_role,s::ShellSideRole::CoatingReversed);
  const std::uint32_t saved[]{3,2,1,0};
  for(unsigned k=0;k<4;++k)EXPECT_EQ(native.primary[0].nodes[k],saved[k]);
}
TEST(Type25InterfaceNative, EveryCoalescedOriginAndSolidFlagSurvivesWithoutInventedOwner) {
  auto c=Origins();const auto native=Oracle(c.Input());Compare(c);
  ASSERT_EQ(native.surface_solid_flags,(std::vector<std::uint8_t>{1,1}));
  std::size_t multiple=0;
  for(std::size_t i=0;i<native.primary.size();++i) {
    const auto& identity=native.identities[i];
    if(identity.origin==s::PrimaryOrigin::MultipleOrigins) {
      ++multiple;EXPECT_EQ(identity.physical_parent_id,0u);EXPECT_EQ(identity.local_face,0u);
      EXPECT_GE(identity.origin_count,2u);EXPECT_EQ(native.primary[i].source_id,0u);
    }
  }
  EXPECT_GT(multiple,0u);
  EXPECT_EQ(native.raw_to_primary.size(),c.raw.size());
  std::reverse(c.physical.solids.begin(),c.physical.solids.end());c.Extract();Compare(c);
}
TEST(Type25InterfaceNative, GeneratedRaw8PentaAndCoordinateUnitCouponsRetainAllNativeFields) {
  for(unsigned seed=0;seed<16;++seed) {
    SCOPED_TRACE(seed);
    auto c=seed&1?Mixed(seed&2):Origins();
    if(seed&4) {
      c.coordinates=s::Coordinates::Si;c.units={.01,2.,.5};
      for(auto& x:c.points){x.x*=c.units.length_m;x.y*=c.units.length_m;x.z*=c.units.length_m;}
    }
    if(seed&8)std::reverse(c.raw.begin(),c.raw.end()); // Exact prescribed raw occurrence operand.
    Compare(c);
  }
}
}
