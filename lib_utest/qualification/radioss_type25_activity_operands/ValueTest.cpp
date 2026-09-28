// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "lib_src/collision/radioss_type25/activity_operands/Storage.h"
#include "lib_src/collision/radioss_type25/activity_operands/Values.h"
#include <gtest/gtest.h>
namespace activity_operands_test {
namespace n = tlfea::contact::radioss_type25; namespace a = n::activity_operands;
namespace native = type25_activity_native;
native::SurfaceCase TwoLayers(bool first, bool second, double coefficient = 20) {
  native::Element q; q.nodes = {1,2,3,4}; q.off = first ? 1 : 0;
  auto other = q; other.off = second ? 1 : 0;
  native::SurfaceCase c; c.mesh = {4,{q,other}}; c.corners = {{1,2,3,4}};
  c.coefficients = {coefficient}; c.affected = {1}; return c;
}
TEST(ActivityOperandValues, NativeContainingSurvivorAndCompleteRemoval) {
  const n::activity_source::Controls controls{n::activity_source::Deletion::ContainingElement,false,n::startup::SolidErosion::Disabled};
  for (bool survivor : {false,true}) {
    auto c = TwoLayers(false,survivor); c.affected = survivor ? std::vector<int>{1} : std::vector<int>{1,1};
    const auto oracle = native::Surfaces(c);
    const auto value = a::detail::Main(20,0,unsigned(c.affected.size()),survivor,controls);
    ASSERT_TRUE(value.valid); EXPECT_FALSE(value.exposure); EXPECT_EQ(value.coefficient,oracle.coefficients[0]);
    EXPECT_EQ(value.removed ? c.affected.size() : 0,oracle.removed.size());
  }
}
TEST(ActivityOperandValues, NativeNegativeExposureChecksIntermediateMultiplicity) {
  const n::activity_source::Controls controls{n::activity_source::Deletion::ContainingElement,false,n::startup::SolidErosion::Enabled};
  for (unsigned events : {1u,2u}) for (bool survivor : {false,true}) {
    auto c = TwoLayers(false,survivor,-20); c.solid_erosion=true; c.connected_elements={2};
    c.affected.assign(events,1); const auto oracle=native::Surfaces(c);
    const auto value=a::detail::Main(-20,2,events,survivor,controls);
    ASSERT_TRUE(value.valid); EXPECT_EQ(value.connected,oracle.connected_elements[0]);
    EXPECT_EQ(value.exposure,!oracle.exposed.empty());
    if(!value.exposure)EXPECT_EQ(value.coefficient,oracle.coefficients[0]);
  }
}
TEST(ActivityOperandValues, AlreadyZeroPreservesNativeRemovalEventMultiplicity) {
  auto c=TwoLayers(false,false,0);c.solid_erosion=true;c.connected_elements={0};c.affected={1,1};
  const auto oracle=native::Surfaces(c);
  const auto value=a::detail::Main(0,0,2,false,{n::activity_source::Deletion::ContainingElement,false,n::startup::SolidErosion::Enabled});
  ASSERT_TRUE(value.valid);EXPECT_TRUE(value.removed);EXPECT_EQ(value.connected,-2);
  EXPECT_EQ(value.connected,oracle.connected_elements[0]);EXPECT_EQ(oracle.removed.size(),2u);
  EXPECT_EQ(value.coefficient,0); // Zero was already zero; unique new-zero count is separately zero.
}
TEST(ActivityOperandValues, NativeOrphanMarkersAndWallRetention) {
  const auto tags=native::Tag(TwoLayers(false,false).mesh);
  for(bool registered:{false,true}) {
    const auto oracle=native::Secondaries(tags,{1,2,3},{10,0,-0.0},registered);
    const n::activity_source::Controls controls{registered?n::activity_source::Deletion::ContainingElement:n::activity_source::Deletion::Disabled,false,n::startup::SolidErosion::Disabled};
    const double before[]{10,0,-0.0};
    for(unsigned i=0;i<3;++i) {
      const auto marked=a::detail::MarkSecondary(before[i],false,controls);
      EXPECT_TRUE(a::detail::Same(marked,oracle.marked_coefficients[i]));
      EXPECT_EQ(a::detail::NormalizeSecondary(marked),registered?0:before[i]);
    }
  }
}
TEST(ActivityOperandValues, ExactLayoutBoundaryAndOverflow) {
  a::detail::Shape shape;shape.nodes=9;shape.parents=3;shape.mains=4;shape.primaries=2;
  shape.secondaries=7;shape.incidence=12;shape.containing=3;shape.emitting=6;shape.normals=true;
  a::detail::Layout layout;ASSERT_TRUE(a::detail::MakeLayout(shape,256,1u<<20,layout));
  const auto bytes=layout.bytes;EXPECT_FALSE(a::detail::MakeLayout(shape,256,bytes-1,layout));EXPECT_EQ(layout.bytes,bytes);
  shape.parents=SIZE_MAX;EXPECT_FALSE(a::detail::MakeLayout(shape,256,SIZE_MAX,layout));EXPECT_EQ(layout.bytes,bytes);
}
} // namespace activity_operands_test
