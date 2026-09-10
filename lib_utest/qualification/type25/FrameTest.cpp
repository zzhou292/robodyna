// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <gtest/gtest.h>
#include <cstring>
#include <limits>

namespace type25_test {
TEST(Type25Frame,FiniteOffsetReferenceAndParallelSeedFallback) {
  for(const auto chord:{spring::Vec3{.007,.002,.001},spring::Vec3{0,.01,0}}) {
    const spring::Vec3 nodes[2]{{},chord};spring::Reference ref;
    ASSERT_EQ(spring::InitializeReference({1000,.001,1},nodes,{},ref),spring::Status::Success);
    const auto x=tl::math::Divide(chord,ref.length_m);
    EXPECT_TRUE(tl::math::Orthonormal(tl::math::Columns(x,ref.transverse_axis,tl::math::Cross(x,ref.transverse_axis))));
    if(chord.x==0)EXPECT_DOUBLE_EQ(ref.transverse_axis.x,1);
  }
}
TEST(Type25Frame,TranslationPreservesFrameAndFailureIsAtomic) {
  Fixture f;spring::Reference reference;const auto& c=f.connections[2];
  ASSERT_EQ(spring::InitializeReference(f.Input().source_units,c.position,c.seed,reference),spring::Status::Success);
  spring::EndpointKinematics nodes[2]{{c.position[0],{8,0,0},{}},{c.position[1],{8,0,0},{}}};
  spring::Frame frame;ASSERT_EQ(spring::AdvanceFrame(f.Input().source_units,reference.transverse_axis,nodes,1e-8,frame),spring::Status::Success);
  EXPECT_TRUE(tl::math::Orthonormal(frame.axes));EXPECT_DOUBLE_EQ(frame.length_m,reference.length_m);
  const auto before=frame;nodes[1].position=nodes[0].position;
  EXPECT_EQ(spring::AdvanceFrame(f.Input().source_units,reference.transverse_axis,nodes,1e-8,frame),spring::Status::DegenerateGeometry);
  EXPECT_EQ(std::memcmp(&frame,&before,sizeof(frame)),0);
  nodes[1].position.z=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(spring::AdvanceFrame(f.Input().source_units,reference.transverse_axis,nodes,1e-8,frame),spring::Status::InvalidInput);
  EXPECT_EQ(std::memcmp(&frame,&before,sizeof(frame)),0);
}
} // namespace type25_test
