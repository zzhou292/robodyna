// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "Observed.h"
namespace type25_startup_test {
TEST(Type25FixedStartup, ActualObservedEightTriangleWallFieldsAreComputedFromPrimaryMesh) {
  Case source;source.ids.assign(std::begin(observed::Ids),std::end(observed::Ids));
  source.positions.assign(std::begin(observed::Positions),std::end(observed::Positions));
  for(unsigned i=0;i<8;++i)source.Add(n::ShellLayout::Triangle3,
      observed::PrimaryNodes[4*i],observed::PrimaryNodes[4*i+1],observed::PrimaryNodes[4*i+2],observed::PrimaryNodes[4*i+3]);
  const Built built(source);Same(built,Oracle(source.Input(),source.coefficients.data(),source.coefficients.size()));
  ASSERT_EQ(built.startup.main_count,16u);ASSERT_EQ(built.ready.normals.reference_count,18u);
  for(std::size_t m=0;m<16;++m) {
    SCOPED_TRACE(m);
    const auto& main=built.startup.mains[m];EXPECT_EQ(main.segment_type,observed::Roles[m]);EXPECT_EQ(main.global_id,observed::Globals[m]);
    for(unsigned k=0;k<4;++k) {
      SCOPED_TRACE(k);
      EXPECT_EQ(main.nodes[k]+1,observed::Connectivity[4*m+k]);EXPECT_EQ(main.neighbors[k],observed::Neighbors[4*m+k]);
      EXPECT_EQ(main.normal_reference[k],observed::References[4*m+k]);
      const auto value=built.ready.normals.face_normals[4*m+k];const auto index=3*(4*m+k);
      EXPECT_EQ(Bits(value.x),observed::NormalBits[index])
          << " Starter x bits=" << Bits(built.startup.starter.face_normals[4*m+k].x);EXPECT_EQ(Bits(value.y),observed::NormalBits[index+1]);
      EXPECT_EQ(Bits(value.z),observed::NormalBits[index+2]);
    }
  }
  for(std::size_t ref=0;ref<18;++ref) {
    EXPECT_EQ(built.ready.normals.references[ref].boundary,observed::Bounds[ref]);
    for(unsigned k=0;k<2;++k) {
      const auto value=built.ready.normals.references[ref].bisector[k];const auto index=6*ref+3*k;
      EXPECT_EQ(Bits(value.x),observed::BisectorBits[index]);EXPECT_EQ(Bits(value.y),observed::BisectorBits[index+1]);
      EXPECT_EQ(Bits(value.z),observed::BisectorBits[index+2]);
    }
  }
}
} // namespace type25_startup_test
