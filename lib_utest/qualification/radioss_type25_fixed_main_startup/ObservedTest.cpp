// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "Observed.h"
namespace type25_startup_test {
TEST(Type25FixedStartup, ObservedTopologyAndAllActiveNativeStagesHaveDistinctPhaseEvidence) {
  Case source;source.ids.assign(std::begin(observed::Ids),std::end(observed::Ids));
  source.positions.assign(std::begin(observed::Positions),std::end(observed::Positions));
  for(unsigned i=0;i<8;++i)source.Add(n::ShellLayout::Triangle3,
      observed::PrimaryNodes[4*i],observed::PrimaryNodes[4*i+1],observed::PrimaryNodes[4*i+2],observed::PrimaryNodes[4*i+3]);
  const Built built(source);
  // Both stages, every REAL4 bit, including signed zeros, compare to the exact
  // independently compiled native stages on these same absolute coordinates.
  Same(built,Oracle(source.Input(),source.coefficients.data(),source.coefficients.size()));
  ASSERT_EQ(built.startup.main_count,16u);ASSERT_EQ(built.ready.normals.reference_count,18u);
  // This immutable topology is observed later at cycle216 and is still valid
  // for this non-eroding source. Later ACTNOR/TAGNOD numerical fields are not
  // an initial all-active-stage oracle.
  for(std::size_t m=0;m<16;++m) {
    SCOPED_TRACE(m);
    const auto& main=built.startup.mains[m];
    EXPECT_EQ(main.segment_type,observed::Roles[m]);EXPECT_EQ(main.global_id,observed::Globals[m]);
    for(unsigned k=0;k<4;++k) {
      SCOPED_TRACE(k);
      EXPECT_EQ(main.nodes[k]+1,observed::Connectivity[4*m+k]);EXPECT_EQ(main.neighbors[k],observed::Neighbors[4*m+k]);
      EXPECT_EQ(main.normal_reference[k],observed::References[4*m+k]);
    }
  }
  // Preserve the discovered phase evidence explicitly. At cycle216 the three
  // used slots of native main4 contain -0 in X; the all-active initial stage
  // produces +0 after averaging. Neither input nor output is normalized here.
  EXPECT_EQ(observed::InputCycle,0);
  EXPECT_EQ(observed::ClassificationCycle,216);
  EXPECT_GT(observed::ClassificationTime,0.);
  for(unsigned k:{0u,1u,3u}) {
    EXPECT_EQ(observed::NormalBits[3*(4*3+k)],0x80000000u);
    EXPECT_EQ(Bits(built.ready.normals.face_normals[4*3+k].x),0u);
  }
}
} // namespace type25_startup_test
