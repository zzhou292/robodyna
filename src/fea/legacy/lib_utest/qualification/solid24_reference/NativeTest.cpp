// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <gtest/gtest.h>
namespace solid24_test {
TEST(Solid24Native, CompleteSelectedVolumeLengthFrameAndMass) {
  auto reverse=Distorted();
  for(unsigned n=0;n<4;++n)std::swap(reverse.position_m[n],reverse.position_m[n+4]);
  for(const auto& input:{Brick(),Distorted(),reverse}) {
    const auto native=Native(input);ASSERT_EQ(native.status,0);
    s::Reference r;ASSERT_EQ(s::InitializeReference(input,r),s::Status::Success);
    ASSERT_TRUE(Agree(Values(r),native.values));
    for(unsigned n=0;n<8;++n)EXPECT_EQ(r.source_slot(n),unsigned(native.permutation[n]));
  }
}
TEST(Solid24Native, NativeCollapsedRejection) {
  auto input=Brick();for(auto& x:input.position_m)x.z=0;
  EXPECT_NE(Native(input).status,0);
  s::Reference r;EXPECT_EQ(s::InitializeReference(input,r),s::Status::InvalidGeometry);
}
TEST(Solid24Native, PositiveVolumeWithNativeSmallFaceLengthFactor) {
  auto input=Brick();
  const s::Vec3 x[8]{{0,0,0},{1,0,0},{0,1e-5,0},{0,1e-5,0},
                     {0,0,.1},{0,0,.1},{0,0,.1},{0,0,.1}};
  for(unsigned n=0;n<8;++n)input.position_m[n]=x[n];
  const auto native=Native(input);ASSERT_EQ(native.status,0);
  s::Reference r;ASSERT_EQ(s::InitializeReference(input,r),s::Status::Success);
  ASSERT_TRUE(Agree(Values(r),native.values));
  const unsigned face[6][4]{{0,1,2,3},{4,5,6,7},{0,1,5,4},{1,2,6,5},{2,3,7,6},{3,0,4,7}};
  const auto& local=r.geometry().local_position_m;double maximum=0,areas[6];
  for(unsigned f=0;f<6;++f) {
    areas[f]=s::detail::FaceMeasure(local[face[f][0]],local[face[f][1]],local[face[f][2]],local[face[f][3]]);
    maximum=std::max(maximum,areas[f]);
  }
  unsigned small=0;for(double area:areas)small+=area<1e-4*maximum;
  ASSERT_GE(small,3u);
  EXPECT_NEAR(r.geometry().characteristic_length_m/(4*r.geometry().volume_m3/std::sqrt(maximum)),1000,1e-10);
}
}  // namespace solid24_test
