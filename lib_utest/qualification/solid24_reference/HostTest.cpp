// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include <gtest/gtest.h>
#include <cstring>

namespace solid24_test {
TEST(Solid24Reference, BrickDimensionsMassAndCyclicFrame) {
  const auto input=Brick(); s::Reference r;
  ASSERT_EQ(s::InitializeReference(input,r),s::Status::Success);
  EXPECT_NEAR(r.geometry().volume_m3,.02*.03*.04,1e-19);
  EXPECT_NEAR(r.geometry().characteristic_length_m,.02,1e-15);
  EXPECT_NEAR(r.mass().element_mass_kg,1980*.02*.03*.04,1e-15);
  EXPECT_EQ(r.geometry().frame.v[0],1);
  EXPECT_EQ(r.geometry().frame.v[4],1);
  EXPECT_EQ(r.geometry().frame.v[8],1);
  EXPECT_EQ(r.unique_node_count(),8u);
  for(unsigned n=0;n<8;++n) {
    EXPECT_EQ(r.source_slot(n),n);
    EXPECT_EQ(r.mass().source_slot_mass_kg[n],r.mass().source_slot_mass_kg[0]);
  }
}
TEST(Solid24Reference, ReversedOrientationAndInputAlias) {
  auto input=Distorted();
  for(unsigned n=0;n<4;++n) {std::swap(input.position_m[n],input.position_m[n+4]);std::swap(input.source_node_id[n],input.source_node_id[n+4]);}
  s::Reference r;
  ASSERT_EQ(s::InitializeReference(input,r),s::Status::Success);
  for(unsigned n=0;n<8;++n) EXPECT_EQ(r.source_slot(n),(n+4)%8);
  const auto before=Values(r);
  ASSERT_EQ(s::InitializeReference(r.input(),r),s::Status::Success);
  EXPECT_EQ(Values(r),before);
}
TEST(Solid24Reference, RejectsWedgeAndLateInvalidInputWithoutPublication) {
  s::Reference r; ASSERT_EQ(s::InitializeReference(Distorted(),r),s::Status::Success);
  std::array<unsigned char,sizeof(r)> bytes;std::memcpy(bytes.data(),&r,sizeof(r));
  auto input=Brick(); input.source_node_id[5]=input.source_node_id[4]; input.source_node_id[7]=input.source_node_id[6];
  EXPECT_EQ(s::InitializeReference(input,r),s::Status::UnsupportedProfile);
  EXPECT_EQ(std::memcmp(bytes.data(),&r,sizeof(r)),0);
  input=Brick();input.position_m[7].x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(s::InitializeReference(input,r),s::Status::InvalidInput);
  EXPECT_EQ(std::memcmp(bytes.data(),&r,sizeof(r)),0);
  input=Brick();for(auto& x:input.position_m)x.z=0;
  EXPECT_EQ(s::InitializeReference(input,r),s::Status::InvalidGeometry);
  EXPECT_EQ(std::memcmp(bytes.data(),&r,sizeof(r)),0);
  input=Brick();input.profile.rotational_inertia=1;
  EXPECT_EQ(s::InitializeReference(input,r),s::Status::UnsupportedProfile);
  EXPECT_EQ(std::memcmp(bytes.data(),&r,sizeof(r)),0);
  EXPECT_EQ(s::InitializeReference(Brick(),r),s::Status::Success);
}
}  // namespace solid24_test
