// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "SourceFixture.h"
#include <gtest/gtest.h>
namespace solid24_test {
TEST(Solid24Source, All1309OriginalBricksMatchSelectedNativeStartup) {
  unsigned count=0,reversed=0;double total=0;
  for(unsigned i=0;i<SourceCount;++i) {
    const auto input=Source(i);if(!IsBrick(input))continue;
    SCOPED_TRACE(input.source_element_id);
    s::Reference r;const auto native=Native(input);
    ASSERT_EQ(native.status,0);
    ASSERT_EQ(s::InitializeReference(input,r),s::Status::Success);
    ASSERT_TRUE(Agree(Values(r),native.values));
    for(unsigned n=0;n<8;++n) {
      ASSERT_EQ(r.source_slot(n),unsigned(native.permutation[n]));
      ASSERT_EQ(r.input().source_node_id[n],input.source_node_id[n]);
      ASSERT_GT(r.mass().source_slot_mass_kg[n],0);
    }
    const auto working=Native(Source(i,true));ASSERT_EQ(working.status,0);
    ASSERT_EQ(working.permutation,native.permutation);
    auto si=working.values;
    for(unsigned n=9;n<33;++n)si[n]*=.001;
    si[33]*=1e-9;si[34]*=.001;
    for(unsigned n=35;n<44;++n)si[n]*=1000;
    double world=0;
    for(const auto& x:input.position_m)world=std::max({world,std::abs(x.x),std::abs(x.y),std::abs(x.z)});
    const double conditioning=world/r.geometry().characteristic_length_m;
    ASSERT_TRUE(AgreeWorkingUnits(Values(r),si,conditioning));
    if(input.source_element_id==2191071) {
      // This cell has a cancelling frame component. Unit roundoff must not
      // conceal a physical frame perturbation or a changed source mass.
      auto changed=si;changed[5]+=1e-6;
      EXPECT_FALSE(AgreeWorkingUnits(Values(r),changed,conditioning));
      changed=si;changed[35]*=1.001;
      EXPECT_FALSE(AgreeWorkingUnits(Values(r),changed,conditioning));
    }
    total+=r.mass().element_mass_kg;++count;reversed+=r.source_slot(0)!=0;
  }
  EXPECT_EQ(count,1309u);RecordProperty("original_bricks",count);
  RecordProperty("orientation_reversals",reversed);RecordProperty("brick_only_mass_kg",std::to_string(total));
}
TEST(Solid24Source, All195OriginalWedgesRequireSeparateSelectedDispatcher) {
  unsigned count=0;
  for(unsigned i=0;i<SourceCount;++i) {
    const auto input=Source(i);if(IsBrick(input))continue;
    s::Reference r;
    EXPECT_EQ(s::InitializeReference(input,r),s::Status::UnsupportedProfile);
    EXPECT_FALSE(r.prepared());++count;
  }
  EXPECT_EQ(count,195u);
}
}  // namespace solid24_test
