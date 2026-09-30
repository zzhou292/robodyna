// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "SourceFixture.h"
#include <gtest/gtest.h>
namespace solid6z_test {
TEST(Solid6zSource, All195MappedOriginalWedgesMatchCompleteNativeStartup) {
  unsigned count=0,reversed=0; double total=0;
  for (unsigned i=0;i<SourceCount;++i) {
    s::ReferenceInput input;
    if (!Source(i,input)) continue;
    SCOPED_TRACE(input.source_element_id);
    s::Reference result; const auto native=Native(input);
    ASSERT_EQ(native.status,0);
    ASSERT_EQ(s::InitializeReference(input,result),s::Status::Success);
    ASSERT_TRUE(Agree(Values(result),native.values));
    for (unsigned n=0;n<6;++n) {
      ASSERT_EQ(result.source_slot(n),unsigned(native.permutation[n]));
      ASSERT_EQ(result.input().source_node_id[n],input.source_node_id[n]);
      ASSERT_GT(result.mass().source_slot_mass_kg[n],0);
    }
    s::ReferenceInput working; ASSERT_TRUE(Source(i,working,true));
    const auto native_working=Native(working); ASSERT_EQ(native_working.status,0);
    ASSERT_EQ(native_working.permutation,native.permutation);
    auto si=native_working.values;
    for (unsigned n=9;n<27;++n) si[n]*=.001;
    for (unsigned n=27;n<36;++n) si[n]*=1000;
    for (unsigned n=36;n<39;++n) si[n]*=1e-9;
    si[39]*=.001;
    for (unsigned n=40;n<47;++n) si[n]*=1000;
    double world=0;
    for (const auto& x:input.position_m)
      world=std::max({world,std::abs(x.x),std::abs(x.y),std::abs(x.z)});
    const double conditioning=world/result.geometry().characteristic_length_m;
    const double bound=64+256*std::max(1.,conditioning);
    ASSERT_TRUE(Agree(Values(result),si,bound));
    auto altered=si; altered[40]*=1.001;
    EXPECT_FALSE(Agree(Values(result),altered,bound));
    total+=result.mass().element_mass_kg; ++count; reversed+=result.source_slot(0)!=0;
  }
  EXPECT_EQ(count,195u);
  RecordProperty("mapped_original_wedges",count);
  RecordProperty("orientation_reversals",reversed);
  RecordProperty("wedge_only_mass_kg",std::to_string(total));
}
}  // namespace solid6z_test
