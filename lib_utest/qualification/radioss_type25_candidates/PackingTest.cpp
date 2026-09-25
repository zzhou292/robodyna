#include "PackingFixture.h"
#include <gtest/gtest.h>
using namespace candidate_test;
TEST(NativeCandidatePacking,CompleteCor3tNativeGapAndConstraintCodes) {
  const auto rows=PackingCases();RecordProperty("native_rows",static_cast<int>(rows.size()));
  for(std::size_t i=0;i<rows.size();++i) {
    SCOPED_TRACE(i);c::PackedRow packed;
    ASSERT_EQ(c::PackLocal(rows[i],&packed),c::Status::Ok);const auto native=NativePack(rows[i]);
    ASSERT_EQ(Bits(packed.gap),Bits(native.gap));ASSERT_EQ(packed.symmetry,native.symmetry);
  }
}
TEST(NativeCandidatePacking,EveryFiveNodeConstraintCombination) {
  auto row=PackingCases().back();row.nodes[0]=1;row.nodes[1]=2;row.nodes[2]=3;row.nodes[3]=4;
  for(unsigned bits=0;bits<32768;++bits) {
    for(unsigned i=0;i<5;++i)row.constraint_codes[i]=(bits>>(3*i))&7;
    c::PackedRow packed;ASSERT_EQ(c::PackLocal(row,&packed),c::Status::Ok);
    ASSERT_EQ(packed.symmetry,NativePack(row).symmetry);
  }
}
TEST(NativeCandidatePacking,PairVelocityUsesL1AndExistingPreviousStep) {
  auto row=PackingCases().back();row.screen.secondary_gap=0;row.screen.main_gap=0;
  row.screen.curvature=0;row.screen.drad=0;row.screen.gap_load=0;
  for(auto& velocity:row.main_velocities)velocity={0,0,0};
  row.secondary_velocity={1,-2,3};row.previous_dt=.25;
  c::PackedRow packed;ASSERT_EQ(c::PackLocal(row,&packed),c::Status::Ok);
  EXPECT_EQ(packed.gap,1.01*1.5);
  row.screen.stored_motion=999.;ASSERT_EQ(c::PackLocal(row,&packed),c::Status::Ok);
  EXPECT_EQ(packed.gap,1.01*1.5);
  row.previous_dt=0.;ASSERT_EQ(c::PackLocal(row,&packed),c::Status::Ok);EXPECT_EQ(packed.gap,0.);
}
TEST(NativeCandidatePacking,InvalidAndOverflowAreAtomic) {
  auto row=PackingCases().front();c::PackedRow packed;packed.gap=99.;
  row.constraint_codes[0]=8;EXPECT_EQ(c::PackLocal(row,&packed),c::Status::InvalidInput);EXPECT_EQ(packed.gap,99.);
  row.constraint_codes[0]=0;row.previous_dt=std::numeric_limits<double>::max();
  EXPECT_EQ(c::PackLocal(row,&packed),c::Status::NonfiniteResult);EXPECT_EQ(packed.gap,99.);
}
