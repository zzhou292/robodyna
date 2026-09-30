// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Rig.cuh"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>
namespace native_diagnostic_test {
TEST_F(NativeDiagnosticCuda, NonzeroSeedCanonicalPermutationAndRepeatedUse) {
  Input in(257);in.units.energy=.125;Rig rig(in.flags.size());
  for(unsigned repeat=0;repeat<16;++repeat) {
    std::rotate(in.slots.begin(),in.slots.begin()+17,in.slots.end());
    for(std::size_t i=0;i<in.flags.size();++i)in.flags[i]=(i+repeat)%5!=0;
    rig.Compare(in);
  }
}
TEST_F(NativeDiagnosticCuda, CanonicalCancellationOrderIsNotReassociated) {
  Input in(3);in.seed.elastic_energy=0;in.response[0].normal.elastic_energy=1e16;
  in.response[1].normal.elastic_energy=1;in.response[2].normal.elastic_energy=-1e16;
  Rig rig(3);EXPECT_EQ(Bits(rig.Compare(in).elastic_energy),Bits(0));
  in.slots={0,2,1};EXPECT_EQ(Bits(rig.Compare(in).elastic_energy),Bits(1));
}
TEST_F(NativeDiagnosticCuda, EmptyCountPreservesSeedsAndStillConvertsUnits) {
  Input in(0);Rig rig(0);in.units.energy=2;
  auto result=rig.Compare(in,true);EXPECT_EQ(Bits(result.elastic_energy),Bits(5));
  EXPECT_EQ(Bits(result.damping_work),Bits(-6.25));EXPECT_EQ(result.active,17u);
}
TEST_F(NativeDiagnosticCuda, SignedZeroSeedsAndOperandsRemainBitExact) {
  for(std::size_t count:{0u,1u,5u}) {
    Input in(count);Rig rig(count);
    for(double seed:{0.,-0.})for(double operand:{0.,-0.}) {
      in.seed.elastic_energy=in.seed.damping_work=in.seed.friction_work=seed;
      for(auto& response:in.response) {
        response.normal.elastic_energy=response.normal.damping_work=response.friction_work=operand;
      }
      rig.Compare(in,count==0);
    }
  }
}
TEST_F(NativeDiagnosticCuda, InactivePoisonedRowsNeverReadSlotsOrResponses) {
  Input in(17);Rig rig(17);
  for(auto& response:in.response) {
    response.normal.elastic_energy=response.normal.damping_work=response.friction_work=std::nan("7");
  }
  std::fill(in.flags.begin(),in.flags.end(),0);std::fill(in.slots.begin(),in.slots.end(),UINT32_MAX);
  rig.Compare(in,true);
  // Positive occurrences read only their own slot; masked invalid slots stay unread.
  in.flags[4]=1;in.slots[4]=7;in.response[7]={};rig.Compare(in);
}
TEST_F(NativeDiagnosticCuda, NonfiniteSeedsAndTermsAndOverflowRetainTerminalFailure) {
  Input in(2);Rig rig(2);const Input original=in;
  const auto bad=static_cast<unsigned long long>(n::TransactionStatus::NumericalFailure);
  for(double value:{std::nan("11"),double(INFINITY),double(-INFINITY),DBL_MAX}) {
    in=original;in.response[0].normal.elastic_energy=value;
    in.response[1].normal.elastic_energy=value;EXPECT_EQ(rig.Compare(in).failure,bad);
    in=original;in.seed.damping_work=value;in.response[0].normal.damping_work=value;
    EXPECT_EQ(rig.Compare(in).failure,bad);
    in=original;in.response[0].friction_work=value;in.response[1].friction_work=value;
    EXPECT_EQ(rig.Compare(in).failure,bad);
  }
  in=original;in.seed.elastic_energy=DBL_MAX;in.response[0].normal.elastic_energy=0;
  in.response[1].normal.elastic_energy=0;in.units.energy=2;
  EXPECT_EQ(rig.Compare(in).failure,bad);
  // Restore operands on the same allocations; no retained partial-total cache.
  EXPECT_EQ(rig.Compare(original).failure,~0ull);
}
TEST_F(NativeDiagnosticCuda, ExistingFailureKeyIsMinimizedWithoutSkippingFold) {
  Input in(3);Rig rig(3);const auto bad=static_cast<unsigned long long>(n::TransactionStatus::NumericalFailure);
  for(auto prior:{0ull,3ull,91ull,(17ull<<16)|3ull,~0ull}) {
    in.seed.failure=prior;in.response[0].normal.elastic_energy=.5;
    auto normal=rig.Compare(in);EXPECT_EQ(normal.failure,prior);EXPECT_GT(normal.active,in.seed.active);
    in.response[0].normal.elastic_energy=INFINITY;
    EXPECT_EQ(rig.Compare(in).failure,std::min(prior,bad));
  }
}
TEST_F(NativeDiagnosticCuda, CounterWrapAndCompleteControlFreshRepair) {
  Input in(4);Rig rig(4);in.seed.active=std::numeric_limits<std::uint64_t>::max()-1;
  for(auto& response:in.response)response.contact_active=true;
  EXPECT_EQ(rig.Compare(in).active,2u);
  in.seed.failure=123;in.seed.friction_work=NAN;rig.Compare(in);
  in.seed=Seed();EXPECT_EQ(rig.Compare(in).failure,~0ull);
}
} // namespace native_diagnostic_test
