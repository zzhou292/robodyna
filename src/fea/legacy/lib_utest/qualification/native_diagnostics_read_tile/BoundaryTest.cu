// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Rig.cuh"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>
namespace native_diagnostic_test {
TEST_F(NativeDiagnosticCuda, TileBoundariesMixedMasksAndNonbooleanPositiveFlags) {
  for(std::size_t count:{31u,32u,33u,63u,64u,65u,127u,128u,129u,191u,192u,193u}) {
    SCOPED_TRACE(count);
    Input in(count);Rig rig(count);in.units.energy=.125;
    std::reverse(in.slots.begin(),in.slots.end());
    for(std::size_t i=0;i<count;++i)in.flags[i]=i%4==0?0:i%2?2:UINT32_MAX;
    rig.Compare(in);
    in.seed.active=std::numeric_limits<std::uint64_t>::max()-3;
    for(auto& response:in.response)response.contact_active=true;
    rig.Compare(in);
  }
}
TEST_F(NativeDiagnosticCuda, CancellationAcrossTilesKeepsEachCanonicalAddition) {
  for(std::size_t first:{63u,127u}) {
    SCOPED_TRACE(first);
    Input in(first+3);Rig rig(in.flags.size());
    std::fill(in.flags.begin(),in.flags.end(),0);
    in.seed.elastic_energy=in.seed.damping_work=in.seed.friction_work=0;
    const double terms[]{1e16,1,-1e16};
    for(unsigned i=0;i<3;++i) {
      in.flags[first+i]=1;
      auto& response=in.response[first+i];response.normal.elastic_energy=terms[i];
      response.normal.damping_work=terms[i];response.friction_work=terms[i];
    }
    const auto ordered=rig.Compare(in);
    EXPECT_EQ(Bits(ordered.elastic_energy),Bits(0.));
    EXPECT_EQ(Bits(ordered.damping_work),Bits(0.));EXPECT_EQ(Bits(ordered.friction_work),Bits(0.));
    std::swap(in.slots[first+1],in.slots[first+2]);
    const auto reordered=rig.Compare(in);
    EXPECT_EQ(Bits(reordered.elastic_energy),Bits(1.));
    EXPECT_EQ(Bits(reordered.damping_work),Bits(1.));EXPECT_EQ(Bits(reordered.friction_work),Bits(1.));
  }
}
TEST_F(NativeDiagnosticCuda, PoisonedInactiveTilesAndPartialTailNeverReadOperands) {
  for(std::size_t count:{63u,64u,65u,127u,128u,129u}) {
    SCOPED_TRACE(count);
    Input in(count);Rig rig(count);
    std::fill(in.flags.begin(),in.flags.end(),0);std::fill(in.slots.begin(),in.slots.end(),UINT32_MAX);
    for(auto& response:in.response)
      response.normal.elastic_energy=response.normal.damping_work=response.friction_work=std::nan("23");
    in.seed.elastic_energy=in.seed.damping_work=in.seed.friction_work=-0.;
    const auto inactive=rig.Compare(in,true);EXPECT_EQ(Bits(inactive.elastic_energy),Bits(-0.));
    in.flags.back()=UINT32_MAX;in.slots.back()=0;in.response[0]={};
    rig.Compare(in);
  }
}
TEST_F(NativeDiagnosticCuda, CrossTileNonfinitePriorFailureAndSameDeviceRepair) {
  Input original(129);Rig rig(129);
  const auto failure=static_cast<unsigned long long>(n::TransactionStatus::NumericalFailure);
  for(auto prior:{0ull,3ull,(17ull<<16)|3ull,~0ull}) {
    Input in=original;in.seed.failure=prior;
    in.response[63].normal.elastic_energy=DBL_MAX;in.response[64].normal.elastic_energy=DBL_MAX;
    in.response[128].normal.elastic_energy=-DBL_MAX;
    EXPECT_EQ(rig.Compare(in).failure,std::min(prior,failure));
    in=original;in.seed.failure=prior;in.response[63].normal.damping_work=std::nan("31");
    in.response[128].friction_work=std::nan("37");rig.Compare(in);
    // Reuse these exact two device arenas and stream after failed complete Control output.
    EXPECT_EQ(rig.Compare(original).failure,~0ull);
  }
  for(double units:{-0.,-2.,double(INFINITY),std::nan("41")}) {
    auto in=original;in.units.energy=units;rig.Compare(in);
    EXPECT_EQ(rig.Compare(original).failure,~0ull);
  }
}
} // namespace native_diagnostic_test

// One CUDA translation unit keeps the literal private oracle kernels unique.
#include "ExistingCudaTest.cu"
