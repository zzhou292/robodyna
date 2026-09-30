// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Packet.h"
namespace type25_assembly_test {
TEST(Type25Assembly, CompleteNativeOutputsAcrossCohortsAndRepeatedEndpoints) {
  const auto c = Corpus();
  const auto expected = NativeAssemble(c.rows,c.responses,c.cohorts,c.incoming);
  const auto actual = Gather(c);
  ASSERT_EQ(actual.size(),expected.size());
  for (std::size_t n = 0; n < actual.size(); ++n) { SCOPED_TRACE(n); Same(actual[n],expected[n]); }
}
TEST(Type25Assembly, CohortsPreserveDifferentFloatingPointOrders) {
  Case c;
  c.rows = {{{0,1,1,1},0},{{0,1,1,1},1}};
  c.responses = {Response(1e20),Response(1.)};
  for (auto& r : c.responses) {
    r.normal.weights[0] = 1.; r.normal.weights[1] = 0.;
    r.normal.weights[2] = 0.; r.normal.weights[3] = 0.;
  }
  c.incoming.resize(2); c.cohorts = {2};
  const auto together = Gather(c);
  auto expected = NativeAssemble(c.rows,c.responses,c.cohorts,c.incoming);
  for (unsigned n = 0; n < 2; ++n) Same(together[n],expected[n]);
  c.cohorts = {1,2};
  const auto split = Gather(c);
  expected = NativeAssemble(c.rows,c.responses,c.cohorts,c.incoming);
  for (unsigned n = 0; n < 2; ++n) Same(split[n],expected[n]);
  EXPECT_EQ(together[0].force.x,0.); EXPECT_EQ(split[0].force.x,1.);
}
TEST(Type25Assembly, InactivePostResponseWeightsDoNotConsumeForceOrStiffnessScratch) {
  auto r = Response(); r.contact_active = false;
  for (double& weight : r.normal.weights) weight = -0.;
  r.normal.stability_stiffness = std::numeric_limits<double>::quiet_NaN();
  r.native_resultant = {std::numeric_limits<double>::infinity(),0,0};
  ass::NativeEndpoints output; output.active = true;
  EXPECT_EQ(ass::PrepareNativeEndpoints(Controls(),r,&output),ass::Status::Ok);
  EXPECT_FALSE(output.active);
}
TEST(Type25Assembly, FailedPacketPreparationAndUnitOverflowLeaveOutputUnchanged) {
  auto r = Response(); ass::NativeEndpoints output;
  output.secondary_stiffness = 199.;
  auto config = Controls(); config.parallel_assembly = 1;
  EXPECT_EQ(ass::PrepareNativeEndpoints(config,r,&output),ass::Status::UnsupportedProfile);
  EXPECT_EQ(output.secondary_stiffness,199.);
  r.normal.weights[0] = 2.; r.native_resultant.x = std::numeric_limits<double>::max();
  EXPECT_EQ(ass::PrepareNativeEndpoints(Controls(),r,&output),ass::Status::NonfiniteResult);
  EXPECT_EQ(output.secondary_stiffness,199.);
  r = Response(); r.contact_active = false;
  EXPECT_EQ(ass::PrepareNativeEndpoints(Controls(),r,&output),ass::Status::InvalidInput);
  r = Response(); ASSERT_EQ(ass::PrepareNativeEndpoints(Controls(),r,&output),ass::Status::Ok);
  ass::SiEndpoints si; si.secondary_stiffness = 119.;
  EXPECT_EQ(ass::EndpointsToSi({1.,2e307,1.},output,&si),ass::Status::NonfiniteResult);
  EXPECT_EQ(si.secondary_stiffness,119.);
}
TEST(Type25Assembly, NativeProductsAreConvertedOnceAtTheSiBoundary) {
  ass::NativeEndpoints native; ass::SiEndpoints si;
  ASSERT_EQ(ass::PrepareNativeEndpoints(Controls(),Response(),&native),ass::Status::Ok);
  ASSERT_EQ(ass::EndpointsToSi({.001,1000.,1.},native,&si),ass::Status::Ok);
  EXPECT_TRUE(si.active);
  EXPECT_EQ(si.secondary_resultant.x,native.secondary_resultant.x);
  EXPECT_EQ(si.secondary_resultant.y,native.secondary_resultant.y);
  EXPECT_EQ(si.secondary_resultant.z,native.secondary_resultant.z);
  EXPECT_EQ(si.secondary_stiffness,native.secondary_stiffness*1000.);
  for (unsigned slot = 0; slot < 4; ++slot) {
    EXPECT_EQ(si.main_force[slot].x,native.main_force[slot].x);
    EXPECT_EQ(si.main_force[slot].y,native.main_force[slot].y);
    EXPECT_EQ(si.main_force[slot].z,native.main_force[slot].z);
    EXPECT_EQ(si.main_stiffness[slot],native.main_stiffness[slot]*1000.);
  }
}
TEST(Type25Assembly, LateNodeFailureIsAtomicAndRetryUsesUntouchedIncoming) {
  const ass::Connectivity row{{0,0,0,0},0}; const std::uint32_t end = 1;
  const ass::Schedule schedule{&end,1,1};
  std::uint32_t offsets[2],ranks[5];
  ASSERT_TRUE(ass::BuildIncidence(&row,schedule,1,offsets,2,ranks,5));
  ass::NativeEndpoints packet;
  ASSERT_EQ(ass::PrepareNativeEndpoints(Controls(),Response(),&packet),ass::Status::Ok);
  const ass::NativeNodalValue initial{{12.,-0.,8.},7.};
  ass::NativeNodalValue output{{999.,888.,777.},666.}; const auto saved = output;
  packet.secondary_resultant.z = std::numeric_limits<double>::infinity();
  EXPECT_EQ(ass::GatherNode(0,&row,&packet,schedule,{offsets,ranks,1,5},initial,&output),
      ass::Status::InvalidInput); Same(output,saved);
  ASSERT_EQ(ass::PrepareNativeEndpoints(Controls(),Response(),&packet),ass::Status::Ok);
  ASSERT_EQ(ass::GatherNode(0,&row,&packet,schedule,{offsets,ranks,1,5},initial,&output),ass::Status::Ok);
  const auto native = NativeAssemble({row},{Response()},{1},{initial}); Same(output,native[0]);
  packet.main_force[0].x = std::numeric_limits<double>::max();
  packet.main_force[1].x = std::numeric_limits<double>::max(); output = saved;
  EXPECT_EQ(ass::GatherNode(0,&row,&packet,schedule,{offsets,ranks,1,5},initial,&output),
      ass::Status::NonfiniteResult); Same(output,saved);
}
} // namespace type25_assembly_test
