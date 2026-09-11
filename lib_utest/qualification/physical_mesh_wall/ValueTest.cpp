// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/NodalWallContactPoint.h"
#include "lib_src/collision/NodalWallContactReduction.h"
#include "lib_src/collision/nodal_wall_mapped/Layout.h"
#include <gtest/gtest.h>
#include <cstring>
#include <limits>
namespace {
namespace c=tlfea::contact;
std::uint64_t Bits(double value) {
  std::uint64_t output;
  std::memcpy(&output,&value,sizeof(value));
  return output;
}
void Exact(const c::Q4CertifiedIntegral& a,const c::Q4CertifiedIntegral& b) {
  EXPECT_EQ(Bits(a.value),Bits(b.value)); EXPECT_EQ(Bits(a.lower),Bits(b.lower));
  EXPECT_EQ(Bits(a.upper),Bits(b.upper)); EXPECT_EQ(Bits(a.error),Bits(b.error));
}
TEST(PhysicalWallValues, PenaltyAndCertificatesMatchLegacyAcrossMassesAndSignedContact) {
  const c::NodalWallNodeWeight weight{0,{.25,.25,.25,0}};
  const c::NodalWallConfig config{0,1234,1,1,1};
  for(double x:{-.01,-0.0,0.0,.001,.02}) for(double velocity:{-3.,-0.,0.,2.}) {
    c::NodalWallPointResult physical;
    ASSERT_EQ(c::EvaluatePhysicalWallPoint(weight,{x,.3,.4},{velocity,1,2},7,config,11,&physical).status,
        c::NodalWallStatus::Ok);
    EXPECT_FALSE(physical.row.valid);
    EXPECT_EQ(physical.local_velocity_first_timestep,0);
    for(double inverse:{1e-12,1.,1e12}) {
      const std::uint8_t fixed=0;
      const c::LumpedTranslationMassView mass{&inverse,&fixed,1,7,c::TranslationMassModel::kIsotropicLumped};
      c::NodalWallPointResult legacy;
      ASSERT_EQ(c::EvaluateNodalWallPoint(weight,{x,.3,.4},{velocity,1,2},mass,config,11,&legacy).status,
          c::NodalWallStatus::Ok);
      Exact(physical.force,legacy.force); Exact(physical.potential,legacy.potential);
      Exact(physical.stiffness,legacy.stiffness);
      EXPECT_EQ(Bits(physical.force_world.x),Bits(legacy.force_world.x));
      EXPECT_EQ(Bits(physical.surface_power),Bits(legacy.surface_power));
      EXPECT_TRUE(legacy.row.valid);
    }
  }
}
TEST(PhysicalWallValues, IndependentClosedFormAndLateOverflowPreserveOutput) {
  const c::NodalWallNodeWeight weight{3,{.125,.125,.125,0}};
  c::NodalWallConfig config{.5,128,.25,1,1};
  c::NodalWallPointResult result;
  ASSERT_EQ(c::EvaluatePhysicalWallPoint(weight,{.625,2,3},{4,5,6},2,config,9,&result).status,c::NodalWallStatus::Ok);
  EXPECT_DOUBLE_EQ(result.force.value,2);
  EXPECT_DOUBLE_EQ(result.potential.value,.125);
  EXPECT_DOUBLE_EQ(result.surface_power,-8);
  EXPECT_DOUBLE_EQ(result.wall_moment.y,6);
  EXPECT_DOUBLE_EQ(result.wall_moment.z,-4);
  const auto before=result;
  config.stiffness_per_area=std::numeric_limits<double>::max();
  EXPECT_NE(c::EvaluatePhysicalWallPoint(weight,{.625,2,3},{1e300,0,0},2,config,9,&result).status,c::NodalWallStatus::Ok);
  EXPECT_EQ(Bits(result.surface_power),Bits(before.surface_power));
  Exact(result.force,before.force); Exact(result.potential,before.potential);
}
TEST(PhysicalWallValues, InactiveShareHasNoInventedMassRowAndKeepsLayerSumOrder) {
  c::NodalWallPointResult zero;
  zero.node=1; zero.base_epoch=3; zero.attempt=4; zero.valid=true;
  c::NodalWallPointResult active;
  ASSERT_EQ(c::EvaluatePhysicalWallPoint({1,{.25,.25,.25,0}},{.1,0,0},{},3,
      {0,100,1,1,1},4,&active).status,c::NodalWallStatus::Ok);
  auto sum=zero;
  ASSERT_TRUE(c::nodal_wall_reduction::AddPhysicalShare(sum,active));
  ASSERT_TRUE(c::nodal_wall_reduction::AddPhysicalShare(sum,zero));
  Exact(sum.force,active.force); Exact(sum.potential,active.potential);
  EXPECT_FALSE(sum.row.valid);
  auto legacy=zero;
  EXPECT_FALSE(c::nodal_wall_reduction::AddShare(legacy,active));
}
TEST(PhysicalWallValues, CompleteVehicleSidecarExactCapsAndLateRetry) {
  namespace m=c::nodal_wall_mapped;
  m::Layout layout;
  ASSERT_TRUE(m::MakeLayout(349645,359785,773,32u<<20,layout));
  EXPECT_EQ(layout.accepted.count,349645u);
  EXPECT_EQ(layout.roots.count,359785u);
  EXPECT_LT(layout.bytes,16u<<20);
  m::Layout same;
  ASSERT_TRUE(m::MakeLayout(349645,359785,773,layout.bytes,same));
  const auto old=same.bytes;
  EXPECT_FALSE(m::MakeLayout(349645,359785,773,layout.bytes-1,same));
  EXPECT_EQ(same.bytes,old);
  EXPECT_FALSE(m::MakeLayout(SIZE_MAX,359785,773,SIZE_MAX,same));
  EXPECT_EQ(same.bytes,old);
  ASSERT_TRUE(m::MakeLayout(349645,359785,773,layout.bytes,same));
  EXPECT_EQ(same.bytes,old);
}
} // namespace
