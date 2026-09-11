#include "lib_src/materials/detail/SolidCallerValues.h"
#include <gtest/gtest.h>

namespace c = tl::material::solid_caller;
TEST(SolidCallerValues, DensityAndCompressionWorkUseActualStorageAndPressure) {
  const auto density = c::LagrangianDensity(4,4,2,1);
  EXPECT_EQ(density.density_kg_m3,8);
  EXPECT_EQ(density.volume_increment_m3,-1);
  const double old[]{-2,-2,-2,0,0,0}, now[]{-4,-4,-4,0,0,0};
  const double rate[]{-.1,-.1,-.1,0,0,0};
  const auto work = c::NoEosInternalWork(old,now,rate,.01,1.5,-1,1,2,7,2,1e-20);
  // Hydrostatic stresses have zero deviatoric power; pressure/Q uses actual dV.
  EXPECT_EQ(work.increment_j,4.5);
  EXPECT_EQ(work.energy_density_j_m3,9.25);
}
TEST(SolidCallerValues, NativeVolumeFloorIsAnExplicitNormalizationInput) {
  const double zero[6]{};
  const auto si = c::NoEosInternalWork(zero,zero,zero,1,1e-24,0,0,0,2,1e-24,1e-20);
  const auto mm = c::NoEosInternalWork(zero,zero,zero,1,1e-24,0,0,0,2,1e-24,1e-29);
  EXPECT_EQ(si.increment_j,0);
  EXPECT_EQ(mm.increment_j,0);
  EXPECT_EQ(si.energy_density_j_m3,(2*1e-24)/1e-20);
  EXPECT_EQ(mm.energy_density_j_m3,2);
}
