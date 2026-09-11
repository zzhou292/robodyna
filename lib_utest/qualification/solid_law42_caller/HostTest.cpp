// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/materials/law42/Caller.h"
#include "lib_src/elements/solid_common/PhysicalHourglassModes.h"
#include "lib_src/elements/solid_common/SolidCharacteristicLength.h"
#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <limits>
#include <vector>

namespace {
namespace law = tl::material::law42;
namespace solid = tl::fea::solid_common;

std::vector<double> Values(const law::CallerResult& r) {
  std::vector<double> values;
  for (double x : r.history.stress_pa) values.push_back(x);
  values.insert(values.end(), {r.history.density_kg_m3,
      r.history.internal_energy_density_j_m3, r.history.bulk_pressure_pa});
  for (double x : r.point.stress_pa) values.push_back(x);
  values.insert(values.end(), {r.point.maximum_principal_stress_pa,
      r.point.minimum_principal_stress_pa, r.point.active, r.point.relative_volume,
      r.point.sound_speed_m_s, r.point.hourglass_tangent_factor,
      r.point.material_viscosity_pa_s});
  for (double x : r.total_strain) values.push_back(x);
  values.insert(values.end(), {r.volume_increment_m3, r.average_volume_m3,
      r.internal_work_j, r.unscaled_element_dt_s, r.raw_stiffness_n_m});
  return values;
}
bool Same(const law::CallerResult& a, const law::CallerResult& b) {
  const auto x = Values(a), y = Values(b);
  return std::memcmp(x.data(), y.data(), x.size()*sizeof(double)) == 0;
}

TEST(Law42Caller, TotalStrainRetainsNonsymmetricGradientAndRejectsLateOverflow) {
  const double gradient[9]{.125, .25, -.5, 0, -.25, .125, .5, 0, .25};
  double strain[6];
  ASSERT_TRUE(law::TotalStrain(gradient, strain));
  // Independent matrix product B=(I+gradient)*(I+gradient)^T.
  double b[3][3]{};
  for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<3; ++j)
    for (unsigned k=0; k<3; ++k)
      b[i][j] += (gradient[3*i+k]+(i==k))*(gradient[3*j+k]+(j==k));
  const double expected[]{b[0][0]-1,b[1][1]-1,b[2][2]-1,
                          2*b[0][1],2*b[1][2],2*b[2][0]};
  for (unsigned k=0; k<6; ++k) EXPECT_DOUBLE_EQ(strain[k],expected[k]);
  std::array<double,6> before;
  std::copy(std::begin(strain),std::end(strain),before.begin());
  double bad[9]{};
  bad[8] = std::numeric_limits<double>::max();
  EXPECT_FALSE(law::TotalStrain(bad,strain));
  EXPECT_EQ(std::memcmp(strain,before.data(),sizeof(strain)),0);
}

TEST(Law42Caller, SharedEnergyCarriesAcrossCompressionAndStationaryInterval) {
  law::Parameters p;
  ASSERT_EQ(law::Prepare(24e6,.463,1980,1e26,p),law::Status::Ok);
  law::CallerHistory h;
  h.density_kg_m3=1980;
  h.internal_energy_density_j_m3=64; // Includes the caller's prior HG work.
  law::CallerInput i;
  i.dt_s=1e-6;
  i.storage_volume_m3=1e-6;
  i.current_volume_m3=i.storage_volume_m3;
  i.characteristic_length_m=.01;
  law::CallerResult rest;
  ASSERT_EQ(law::UpdateCaller(p,h,i,rest),law::Status::Ok);
  EXPECT_DOUBLE_EQ(rest.history.internal_energy_density_j_m3,64);
  EXPECT_DOUBLE_EQ(rest.internal_work_j,0);
  i.displacement_gradient[0]=-.01;
  i.current_volume_m3=.99*i.storage_volume_m3;
  i.engineering_rate_per_s[0]=-10000;
  law::CallerResult compressed;
  ASSERT_EQ(law::UpdateCaller(p,h,i,compressed),law::Status::Ok);
  EXPECT_GT(compressed.history.density_kg_m3,h.density_kg_m3);
  EXPECT_GT(compressed.history.bulk_pressure_pa,0);
  EXPECT_GT(compressed.internal_work_j,0);
  EXPECT_LT(compressed.unscaled_element_dt_s,rest.unscaled_element_dt_s);
  i.engineering_rate_per_s[0]=0;
  law::CallerResult stationary;
  ASSERT_EQ(law::UpdateCaller(p,compressed.history,i,stationary),law::Status::Ok);
  EXPECT_NEAR(stationary.internal_work_j,0,1e-15);
  EXPECT_NEAR(stationary.history.internal_energy_density_j_m3,
              compressed.history.internal_energy_density_j_m3,1e-9);
}

TEST(Law42Caller, CutoffBadParametersAndTailOverflowPreserveCompleteResult) {
  law::Parameters p;
  ASSERT_EQ(law::Prepare(24e6,.463,1980,1e26,p),law::Status::Ok);
  law::CallerHistory h;
  h.density_kg_m3=1980;
  law::CallerInput i;
  i.dt_s=1e-6;i.storage_volume_m3=1e-6;i.current_volume_m3=1.01e-6;
  i.characteristic_length_m=.01;i.displacement_gradient[0]=.01;
  law::CallerResult out;
  ASSERT_EQ(law::UpdateCaller(p,h,i,out),law::Status::Ok);
  const auto before=out;
  auto invalid=p;invalid.density_kg_m3=0;
  EXPECT_EQ(law::UpdateCaller(invalid,h,i,out),law::Status::InvalidParameters);
  EXPECT_TRUE(Same(out,before));
  invalid=p;invalid.tension_cutoff_pa=1;
  EXPECT_EQ(law::UpdateCaller(invalid,h,i,out),law::Status::InvalidInput);
  EXPECT_TRUE(Same(out,before));
  auto bad=i;bad.engineering_rate_per_s[5]=std::numeric_limits<double>::max();
  bad.displacement_gradient[6]=.01;
  EXPECT_EQ(law::UpdateCaller(p,h,bad,out),law::Status::NonfiniteResult);
  EXPECT_TRUE(Same(out,before));
  ASSERT_EQ(law::UpdateCaller(p,h,i,out),law::Status::Ok);
  EXPECT_TRUE(Same(out,before));
}

TEST(Law42Caller, SharedEightSlotLengthAndModesRetainTranslationAndBalance) {
  solid::Vec3 x[8]{{0,0,0},{2,0,0},{2,3,0},{0,3,0},
                   {0,0,4},{2,0,4},{2,3,4},{0,3,4}};
  double length=99;
  ASSERT_TRUE(solid::Law42CharacteristicLength(x,24,length));
  EXPECT_DOUBLE_EQ(length,2);
  x[7].x=std::numeric_limits<double>::max();
  EXPECT_FALSE(solid::Law42CharacteristicLength(x,24,length));
  EXPECT_DOUBLE_EQ(length,2);
  solid::Vec3 tiny[8]{};
  for (unsigned n=0;n<8;++n) tiny[n]={double(n&1)*1e-6,double((n>>1)&1)*1e-6,double(n>>2)*1e-6};
  ASSERT_TRUE(solid::Law42CharacteristicLength(tiny,1e-18,length));
  EXPECT_DOUBLE_EQ(length,4e-18/std::sqrt(1e-20));
  solid::Vec3 velocity[8];
  for (auto& v:velocity) v={1,-2,3};
  const double projection[4][4]{};
  double rates[3][4];
  solid::ModeRates(velocity,projection,rates);
  for (const auto& row:rates) for (double v:row) EXPECT_EQ(v,0);
  const double mode[3][4]{{1,2,3,4},{-1,-3,5,7},{9,2,-4,6}};
  solid::Vec3 force[8];
  solid::ModeForces(projection,mode,force);
  solid::Vec3 sum{};
  for (const auto& f:force) sum=solid::Add(sum,f);
  EXPECT_EQ(sum.x,0);EXPECT_EQ(sum.y,0);EXPECT_EQ(sum.z,0);
}
} // namespace
