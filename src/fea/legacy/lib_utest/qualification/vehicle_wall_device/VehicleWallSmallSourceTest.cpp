#include "VehicleWallFixture.h"
namespace vehicle_wall_device_test {
TEST(VehicleWallStartup, MixedNativeMassAndWeightFixtureExceedsOldCapsWithoutChangingAreaOrIncidence) {
  Fixture f(2049,73,4097);ASSERT_TRUE(f.Prepare());detail::PreparedModel model;
  auto c=f.Config();c.limits.profile=sc::NodalWallDeviceProfile::Legacy;
  EXPECT_EQ(detail::PrepareModel(c,f.wall.view(),f.weights,f.Positions(),f.inverse.data(),f.fixed.data(),f.motion,&model).status,Code::ResourceLimit);
  c=f.Config();ASSERT_EQ(detail::PrepareModel(c,f.wall.view(),f.weights,f.Positions(),f.inverse.data(),f.fixed.data(),f.motion,&model).status,Code::Ok);
  ASSERT_NO_FATAL_FAILURE(CheckIncidence(f,model));
  const long double total=f.nq/4.L+f.nt/8.L;
  EXPECT_LE(f.weights.total_area().lower,total);EXPECT_GE(f.weights.total_area().upper,total);
  for(std::size_t n=0;n<f.n;++n){EXPECT_GT(f.inverse[n],0);EXPECT_GT(f.inverse_j[n],0);EXPECT_TRUE(std::isfinite(f.inverse_j[n]));}
}
} // namespace vehicle_wall_device_test
