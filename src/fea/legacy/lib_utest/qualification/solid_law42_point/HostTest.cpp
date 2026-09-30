// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include <cstring>
#include <limits>
namespace law42_test {
TEST(Law42Host,ZeroAndHydrostaticResponse) {
  auto p=Material();law::Result r;
  ASSERT_EQ(law::Update(p,Stretch(1,1,1),r),law::Status::Ok);
  for(double x:r.stress_pa)EXPECT_EQ(x,0);
  Near(r.sound_speed_m_s,std::sqrt((p.bulk_pa+4*p.mu_pa/3)/1980),1);
  for(double stretch:{.7,.99,1.03,1.5}) {
    ASSERT_EQ(law::Update(p,Stretch(stretch,stretch,stretch),r),law::Status::Ok);
    double pressure=p.bulk_pa*(stretch*stretch*stretch-1);
    for(unsigned k=0;k<3;++k)Near(r.stress_pa[k],pressure,p.bulk_pa);
    for(unsigned k=3;k<6;++k)Near(r.stress_pa[k],0,p.bulk_pa);
  }
}
TEST(Law42Host,IsochoricFiniteStrainAndFrameCovariance) {
  auto p=Material();auto x=Stretch(1.4,1/std::sqrt(1.4),1/std::sqrt(1.4));
  law::Result r,rotated;
  ASSERT_EQ(law::Update(p,x,r),law::Status::Ok);
  const double mean=(1.4*1.4+2/1.4)/3;
  Near(r.stress_pa[0],p.mu_pa*(1.4*1.4-mean),p.mu_pa);
  Near(r.stress_pa[1],p.mu_pa*(1/1.4-mean),p.mu_pa);
  ASSERT_EQ(law::Update(p,Rotate(x,.71),rotated),law::Status::Ok);
  const double c=std::cos(.71),s=std::sin(.71);
  Near(rotated.stress_pa[0],c*c*r.stress_pa[0]+s*s*r.stress_pa[1],p.mu_pa);
  Near(rotated.stress_pa[3],s*c*(r.stress_pa[0]-r.stress_pa[1]),p.mu_pa);
  Near(rotated.sound_speed_m_s,r.sound_speed_m_s,1);
  Near(rotated.hourglass_tangent_factor,r.hourglass_tangent_factor,1);
}
TEST(Law42Host,TensionRemovalAndCurrentDensity) {
  auto p=Material(1e6);law::Result r;
  ASSERT_EQ(law::Update(p,Stretch(1.2,1.2,1.2),r),law::Status::Ok);
  EXPECT_EQ(r.active,0);for(double x:r.stress_pa)EXPECT_EQ(x,0);
  auto x=Stretch(.9,.9,.9);x.active=0;
  ASSERT_EQ(law::Update(p,x,r),law::Status::Ok);
  EXPECT_EQ(r.active,0);for(double y:r.stress_pa)EXPECT_EQ(y,0);
  p=Material();x=Stretch(1.1,.9,1);law::Result other;
  ASSERT_EQ(law::Update(p,x,r),law::Status::Ok);
  x.density_kg_m3*=4;
  ASSERT_EQ(law::Update(p,x,other),law::Status::Ok);
  Near(other.sound_speed_m_s,r.sound_speed_m_s/2,1);
  for(unsigned k=0;k<6;++k)EXPECT_EQ(other.stress_pa[k],r.stress_pa[k]);
}
TEST(Law42Host,InvalidStatePreservesPriorResultAndRetries) {
  auto p=Material();auto x=Stretch(1.2,.8,1.1);law::Result result;
  ASSERT_EQ(law::Update(p,x,result),law::Status::Ok);const auto prior=result;
  x.total_strain[0]=-1;
  EXPECT_EQ(law::Update(p,x,result),law::Status::InvalidStretch);
  EXPECT_EQ(std::memcmp(&prior,&result,sizeof result),0);
  x=Stretch(1.2,.8,1.1);x.total_strain[5]=std::numeric_limits<double>::infinity();
  EXPECT_EQ(law::Update(p,x,result),law::Status::InvalidInput);
  EXPECT_EQ(std::memcmp(&prior,&result,sizeof result),0);
  p.bulk_pa*=1.001;
  EXPECT_EQ(law::Update(p,Stretch(1,1,1),result),law::Status::InvalidParameters);
  EXPECT_EQ(std::memcmp(&prior,&result,sizeof result),0);
  ASSERT_EQ(law::Update(Material(),Stretch(1,1,1),result),law::Status::Ok);
}
TEST(Law42Host,ReversibleLoadingAndNearlyIncompressiblePressureBranch) {
  auto p=Material();law::Result r;
  for(double a:{1.0,1.1,1.3,1.1,1.0})ASSERT_EQ(law::Update(p,Stretch(a,1,1),r),law::Status::Ok);
  for(double x:r.stress_pa)EXPECT_EQ(x,0);
  p=Material(1e26,.4999);
  ASSERT_EQ(law::Update(p,Stretch(.001,.8,.9),r),law::Status::Ok);
  EXPECT_GT(r.sound_speed_m_s,0);EXPECT_EQ(r.active,1);
}
} // namespace law42_test
