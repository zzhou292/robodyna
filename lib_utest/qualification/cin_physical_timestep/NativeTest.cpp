// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <gtest/gtest.h>
extern "C" {
void cin_native_nodal(int,const double*,const double*,const double*,const double*,const double*,double*);
void cin_native_rigid_terms(int,const double*,const double*,const double*,const double*,double*);
}
namespace cin_step_test {
TEST(CinPhysicalStepNative, BothOrdinaryChannelsMatchCompleteSelectedNativeLoops) {
  constexpr int n=5;
  const double mass[]{.001,.1,2,100,1e4},inertia[]{1e-9,1e-7,.1,.3,100};
  const double kn[]{1e8,2,4e5,100,1e-2},kr[]{.001,.5,1e2,1e3,1e5};
  for(double factor: {.125,.8,1.}) {
    double native[2*n];
    cin_native_nodal(n,mass,inertia,kn,kr,&factor,native);
    for(int i=0;i<n;++i) {
      dt::ScalarLimit value;
      ASSERT_TRUE(dt::OrdinaryLimit(mass[i],kn[i],factor,value));
      EXPECT_DOUBLE_EQ(value.dt,native[i]);
      ASSERT_TRUE(dt::OrdinaryLimit(inertia[i],kr[i],factor,value));
      EXPECT_DOUBLE_EQ(value.dt,native[n+i]);
    }
  }
}
TEST(CinPhysicalStepNative, PlainNativeLeverAndRotationalProducerIsDistinctFromBodyTrace) {
  const double positions[]{-1,2,3, 4,-5,6, 0,.5,-.25};
  const double center[]{.25,-.1,.8};
  const double kn[]{3,5,7},kr[]{.1,.3,.8};
  double native[6];
  cin_native_rigid_terms(3,positions,center,kn,kr,native);
  double translation=0,rotation=0;
  for(unsigned i=0;i<3;++i) {
    const double dx=positions[3*i]-center[0],dy=positions[3*i+1]-center[1],dz=positions[3*i+2]-center[2];
    EXPECT_DOUBLE_EQ(native[i],kn[i]);
    EXPECT_DOUBLE_EQ(native[3+i],kr[i]+(dx*dx+dy*dy+dz*dz)*kn[i]);
    translation+=native[i]; rotation+=native[3+i];
  }
  EXPECT_EQ(translation,15);
  EXPECT_NEAR(rotation,369.875,1e-12);
  EXPECT_NE(rotation,kr[0]+kr[1]+kr[2]);
}
} // namespace cin_step_test
