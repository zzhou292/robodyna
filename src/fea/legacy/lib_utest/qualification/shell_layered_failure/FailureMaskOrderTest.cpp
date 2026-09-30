#include "FailureSectionFixture.h"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <limits>

namespace layered_failure_test {
namespace {
sec::ShellLayeredJ2FailureResult RemovedValue(double force_x) {
  sec::ShellLayeredJ2FailureResult value;value.history=Seed(7);
  for(auto& point:value.history.current_force_point) point.stress[0]=force_x;
  value.current.reported_thickness=.002;
  // Caller-visible material resultants are already parent masked. The retained
  // current-force points deliberately remain separate and unmasked.
  value.current.material_stress[0]=force_x*0.;
  return value;
}
}
TEST(LayeredFailureValues, RemovedRawResultantPrecedesViscosityAndFinalMask) {
  const auto value=RemovedValue(100.);
  double dx[8]{};dx[0]=-1.;
  WorkHistory work;
  ASSERT_TRUE(sec::ApplyLayeredJ2FailureWork(value,dx,.002,.01,1.,work));
  // Native order: (100 + 1*(-1))*0 = +0. Mask-before-add yields -0.
  const double native=(100.+1.*(-1.))*0.;
  EXPECT_EQ(work.stress[0],0.);EXPECT_EQ(std::signbit(work.stress[0]),std::signbit(native));
  EXPECT_FALSE(std::signbit(work.stress[0]));
  EXPECT_FALSE(std::signbit(work.material_stress[0]));
  EXPECT_TRUE(std::signbit((100.*0.+1.*(-1.))*0.)); // Wrong-order sensitivity.
}
TEST(LayeredFailureValues, RemovedRawOverflowRejectsWholeWorkAndCleanRetryIsExact) {
  const double large=std::numeric_limits<double>::max()*.75;
  const auto value=RemovedValue(large);
  double dx[8]{};dx[0]=1.;
  WorkHistory work;work.stress[0]=120.;work.internal_work[0]=7.;work.strain_curvature[7]=.25;
  static_assert(sizeof(WorkHistory)==24*sizeof(double),"Compare only a padding-free work packet");
  const auto before=work;
  // Each operand is finite; the native raw FOR+viscosity sum overflows before
  // the final zero mask. Early material masking incorrectly conceals this.
  EXPECT_FALSE(sec::ApplyLayeredJ2FailureWork(value,dx,.002,.01,large,work));
  EXPECT_EQ(std::memcmp(&work,&before,sizeof(work)),0);
  auto clean=before;
  ASSERT_TRUE(sec::ApplyLayeredJ2FailureWork(value,dx,.002,.01,1.,clean));
  ASSERT_TRUE(sec::ApplyLayeredJ2FailureWork(value,dx,.002,.01,1.,work));
  EXPECT_EQ(std::memcmp(&work,&clean,sizeof(work)),0);
  EXPECT_EQ(work.material_stress[0],0.);EXPECT_EQ(work.internal_work[0],7.+120.*(.5*.002*.01));
}
} // namespace layered_failure_test
