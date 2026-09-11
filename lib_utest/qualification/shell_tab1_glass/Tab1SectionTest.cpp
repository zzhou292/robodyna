#include "Tab1Fixture.h"
#include <limits>
namespace tab1_test {
TEST(Tab1Section, EverySelectedPointMaskRemovesParentAndContinuesNativeOffHistory) {
  const auto p=Material();
  const sec::ShellLayeredTab1Parameters failure{Table()};
  for(unsigned mask=0;mask<8;++mask) {
    SCOPED_TRACE(mask);
    auto h=Seed(mask);
    Work work;
    unsigned removed=0,post=0;
    for(unsigned step=0;step<32;++step) {
      const auto in=Increment(step,work.thickness);
      sec::ShellLayeredTab1Result result;
      ASSERT_EQ(sec::UpdateShellLayeredTab1(p,failure,h,in,(step+1)*in.dt,result),PointStatus::Ok);
      ASSERT_TRUE(sec::ApplyLayeredTab1Work(result,in.strain_curvature_increment,
          in.reference_thickness,.001,123.,work));
      ASSERT_TRUE(sec::MatchesLayeredTab1Resultants(result.history,result.current));
      if(result.removed_now) {
        ++removed;
        EXPECT_FALSE(result.history.element_active);
        if(mask) {
          EXPECT_EQ(step,0u);
          for(unsigned k=0;k<3;++k)
            EXPECT_EQ(result.history.failure[k].point_active,(mask&(1u<<k))==0);
        }
      }
      if(!h.element_active) {
        ++post;
        EXPECT_DOUBLE_EQ(result.current.reported_thickness,in.reported_thickness);
        for(unsigned k=0;k<3;++k) {
          SameFailureHistory(result.history.failure[k],h.failure[k]);
          EXPECT_EQ(result.constitutive_increment[k],0.);
          EXPECT_DOUBLE_EQ(result.history.saved.point[k].plastic_strain,h.saved.point[k].plastic_strain);
          EXPECT_NE(result.history.saved.point[k].filtered_rate_per_s,h.saved.point[k].filtered_rate_per_s);
        }
      }
      h=result.history;
    }
    EXPECT_EQ(removed,1u);
    EXPECT_GT(post,0u);
  }
}
TEST(Tab1Section, LatePointFailureLeavesWholeSectionAndWorkUnchangedForRetry) {
  const auto p=Material();
  const sec::ShellLayeredTab1Parameters failure{Table()};
  auto h=Seed(0);
  const auto in=Increment(0);
  sec::ShellLayeredTab1Result result;
  ASSERT_EQ(sec::UpdateShellLayeredTab1(p,failure,h,in,in.dt,result),PointStatus::Ok);
  const auto before=Bytes(result);
  auto wrong_material=p;
  ASSERT_EQ(mat::PrepareLinearLaw44ShellPlasticity(70e9,.22,2500,{30e6,1e9},
      {true,8000,8,10000},wrong_material),PointStatus::Ok);
  EXPECT_EQ(sec::UpdateShellLayeredTab1(wrong_material,failure,h,in,in.dt,result),PointStatus::InvalidParameters);
  EXPECT_EQ(Bytes(result),before);
  h.saved.point[2].filtered_rate_per_s=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(sec::UpdateShellLayeredTab1(p,failure,h,in,in.dt,result),PointStatus::InvalidHistory);
  EXPECT_EQ(Bytes(result),before);
  h=Seed(0);
  ASSERT_EQ(sec::UpdateShellLayeredTab1(p,failure,h,in,in.dt,result),PointStatus::Ok);
  Work work;
  const auto old=Bytes(work);
  result.history.current_force_point[2].stress[4]=1.;
  EXPECT_FALSE(sec::ApplyLayeredTab1Work(result,in.strain_curvature_increment,.003,.001,0.,work));
  EXPECT_EQ(Bytes(work),old);
}
} // namespace tab1_test
