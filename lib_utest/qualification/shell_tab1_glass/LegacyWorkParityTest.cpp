#include "Tab1Fixture.h"
#include "LegacyFailureWork.h"
#include "lib_utest/qualification/shell_layered_failure/FailureSectionFixture.h"
#include <limits>
namespace tab1_test {
TEST(Tab1SharedWork, FrozenJohnsonOperationsKeepEveryFieldBitAndFailurePublication) {
  const auto p=layered_failure_test::Parameters(false);
  const sec::ConstantFailureParameters criterion{.01};
  for(unsigned mask=0;mask<8;++mask) {
    auto history=layered_failure_test::Seed(mask);
    layered_failure_test::WorkHistory work;
    for(unsigned step=0;step<12;++step) {
      auto in=layered_failure_test::Input(p);
      in.strain_curvature_increment[0]=.003;
      in.strain_curvature_increment[1]=-.001;
      in.strain_curvature_increment[2]=.0003;
      in.strain_curvature_increment[4]=-.00002;
      in.strain_curvature_increment[5]=.12;
      in.strain_curvature_increment[7]=-.03;
      sec::ShellLayeredJ2FailureResult section;
      ASSERT_EQ(sec::UpdateShellLayeredJ2Failure(p,criterion,history,in,(step+1)*in.dt,section),PointStatus::Ok);
      for(double viscosity:{0.,123.,std::numeric_limits<double>::max()}) {
        auto legacy=work,current=work;
        const bool a=sec::LegacyApplyLayeredJ2FailureWork(section,in.strain_curvature_increment,
            in.reference_thickness,.001,viscosity,legacy);
        const bool b=sec::ApplyLayeredJ2FailureWork(section,in.strain_curvature_increment,
            in.reference_thickness,.001,viscosity,current);
        EXPECT_EQ(a,b);
        EXPECT_EQ(Bytes(legacy),Bytes(current));
      }
      ASSERT_TRUE(sec::ApplyLayeredJ2FailureWork(section,in.strain_curvature_increment,
          in.reference_thickness,.001,123.,work));
      history=section.history;
    }
  }
}
} // namespace tab1_test
