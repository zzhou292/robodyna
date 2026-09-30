#include "Tab1NativeSupport.h"
namespace tab1_test {
TEST(Tab1Native, AllAnyPointSelectionsRemovalWorkAndPostOffZeroCRecurrence) {
  const auto material=Material();
  const sec::ShellLayeredTab1Parameters failure{Table()};
  for(unsigned mask=0;mask<8;++mask) {
    SCOPED_TRACE(mask);
    auto h=Seed(mask);
    auto native=NativeSeed(h);
    Work work;
    unsigned removed=0,post=0;
    for(unsigned step=0;step<32;++step) {
      SCOPED_TRACE(step);
      const auto in=Increment(step,work.thickness);
      NativeTrace trace;
      NativeStep(in,(step+1)*in.dt,.001,.02,native,trace);
      sec::ShellLayeredTab1Result result;
      ASSERT_EQ(sec::UpdateShellLayeredTab1(material,failure,h,in,(step+1)*in.dt,result),PointStatus::Ok);
      ASSERT_TRUE(sec::ApplyLayeredTab1Work(result,in.strain_curvature_increment,
          in.reference_thickness,.001,trace.diagnostics[8],work));
      Compare(result,work,native,trace,in.reference_thickness,.001);
      for(unsigned p=0;p<3;++p)
        Close(result.caller_failure_increment[p],native.points[7*p+5]-h.saved.point[p].plastic_strain);
      if(result.removed_now)++removed;
      if(!h.element_active)++post;
      h=result.history;
    }
    EXPECT_EQ(removed,1u); EXPECT_GT(post,0u);
  }
}
} // namespace tab1_test
