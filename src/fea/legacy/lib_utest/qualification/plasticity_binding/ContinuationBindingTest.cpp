#include "PlasticityBindingFixture.h"
#include "lib_src/elements/sections/ShellLayeredJ2Work.h"
#include <memory>

namespace plasticity_binding_test {
using Policy=tl::material::ShellPlasticityCurveContinuation;
TEST(PlasticityContinuationBinding, ExactScopeCopyMoveAndBorrowedCurvesRetainPolicy) {
  Fixture f;
  fe::ShellBatchBinding geometry;
  ASSERT_EQ(geometry.Initialize(f.collection()).status,fe::ShellBindingStatus::Success);
  Binding strict;
  ASSERT_EQ(strict.Initialize(geometry,f.catalog()).status,Status::Success);
  for(auto& m:f.materials) m.continuation=Policy::NativeLastSegment;
  auto extended=std::make_unique<Binding>();
  ASSERT_EQ(extended->Initialize(geometry,f.catalog()).status,Status::Success);
  EXPECT_FALSE(strict.SameScope(*extended));
  EXPECT_FALSE(extended->SameScope(strict));
  Binding copy(*extended),moved(std::move(copy));
  EXPECT_TRUE(extended->SameScope(copy));
  extended.reset();
  f.x[2]=.2; // Borrowed declarations no longer own the prepared curve.
  for(auto family:{fe::ShellBindingFamily::Qeph,fe::ShellBindingFamily::T3}) {
    fe::sections::PointParameters p;
    ASSERT_TRUE(moved.Parameters(family,0,&p));
    EXPECT_EQ(p.continuation,Policy::NativeLastSegment);
    EXPECT_EQ(p.curve.plastic_strain[2],.3);
    EXPECT_TRUE(fe::sections::ValidLayeredJ2Parameters(p));
  }
  EXPECT_TRUE(copy.SameScope(moved));
}
TEST(PlasticityContinuationBinding, LateUnknownAndNonTablePolicyRejectThenRetry) {
  for(unsigned invalid=0;invalid<3;++invalid) {
    Fixture f;
    fe::ShellBatchBinding geometry;
    ASSERT_EQ(geometry.Initialize(f.collection()).status,fe::ShellBindingStatus::Success);
    auto& m=f.materials.back();
    m.continuation=invalid==0?static_cast<Policy>(255):Policy::NativeLastSegment;
    if(invalid==1) {
      m.hardening=tl::material::ShellPlasticityHardeningKind::LinearLaw44;
      m.curve_id=0; m.linear={5400,1000};
    }
    if(invalid==2) {
      m.law=fe::ShellSectionLaw::LayeredLaw1Nip3;
      m.curve_id=0; m.rate={};
    }
    Binding candidate;
    const auto old=Bytes(candidate);
    const auto report=candidate.InitializeSections(geometry,f.catalog());
    EXPECT_EQ(report.status,Status::InvalidMaterial);
    EXPECT_EQ(report.entry,1u);
    EXPECT_EQ(Bytes(candidate),old);
    Fixture good;
    for(auto& material:good.materials) material.continuation=Policy::NativeLastSegment;
    EXPECT_EQ(candidate.InitializeSections(geometry,good.catalog()).status,Status::Success);
  }
}
} // namespace plasticity_binding_test
