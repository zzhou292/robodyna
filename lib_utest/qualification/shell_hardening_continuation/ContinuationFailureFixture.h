#pragma once
#include "ContinuationFixture.h"
#include "lib_utest/qualification/shell_layered_failure/FailureSectionFixture.h"

namespace continuation_test {
namespace failure=layered_failure_test;
inline constexpr unsigned MaximumFailureSteps=180;
inline sec::ShellLayeredJ2Input FailureInput(const Parameters& p) {
  auto in=failure::Input(p);
  in.strain_curvature_increment[0]=.02;
  in.strain_curvature_increment[1]=-.006;
  in.strain_curvature_increment[2]=.0007;
  in.strain_curvature_increment[3]=.0003;
  in.strain_curvature_increment[4]=-.0002;
  in.strain_curvature_increment[5]=8.;
  in.strain_curvature_increment[6]=-2.;
  return in;
}
inline sec::ShellLayeredJ2FailureHistory FailureSeed(OriginalCurve c,unsigned mask,bool virgin) {
  auto h=failure::Seed(mask);
  if(!virgin) for(auto& point:h.saved.point) point.plastic_strain=c.x[c.count-1]+.01;
  return h;
}
struct FailureRecord {
  sec::ShellLayeredJ2FailureResult section;
  failure::WorkHistory work;
  double reference_thickness=0;
  Status status=Status::InvalidParameters;
  bool work_valid=false;
};
} // namespace continuation_test
