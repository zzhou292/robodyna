#pragma once
#include "FailureSectionFixture.h"
#include <array>

namespace layered_failure_test {
// Independent native trajectory; no production history/result layout at the ABI.
struct NativeState {
  std::array<double,21> points{};
  std::array<double,9> failures{0,0,1,0,0,1,0,0,1};
  std::array<double,5> material{},stress{};
  std::array<double,3> moment{};
  std::array<double,2> work{};
  double parent=1,thickness=.002;
};
struct NativeTrace {
  std::array<double,39> point_values{};
  std::array<double,9> diagnostics{};
  int removed=0;
};
NativeState NativeSeed(const sec::ShellLayeredJ2FailureHistory&);
void NativeStep(const sec::PointParameters&,double failure_strain,
    const sec::ShellLayeredJ2Input&,double time,double area,double dm,NativeState&,NativeTrace&);
void Compare(const sec::ShellLayeredJ2FailureResult&,const WorkHistory&,
    const NativeState&,const NativeTrace&,double reference_thickness,double area);
} // namespace layered_failure_test
