#pragma once
#include "ContinuationFailureFixture.h"
#include "lib_utest/qualification/shell_layered_failure/FailureNativeFixture.h"
#include <vector>

namespace continuation_test {
struct NativeFailureRecord {
  failure::NativeState state;
  failure::NativeTrace trace;
  double reference_thickness=0;
};
std::vector<NativeFailureRecord> NativeFailureTrajectory(OriginalCurve,bool,unsigned,bool);
void CompareFailureTrajectory(OriginalCurve,const std::vector<FailureRecord>&,
    const std::vector<NativeFailureRecord>&,bool initially_active);
} // namespace continuation_test
