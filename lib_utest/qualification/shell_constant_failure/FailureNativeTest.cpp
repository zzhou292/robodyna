#include "FailureTestSupport.h"
#include <limits>

namespace failure_test {
TEST(ConstantPlasticFailureNative, OriginalCriterionAccumulatesAcrossLoadHoldAndFailure) {
  for (double threshold : {0.02, 0.25, 2.5, 3.5}) {
    f::ConstantPlasticFailureHistory base;
    const f::ConstantPlasticFailureParameters p{threshold};
    unsigned epoch = 0;
    for (double fraction : {0.125, 0., 0.25, 0., 0.5, 0.25, 0., 1.}) {
      const f::ConstantPlasticFailureInput in{fraction*threshold, ++epoch*0.001, true};
      f::ConstantPlasticFailureResult actual;
      ASSERT_TRUE(f::UpdateConstantPlasticFailure(p, base, in, actual));
      CompareNative(p, base, in, actual, {1.e8, -2.e7, 3.e6, 9.e5, -4.e5});
      base = actual.history;
    }
  }
}
TEST(ConstantPlasticFailureNative, InactiveFlagsAndExactThresholdFollowOriginalLeaf) {
  for (bool active : {false, true}) for (bool element : {false, true}) {
    const f::ConstantPlasticFailureHistory base{0.5, active ? 0. : 0.125, active};
    for (double increment : {0., 0.0625, 0.125, 0.25}) {
      const f::ConstantPlasticFailureParameters p{0.25};
      const f::ConstantPlasticFailureInput in{increment, 0.25, element};
      f::ConstantPlasticFailureResult actual;
      ASSERT_TRUE(f::UpdateConstantPlasticFailure(p, base, in, actual));
      CompareNative(p, base, in, actual, {});
    }
  }
}
TEST(ConstantPlasticFailureNative, PositiveDamageOverflowUsesNativePostFailureSaturation) {
  const f::ConstantPlasticFailureParameters p{std::numeric_limits<double>::min()};
  const f::ConstantPlasticFailureInput in{std::numeric_limits<double>::max(), 0.25, true};
  f::ConstantPlasticFailureResult actual;
  ASSERT_TRUE(f::UpdateConstantPlasticFailure(p, {}, in, actual));
  CompareNative(p, {}, in, actual, {});
}
} // namespace failure_test
