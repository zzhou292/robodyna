#include "HostFamily.h"
#include "NativeAgreement.h"

namespace resident_tab1_test {
template<class F> void NativeDispatch() {
  for (auto plane : placed::Planes) for (unsigned mask = 0; mask < 8; ++mask) {
    SCOPED_TRACE(static_cast<unsigned>(plane));
    SCOPED_TRACE(mask);
    HostFamily<F> host(plane, mask);
    constexpr unsigned parent = Parents - 1;
    const auto reference = host.Reference(parent);
    const auto material = host.mixed->plastic.parameters[parent];
    const auto failure = host.failure->tab1_parameters[parent];
    const typename F::History initial{host.accepted[parent], storage::Tab1FailureHistory(
        host.mixed->plastic.section[host.slab][parent], host.failure->state[host.slab][parent])};
    auto oracle = native::Seed(initial);
    unsigned removed = 0, inactive = 0;
    for (unsigned step = 0; step < 32 && inactive < 2; ++step) {
      SCOPED_TRACE(step);
      const auto old_section = host.mixed->plastic.section[host.slab][parent];
      const bool active = host.failure->state[host.slab][parent].active;
      const double thickness = host.accepted[parent].data().thickness;
      for (unsigned e = 0; e < Parents; ++e) ASSERT_EQ(host.Evaluate(e, step), F::Status::kSuccess);
      const auto interval = placed::Interval(reference, step);
      native::Advance(reference, interval, material, failure, oracle);
      const auto& result = host.failure->state[1 - host.slab][parent];
      NativeAgreement<F>(host.candidate[parent], host.mixed->plastic.section[1 - host.slab][parent],
          result, oracle, interval, thickness, old_section.cumulative_plastic_work_J);
      removed += active && !result.active;
      inactive += !active;
      if (!step && mask == 1) {
        EXPECT_FALSE(result.active);
        EXPECT_FALSE(result.tab1_points()[0].point_active);
        EXPECT_TRUE(result.tab1_points()[1].point_active);
        EXPECT_TRUE(result.tab1_points()[2].point_active);
      }
      if (!step && mask == 7) {
        for (unsigned p = 0; p < 3; ++p) EXPECT_FALSE(result.tab1_points()[p].point_active);
      }
      host.Accept();
    }
    EXPECT_EQ(removed, 1u);
    EXPECT_EQ(inactive, 2u);
  }
}
TEST(ResidentTab1Native,QephCompleteCollectionNativeFirstPointAllPointsAndInactive) { NativeDispatch<placed::Q>(); }
TEST(ResidentTab1Native,T3CompleteCollectionNativeFirstPointAllPointsAndInactive) { NativeDispatch<placed::T>(); }
} // namespace resident_tab1_test
