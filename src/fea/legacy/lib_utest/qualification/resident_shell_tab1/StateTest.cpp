#include "HostFamily.h"
#include "lib_src/elements/failure/ShellFailureStorage.h"
#include <limits>
namespace resident_tab1_test {
TEST(ResidentTab1State,TypedPayloadCopySwitchesLifetimesAndRetainsUncappedDamage) {
  RecordProperty("failure_state_bytes", std::to_string(sizeof(fe::ShellBatchFailureState)));
  RecordProperty("tab1_parameters_bytes", std::to_string(sizeof(fe::sections::ShellLayeredTab1Parameters)));
  fe::ShellBatchFailureState none;
  EXPECT_EQ(none.constant_points(), nullptr);
  EXPECT_EQ(none.tab1_points(), nullptr);
  auto constant = fe::ShellBatchFailureState::Constant();
  auto glass = fe::ShellBatchFailureState::Tab1();
  glass.active = false;
  glass.tab1_points()[2] = {1.75, 1., .25, 2, false};
  fe::ShellBatchSectionState section;
  ASSERT_TRUE(storage::ValidFailureState(glass, fe::ShellFailurePolicy::Tab1AnyPoint, &section, .25));
  for (unsigned i = 0; i < 10; ++i) {
    constant = glass;
    ASSERT_NE(constant.tab1_points(), nullptr);
    EXPECT_EQ(constant.constant_points(), nullptr);
    failure_force_test::Exact(Values(constant), Values(glass));
    constant = fe::ShellBatchFailureState::Constant();
    ASSERT_NE(constant.constant_points(), nullptr);
    EXPECT_EQ(constant.tab1_points(), nullptr);
    EXPECT_EQ(constant.constant_points()[2].damage, 0);
  }
  const auto original = glass;
  for (unsigned fault = 0; fault < 5; ++fault) {
    glass = original;
    auto& point = glass.tab1_points()[2];
    switch (fault) {
      case 0: point.damage = std::numeric_limits<double>::quiet_NaN(); break;
      case 1: point.maximum_damage = .75; break;
      case 2: point.table_segment = 3; break;
      case 3: point.failure_time_s = .5; break;
      case 4: glass.active = true; break;
    }
    EXPECT_FALSE(storage::ValidFailureState(glass, fe::ShellFailurePolicy::Tab1AnyPoint, &section, .25));
  }
  glass = original;
  *reinterpret_cast<unsigned char*>(&glass.tab1_points()[2].point_active) = 2;
  EXPECT_FALSE(storage::ValidFailureEncoding(glass));
  EXPECT_FALSE(storage::ValidFailureState(original, fe::ShellFailurePolicy::ConstantAllPoints, &section, .25));
}
template<class F> void Dispatch() {
  for (auto plane : placed::Planes) for (unsigned mask : {0u, 1u, 7u}) {
    HostFamily<F> host(plane, mask);
    unsigned removed = 0, inactive = 0;
    for (unsigned step = 0; step < 32 && inactive < 2; ++step) {
      const auto& base = host.failure->state[host.slab][Parents - 1];
      const bool was_active = base.active;
      for (unsigned e = 0; e < Parents; ++e) ASSERT_EQ(host.Evaluate(e, step), F::Status::kSuccess);
      const auto& next = host.failure->state[1 - host.slab][Parents - 1];
      removed += was_active && !next.active;
      inactive += !was_active;
      if (!step && mask == 1) {
        EXPECT_FALSE(next.active);
        EXPECT_FALSE(next.tab1_points()[0].point_active);
        EXPECT_TRUE(next.tab1_points()[1].point_active);
        EXPECT_TRUE(next.tab1_points()[2].point_active);
      }
      if (!step && mask == 7) {
        for (unsigned p = 0; p < 3; ++p) EXPECT_FALSE(next.tab1_points()[p].point_active);
      }
      EXPECT_TRUE(host.failure->state[1 - host.slab][0].active);
      EXPECT_TRUE(host.failure->state[1 - host.slab][1].active);
      if (step == 1) {
        const auto force = placed::ForceValues(host.candidate.back());
        const auto saved = Values(host.mixed->plastic.section[1 - host.slab][Parents - 1]);
        const auto sidecar = Values(next);
        auto& old_point = host.mixed->plastic.section[host.slab][Parents - 1].history.point[2];
        const auto old = old_point;
        old_point.plastic_strain = -1.;
        EXPECT_NE(host.Evaluate(Parents - 1, step), F::Status::kSuccess);
        placed::Exact(placed::ForceValues(host.candidate.back()), force);
        placed::Exact(Values(host.mixed->plastic.section[1 - host.slab][Parents - 1]), saved);
        placed::Exact(Values(next), sidecar);
        old_point = old;
        ASSERT_EQ(host.Evaluate(Parents - 1, step), F::Status::kSuccess);
        placed::Exact(placed::ForceValues(host.candidate.back()), force);
        placed::Exact(Values(next), sidecar);
      }
      host.Accept();
    }
    EXPECT_EQ(removed, 1u);
    EXPECT_EQ(inactive, 2u);
  }
}
TEST(ResidentTab1Dispatch,QephCompletePoliciesPlanesRemovalAndExactLateRetry) { Dispatch<placed::Q>(); }
TEST(ResidentTab1Dispatch,T3CompletePoliciesPlanesRemovalAndExactLateRetry) { Dispatch<placed::T>(); }
} // namespace resident_tab1_test
