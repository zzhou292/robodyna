#include "FailureResidentSource.h"
#include "lib_src/math/Fixed3Operations.h"
#include "../shell_failure_force/FailureForceFields.h"
#include "lib_src/elements/qeph/QephBatchFailureSection.h"
#include "lib_src/elements/t3/T3BatchFailureSection.h"

namespace resident_failure_test {
namespace {
namespace q = fe::qeph;
namespace t = fe::t3;
constexpr double H = 1. / 1024;
tl::math::Vec3 Motion(tl::math::Vec3 x) {
  return {4 * (x.x + .25 * x.y), 2 * (x.y - .5 * x.x), 3 * (x.x - x.y)};
}
tl::math::Vec3 Spin(tl::math::Vec3 x) {
  return {8 * x.y, 12 * x.x, 4 * (x.x + x.y)};
}
q::PrescribedInterval Interval(const q::ReferenceData& reference, unsigned step) {
  q::PrescribedInterval out;
  out.dt = H;
  out.base_time = step * H;
  out.sample_index = step + 1;
  for (unsigned n = 0; n < 4; ++n) {
    const auto x = reference.input.position[n];
    out.position_endpoint[n] = tl::math::fixed3::Add(x, tl::math::fixed3::Scale(Motion(x), H * (step + 1) * (step + 1)));
    out.velocity_midpoint[n] = tl::math::fixed3::Scale(Motion(x), 2 * step + 1);
    out.omega_midpoint[n] = tl::math::fixed3::Scale(Spin(x), 2 * step + 1);
  }
  return out;
}
t::PrescribedInterval Interval(const t::ReferenceData& reference, unsigned step) {
  t::PrescribedInterval out;
  out.dt = H;
  out.base_time = step * H;
  out.sample_index = step + 1;
  for (unsigned n = 0; n < 3; ++n) {
    const auto x = reference.input.position[n];
    out.position[n] = tl::math::fixed3::Add(x, tl::math::fixed3::Scale(Motion(x), H * (step + 1) * (step + 1)));
    out.velocity[n] = tl::math::fixed3::Scale(Motion(x), 2 * step + 1);
    out.angular_velocity[n] = tl::math::fixed3::Scale(Spin(x), 2 * step + 1);
  }
  return out;
}
struct Quad {
  using History = q::History;
  using Trial = q::ForceTrial;
  static auto Reference(const fe::ShellBatchBinding& binding, unsigned parent) {
    return binding.qeph_reference(parent);
  }
  static constexpr auto Family = fe::ShellBindingFamily::Qeph;
  template<class... Args> static auto Evaluate(Args&&... args) {
    return q::batch_detail::EvaluateFailureSection(std::forward<Args>(args)...);
  }
};
struct Triangle {
  using History = t::History;
  using Trial = t::ForceTrial;
  static auto Reference(const fe::ShellBatchBinding& binding, unsigned parent) {
    return binding.t3_reference(parent);
  }
  static constexpr auto Family = fe::ShellBindingFamily::T3;
  template<class... Args> static auto Evaluate(Args&&... args) {
    return t::batch_detail::EvaluateFailureSection(std::forward<Args>(args)...);
  }
};

template<class Family> void Exercise() {
  Source source;
  fe::ShellBatchBinding binding;
  fe::ShellBatchPlasticityBinding catalog;
  ASSERT_TRUE(source.Prepare(binding, catalog));
  storage::MixedLayout mixed_layout;
  storage::FailureLayout failure_layout;
  ASSERT_TRUE(mixed_layout.Initialize(Parents, 0, 1 << 20));
  ASSERT_TRUE(failure_layout.Initialize(Parents, 1 << 20));
  tl::util::HostArena mixed_arena, failure_arena;
  ASSERT_TRUE(mixed_arena.Initialize(mixed_layout.bytes));
  ASSERT_TRUE(failure_arena.Initialize(failure_layout.bytes));
  auto* mixed = mixed_layout.Construct(mixed_arena);
  auto* failure = failure_layout.Construct(failure_arena);
  ASSERT_NE(mixed, nullptr);
  ASSERT_NE(failure, nullptr);
  std::array<typename Family::History, Parents> accepted;
  for (unsigned e = 0; e < Parents; ++e) {
    ASSERT_TRUE(catalog.Law(Family::Family, e, &mixed->law[e]));
    if (e == 0) {
      ASSERT_TRUE(catalog.ElasticParameters(Family::Family, e, &mixed->elastic_parameters[e]));
    } else {
      ASSERT_TRUE(catalog.Parameters(Family::Family, e, &mixed->plastic.parameters[e]));
      failure->policy[e] = fe::ShellFailurePolicy::ConstantAllPoints;
      failure->parameters[e].failure_strain = 1e-6;
      for (auto* slab : failure->state) slab[e] = fe::ShellBatchFailureState::Constant();
    }
    ASSERT_EQ(InitializeHistory(Family::Reference(binding, e), {}, accepted[e]),
              decltype(InitializeHistory(Family::Reference(binding, e), {}, accepted[e]))::kSuccess);
  }
  for (unsigned step = 0; step < 3; ++step) {
    const unsigned slab = step % 2;
    for (unsigned e = 0; e < Parents; ++e) {
      SCOPED_TRACE(step);
      SCOPED_TRACE(e);
      const auto reference = Family::Reference(binding, e);
      const auto interval = Interval(reference, step);
      typename Family::Trial next;
      const auto status = Family::Evaluate(reference, accepted[e], interval, *mixed, *failure, slab, e, next);
      ASSERT_EQ(status, decltype(status)::kSuccess);
      EXPECT_EQ(failure->state[1u - slab][e].active, e == 0);
      if (step && e == 1) {
        EXPECT_NE(next.proposed_history.data().strain_curvature[0], accepted[e].data().strain_curvature[0]);
      }
      const auto saved_force = failure_force_test::ForceValues(next);
      const auto saved_section = plasticity_binding_test::Bytes(mixed->plastic.section[1u - slab][e]);
      const auto saved_failure = plasticity_binding_test::Bytes(failure->state[1u - slab][e]);
      const auto policy = failure->policy[e];
      failure->policy[e] = fe::ShellFailurePolicy::Tab1AnyPoint;
      EXPECT_NE(Family::Evaluate(reference, accepted[e], interval, *mixed, *failure, slab, e, next),
                decltype(status)::kSuccess);
      failure_force_test::Exact(failure_force_test::ForceValues(next), saved_force);
      EXPECT_EQ(plasticity_binding_test::Bytes(mixed->plastic.section[1u - slab][e]), saved_section);
      EXPECT_EQ(plasticity_binding_test::Bytes(failure->state[1u - slab][e]), saved_failure);
      failure->policy[e] = policy;
      ASSERT_EQ(Family::Evaluate(reference, accepted[e], interval, *mixed, *failure, slab, e, next),
                decltype(status)::kSuccess);
      failure_force_test::Exact(failure_force_test::ForceValues(next), saved_force);
      accepted[e] = next.proposed_history;
    }
  }
}
} // namespace
TEST(ResidentFailureDispatch,CompleteQephPolicyAndThreeNonzeroIntervals) { Exercise<Quad>(); }
TEST(ResidentFailureDispatch,CompleteT3PolicyAndThreeNonzeroIntervals) { Exercise<Triangle>(); }
} // namespace resident_failure_test
