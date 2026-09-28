#pragma once
#include "WordArchive.h"
namespace crash::cases::vehicle_dynamics::diagnostics::qeph_rejection::codec {
template<class A, class P> void PlasticParameters(A& a, P& p) {
    a(p.curve_count, p.young_pa, p.poisson_ratio, p.density_kg_m3, p.shear_modulus,
      p.a11, p.a12, p.three_g, p.sound_speed, p.rate.enabled, p.rate.cowper_symonds_c_per_s,
      p.rate.cowper_symonds_p, p.rate.cutoff_hz, p.rate.policy, p.inverse_rate_c,
      p.inverse_rate_p, p.angular_cutoff_per_s, p.hardening, p.linear.initial_yield_pa,
      p.linear.tangent_modulus_pa, p.plastic_hardening_pa, p.continuation);
}
template<class A, class P> void PlasticState(A& a, P& p) {
    for (auto& point : p.history.point)
        a(point.stress, point.plastic_strain, point.filtered_rate_per_s);
    auto& d = p.diagnostics;
    a(d.plastic_work_density_increment, d.maximum_plastic_strain, d.mean_plastic_strain,
      d.minimum_tangent_ratio, d.mean_tangent_ratio, d.mean_yield_before_pa,
      d.last_point_yield_before_pa, p.cumulative_plastic_work_J);
}
template<class A, class R> void Material(A& a, R& r) {
    a(r.global_law1.thickness, r.global_law1.coefficient_working_length_m);
    PlasticParameters(a, r.plastic_parameters);
    a(r.curve_strain, r.curve_stress_pa); PlasticState(a, r.accepted_plastic);
    auto& e = r.elastic_parameters;
    a(e.young_pa, e.poisson_ratio, e.density_kg_m3,
      e.elastic.g, e.elastic.a11, e.elastic.a12, e.elastic.sound_speed);
    for (auto& p : r.accepted_elastic.point) a(p.stress);
}
template<class A, class S> void FailureState(A& a, S& state) {
    auto policy = state.policy(); a(policy);
    if constexpr (A::Reading) {
        switch (policy) {
            case tl::fea::ShellFailurePolicy::None: state = {}; break;
            case tl::fea::ShellFailurePolicy::ConstantAllPoints: state = tl::fea::ShellBatchFailureState::Constant(); break;
            case tl::fea::ShellFailurePolicy::Tab1AnyPoint: state = tl::fea::ShellBatchFailureState::Tab1(); break;
            default: output::Require(false, "QEPH diagnostic failure tag is unsupported");
        }
    }
    if (const auto* points = state.constant_points()) {
        // Select the public mutable accessor only while decoding the active tag.
        for (unsigned i = 0; i < 3; ++i) {
            auto point = points[i]; a(point.damage, point.failure_time_s, point.point_active);
            if constexpr (A::Reading) state.constant_points()[i] = point;
        }
    } else if (const auto* points = state.tab1_points()) {
        for (unsigned i = 0; i < 3; ++i) {
            auto point = points[i];
            a(point.damage, point.maximum_damage, point.failure_time_s, point.table_segment, point.point_active);
            if constexpr (A::Reading) state.tab1_points()[i] = point;
        }
    } else output::Require(policy == tl::fea::ShellFailurePolicy::None,
        "QEPH diagnostic failure state has no supported tag");
    for (auto& p : state.current_force_point) a(p.stress);
    a(state.active);
}
template<class A, class R> void Failure(A& a, R& r) {
    a(r.failure_policy, r.constant_failure.failure_strain,
      r.tab1_failure.table.triaxiality, r.tab1_failure.table.failure_strain,
      r.tab1_failure.parent_policy);
    FailureState(a, r.accepted_failure);
}
}
