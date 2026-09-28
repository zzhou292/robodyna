#pragma once
#include "WordArchive.h"
#include "lib_src/elements/qeph/QephHistory.h"
namespace crash::cases::vehicle_dynamics::diagnostics::qeph_rejection::codec {
template<class A, class V> void Vector(A& a, V& v) { a(v.x, v.y, v.z); }
template<class A, class V> void Vectors(A& a, V& values) { for (auto& v : values) Vector(a, v); }
template<class A, class R> void Reference(A& a, R& r) {
    auto& i = r.input;
    Vectors(a, i.position);
    a(i.node_ids, i.density, i.young_modulus, i.poisson_ratio, i.thickness,
      i.placement, i.projection_working_length_m, r.frame.v, r.area, r.derivative_x, r.derivative_y);
    Vectors(a, r.local_position);
    a(r.nodal_mass, r.physical_inertia, r.added_inertia, r.isotropic_inertia, r.prepared);
}
template<class A, class H> void History(A& a, H& h, const native::ReferenceData& reference) {
    auto values = h.data();
    auto stamp = h.stamp();
    bool prepared = h.prepared();
    a(prepared, values.stress, values.material_stress, values.bending_stress,
      values.stabilization, values.strain_curvature, values.thickness, values.internal_work,
      values.hourglass_viscous_work, values.active, stamp.time, stamp.sample_index);
    if constexpr (A::Reading) {
        output::Require(prepared && native::PrepareFailurePrescribedHistory(reference, values, stamp, h) ==
            native::Status::kSuccess, "QEPH diagnostic accepted history cannot be reconstructed exactly");
    } else {
        output::Require(prepared && h.matches_reference(reference),
            "QEPH diagnostic accepted history has no matching reference");
    }
}
template<class A, class K> void Kinematics(A& a, K& k) {
    a(k.frame.v, k.area, k.reciprocal_area, k.characteristic_length, k.nodal_factors,
      k.raw_warpage_abs, k.effective_warpage, k.planar);
    Vectors(a, k.local_position); Vectors(a, k.local_normals);
    a(k.projection_inverse); Vectors(a, k.projection_columns);
    a(k.projected_omega, k.regular_rate, k.hourglass_rate, k.base_time, k.dt,
      k.sample_index, k.projection_metric.working_length_m);
}
template<class A, class F> void Force(A& a, F& f, const native::ReferenceData& reference) {
    History(a, f.proposed_history, reference); Kinematics(a, f.kinematics);
    Vectors(a, f.internal_force); Vectors(a, f.internal_couple);
    auto& d = f.diagnostics;
    a(d.effective_thickness, d.native_sound_speed, d.membrane_viscosity, d.stabilization_viscosity,
      d.translational_stiffness, d.rotational_stiffness, d.unscaled_element_dt,
      d.internal_work_increment, d.hourglass_viscous_work_increment);
}
template<class A, class I> void Interval(A& a, I& i) {
    Vectors(a, i.position_endpoint); Vectors(a, i.velocity_midpoint); Vectors(a, i.omega_midpoint);
    a(i.base_time, i.dt, i.sample_index);
}
}
