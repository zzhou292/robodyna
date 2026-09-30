#pragma once
#include "WordArchive.h"
namespace crash::cases::vehicle_dynamics::diagnostics::qeph_rejection::codec {
template<class A, class D> void Diagnostics(A& a, D& d) {
    a(d.owner_id, d.configuration_id, d.qualification_id, d.epoch, d.base_epoch, d.attempt,
      d.time, d.base_time, d.velocity_time, d.base_velocity_time, d.kick_dt, d.phase,
      d.valid, d.has_completed_interval, d.accepted_force_assembled, d.kinetic_available, d.usage,
      d.kinetic_translation, d.kinetic_rotation, d.kinetic_physical_isotropic,
      d.kinetic_added_isotropic, d.internal_work, d.internal_work_increment,
      d.hourglass_viscous_work, d.hourglass_viscous_work_increment,
      d.minimum_area_ratio, d.minimum_thickness_ratio, d.maximum_displacement,
      d.maximum_absolute_strain, d.maximum_thickness_curvature, d.minimum_native_dt,
      d.internal_kick_work, d.internal_drift_work);
}
template<class A, class M> void Metadata(A& a, M& m) {
    auto& r = m.original;
    a(r.status, r.element, r.node, r.element_status, r.nodal_status);
    auto& o = m.owner;
    a(o.owner_id, o.epoch, o.node_count, o.time, o.fixed_dt, o.velocity_time,
      o.temporal_scheme, o.velocity_phase);
    Diagnostics(a, m.accepted); Diagnostics(a, m.candidate);
}
}
