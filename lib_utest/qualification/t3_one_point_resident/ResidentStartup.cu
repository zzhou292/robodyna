#include "ResidentFixture.h"

namespace t3_one_point_resident_test {
bool Initialize(Rig& rig, fe::ShellBatchPlasticityBinding& catalog,
    fe::ShellBatchFailureBinding& failure, bool one_point, double failure_strain) {
  Source source(one_point, failure_strain);
  if (!source.Prepare(rig.binding, catalog, failure)) return false;
  rig.initial.n = Nodes;
  rig.initial.h = mixed::H;
  for (unsigned n = 0; n < Nodes; ++n) {
    const auto& value = rig.binding.nodes()[n];
    rig.initial.x[3*n] = value.position.x;
    rig.initial.x[3*n+1] = value.position.y;
    rig.initial.x[3*n+2] = value.position.z;
    rig.initial.inverse[n] = 1 / value.native.mass;
    rig.initial.inverse_inertia[n] = 1 / value.native.isotropic_inertia;
  }
  const auto state = rig.initial.Initialize(rig.owner);
  EXPECT_EQ(state.status, fe::NodalStatus::Ok);
  if (state.status != fe::NodalStatus::Ok) return false;
  fe::qeph::QephBatchConfig qc;
  qc.owner = rig.owner.accepted();
  qc.element_count = Parents;
  qc.configuration_id = mixed::Configuration;
  qc.qualification_id = mixed::Qualification;
  qc.usage = fe::qeph::BatchUsage::PrescribedFields;
  t3::T3BatchConfig tc;
  tc.owner = qc.owner;
  tc.element_count = Parents;
  tc.configuration_id = qc.configuration_id;
  tc.qualification_id = qc.qualification_id;
  tc.usage = t3::BatchUsage::PrescribedFields;
  const auto q = rig.qeph.InitializeJoined(qc, rig.binding, catalog, failure, {});
  const auto t = rig.t3.InitializeJoined(tc, rig.binding, catalog, failure, {});
  EXPECT_EQ(q.status, fe::qeph::BatchStatus::Success) << q.message;
  EXPECT_EQ(t.status, t3::BatchStatus::Success) << t.message;
  return q.status == fe::qeph::BatchStatus::Success && t.status == t3::BatchStatus::Success;
}
bool Prepare(Rig& rig, unsigned step, Prepared& prepared) {
  temporal::Snapshot before;
  if (!temporal::Read(rig.owner, before)) return false;
  temporal::Loads loads;
  constexpr double pi = 3.14159265358979323846;
  const double rate = .20 * (2*pi/(32*mixed::H)) * std::cos(2*pi*(step+.5)/32);
  const double kick = rig.owner.accepted().epoch ? mixed::H : .5*mixed::H;
  const auto center = rig.binding.nodes()[0].position;
  for (unsigned n = 0; n < Nodes; ++n) {
    const auto delta = pure::Sub(rig.binding.nodes()[n].position, center);
    const double v[]{rate*delta.x, rate*delta.y, rate*delta.z};
    const double omega[]{.15*n*std::sin(2*pi*(step+.5)/32), -.11*n, .07*n};
    for (unsigned axis = 0; axis < 3; ++axis) {
      loads.force[3*n+axis] = rig.binding.nodes()[n].native.mass * (v[axis]-before.v[3*n+axis]) / kick;
      loads.couple[3*n+axis] = rig.binding.nodes()[n].native.isotropic_inertia *
          (omega[axis]-before.omega[3*n+axis]) / kick;
    }
  }
  return mixed::Prepare(rig, loads, prepared);
}
t3::PrescribedInterval Interval(const Rig& rig, const Prepared& prepared) {
  t3::PrescribedInterval interval;
  interval.base_time = prepared.view.base_time;
  interval.dt = mixed::H;
  interval.sample_index = prepared.view.kinematics.base_epoch + 1;
  for (unsigned local = 0; local < 3; ++local) {
    const auto n = rig.binding.t3_nodes(1)[local];
    const auto& p = prepared.endpoint;
    interval.position[local] = {p.x[3*n], p.x[3*n+1], p.x[3*n+2]};
    interval.velocity[local] = {p.v[3*n], p.v[3*n+1], p.v[3*n+2]};
    interval.angular_velocity[local] = {p.omega[3*n], p.omega[3*n+1], p.omega[3*n+2]};
  }
  return interval;
}
} // namespace t3_one_point_resident_test
