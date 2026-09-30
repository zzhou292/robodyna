#include "CudaFixture.h"

namespace resident_tab1_test {
bool Prepare(Rig& rig, Prepared& output) {
  temporal::Loads loads;
  if (!rig.owner.accepted().epoch) {
    for (unsigned n = 0; n < Nodes; ++n) {
      const auto& node = rig.binding.nodes()[n];
      const auto v = failure_force_test::Velocity(node.position);
      const auto w = failure_force_test::Spin(node.position);
      const double velocity[]{v.x, v.y, v.z}, spin[]{w.x, w.y, w.z};
      for (unsigned a = 0; a < 3; ++a) {
        loads.force[3 * n + a] = node.native.mass * velocity[a] / (.5 * H);
        loads.couple[3 * n + a] = node.native.isotropic_inertia * spin[a] / (.5 * H);
      }
    }
  }
  Prepared next;
  fe::NodalAssemblyView view;
  if (!temporal::BeginLoad(rig.owner, loads, next.token, view)) return false;
  auto report = rig.owner.SealAssembly(next.token);
  EXPECT_EQ(report.status, fe::NodalStatus::Ok);
  if (report.status != fe::NodalStatus::Ok) return false;
  report = fe::AdvanceStaggeredHistory(rig.owner, next.token,
      {view.owner_id, view.accepted.base_epoch, view.attempt, H, 1., Qualification});
  EXPECT_EQ(report.status, fe::NodalStatus::Ok) << report.message;
  if (report.status != fe::NodalStatus::Ok) return false;
  report = rig.owner.CopyPrepared(next.token, next.endpoint.buffer(), &next.view);
  EXPECT_EQ(report.status, fe::NodalStatus::Ok) << report.message;
  if (report.status != fe::NodalStatus::Ok) return false;
  output = next;
  return true;
}
bool Evaluate(Rig& rig, const Prepared& prepared, Frame& output) {
  fe::ShellBatchDiagnostics candidate, joined;
  const auto qr = rig.qeph.EvaluateCandidate(prepared.view, &candidate.qeph);
  const auto tr = rig.t3.EvaluateCandidate(prepared.view, &candidate.t3);
  EXPECT_EQ(qr.status, q::BatchStatus::Success) << qr.message;
  EXPECT_EQ(tr.status, t::BatchStatus::Success) << tr.message;
  if (qr.status != q::BatchStatus::Success || tr.status != t::BatchStatus::Success) return false;
  const auto report = rig.publication.Prepare(rig.owner, prepared.token, candidate.qeph, candidate.t3, &joined);
  EXPECT_EQ(report.status, fe::ShellPublicationStatus::Success) << report.message;
  return report.status == fe::ShellPublicationStatus::Success && Read(rig, output, &joined);
}
bool Commit(Rig& rig, const Prepared& prepared, const Frame& frame) {
  const auto& d = frame.diagnostics.qeph;
  const auto report = rig.publication.Commit(rig.owner, prepared.token, frame.diagnostics,
      {d.owner_id, d.base_epoch, d.attempt, d.qualification_id, true});
  EXPECT_EQ(report.status, fe::ShellPublicationStatus::Success) << report.message;
  return report.status == fe::ShellPublicationStatus::Success;
}
q::PrescribedInterval Interval(const q::ReferenceData&, const std::array<std::size_t, 4>& nodes,
                              const Prepared& p) {
  q::PrescribedInterval out;
  out.base_time = p.view.base_time;
  out.dt = H;
  out.sample_index = p.view.kinematics.base_epoch + 1;
  for (unsigned i = 0; i < 4; ++i) {
    const auto n = nodes[i];
    out.position_endpoint[i] = {p.endpoint.x[3*n], p.endpoint.x[3*n+1], p.endpoint.x[3*n+2]};
    out.velocity_midpoint[i] = {p.endpoint.v[3*n], p.endpoint.v[3*n+1], p.endpoint.v[3*n+2]};
    out.omega_midpoint[i] = {p.endpoint.omega[3*n], p.endpoint.omega[3*n+1], p.endpoint.omega[3*n+2]};
  }
  return out;
}
t::PrescribedInterval Interval(const t::ReferenceData&, const std::array<std::size_t, 3>& nodes,
                              const Prepared& p) {
  t::PrescribedInterval out;
  out.base_time = p.view.base_time;
  out.dt = H;
  out.sample_index = p.view.kinematics.base_epoch + 1;
  for (unsigned i = 0; i < 3; ++i) {
    const auto n = nodes[i];
    out.position[i] = {p.endpoint.x[3*n], p.endpoint.x[3*n+1], p.endpoint.x[3*n+2]};
    out.velocity[i] = {p.endpoint.v[3*n], p.endpoint.v[3*n+1], p.endpoint.v[3*n+2]};
    out.angular_velocity[i] = {p.endpoint.omega[3*n], p.endpoint.omega[3*n+1], p.endpoint.omega[3*n+2]};
  }
  return out;
}
} // namespace resident_tab1_test
