// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CudaFixture.h"
namespace beam18_resident_test {
bool Rig::Initialize(bool attach) {
  auto& f = fixture.mechanics;
  auto owner_config = f.Config();
  owner_config.fixed_dt = fixture.Configuration().owner.fixed_dt;
  const auto cin = f.Cin();
  if (!Good(owner.Initialize(owner_config, f.Kinematics(), f.im.data(), f.Dofs(), fixture.binding, &cin))) return false;
  config = fixture.Configuration();
  config.owner = owner.accepted();
  if (!Good(batch.InitializeJoined(config, fixture.model))) return false;
  Results initial(fixture.model.parents().size());
  b::BatchDiagnostics diagnostics;
  if (!Good(Peer::ReadConstructed(batch, Buffer(initial), diagnostics)) || !Compare(initial, true)) return false;
  native_accepted = native_candidate;
  return !attach || Attach();
}
bool Rig::Attach() {
  if (!Good(Peer::PreflightAttach(batch, owner, fixture.ledger, fixture.binding,
      fixture.Witnesses(), fixture.model, config))) return false;
  Peer::Attach(batch);
  return true;
}
bool Rig::Begin(fe::NodalTrialToken& token, fe::NodalAssemblyView& assembly) {
  return Good(owner.BeginTrial(&token, &assembly)) && Good(batch.AssembleAccepted(owner, token, assembly));
}
bool Rig::Prepare(const fe::NodalTrialToken& token, const fe::NodalAssemblyView& assembly,
    fe::NodalPreparedView& prepared) {
  fe::NodalCinAssemblyView cin;
  if (!Good(owner.BorrowCinAssembly(token, &cin))) return false;
  // Explicit test-only coefficients for the disjoint shell/CIN patches whose
  // force producers are outside this one-family resident gate.
  std::vector<double> stiffness(cin.node_count);
  if (cudaMemcpyAsync(stiffness.data(), cin.translational_stiffness, stiffness.size()*sizeof(double),
      cudaMemcpyDeviceToHost, cin.stream) != cudaSuccess || cudaStreamSynchronize(cin.stream) != cudaSuccess) return false;
  const auto rows = fixture.mechanics.cin_model.rows();
  for (std::size_t p = 0; p < rows.count; ++p) {
    for (auto node : rows.data[p].master_domain_nodes) stiffness[node] += 1;
    stiffness[rows.data[p].secondary_domain_node] += 1;
  }
  if (cudaMemcpyAsync(cin.translational_stiffness, stiffness.data(), stiffness.size()*sizeof(double),
      cudaMemcpyHostToDevice, cin.stream) != cudaSuccess ||
      cudaMemsetAsync(cin.witness_activity, 1, cin.witness_count, cin.stream) != cudaSuccess) return false;
  const auto node = fixture.mechanics.domain.Find(9305);
  double loads[2]{};
  if (cudaMemcpyAsync(loads, assembly.forces.force_x+node, sizeof(double), cudaMemcpyDeviceToHost, cin.stream) != cudaSuccess ||
      cudaMemcpyAsync(loads+1, assembly.forces.couple_z+node, sizeof(double), cudaMemcpyDeviceToHost, cin.stream) != cudaSuccess ||
      cudaStreamSynchronize(cin.stream) != cudaSuccess) return false;
  loads[0] += 10.; loads[1] += .001;
  if (cudaMemcpyAsync(assembly.forces.force_x+node, loads, sizeof(double), cudaMemcpyHostToDevice, cin.stream) != cudaSuccess ||
      cudaMemcpyAsync(assembly.forces.couple_z+node, loads+1, sizeof(double), cudaMemcpyHostToDevice, cin.stream) != cudaSuccess ||
      cudaStreamSynchronize(cin.stream) != cudaSuccess) return false;
  if (!Good(owner.SealAssembly(token)) || !Good(fe::AdvanceStaggeredCin(owner, token,
      {assembly.owner_id, assembly.accepted.base_epoch, assembly.attempt, cin.qualification_id,
       config.owner.fixed_dt, .2, true}))) return false;
  return Good(owner.BorrowPrepared(token, &prepared));
}
bool Rig::Read(Results& results, b::BatchDiagnostics& diagnostics) {
  return Good(batch.CopyAcceptedResults(owner.accepted(), Buffer(results), &diagnostics));
}
bool Rig::Compare(const Results& results, bool initial, const fe::NodalPreparedView* view) {
  const auto parents = fixture.model.parents();
  std::vector<double> x(3*config.owner.node_count), v(x.size()), w(x.size());
  if (!initial) {
    if (!view) return false;
    if (cudaMemcpyAsync(x.data(), view->kinematics.position_xyz, x.size()*sizeof(double), cudaMemcpyDeviceToHost, view->stream) != cudaSuccess ||
        cudaMemcpyAsync(v.data(), view->kinematics.velocity_xyz, v.size()*sizeof(double), cudaMemcpyDeviceToHost, view->stream) != cudaSuccess ||
        cudaMemcpyAsync(w.data(), view->kinematics.angular_velocity_xyz, w.size()*sizeof(double), cudaMemcpyDeviceToHost, view->stream) != cudaSuccess ||
        cudaStreamSynchronize(view->stream) != cudaSuccess) return false;
  }
  native_candidate.resize(parents.size());
  for (std::size_t p = 0; p < parents.size(); ++p) {
    SCOPED_TRACE(p);
    b::PrescribedInterval motion;
    if (!initial) {
      motion.base_time_s = view->base_time;
      motion.dt_s = config.owner.fixed_dt;
      motion.sample_index = view->kinematics.base_epoch + 1;
      for (unsigned n = 0; n < 2; ++n) {
        const auto k = 3*parents[p].domain_nodes[n];
        motion.position_endpoint_m[n] = {x[k], x[k+1], x[k+2]};
        motion.velocity_midpoint_m_s[n] = {v[k], v[k+1], v[k+2]};
        motion.angular_velocity_midpoint_rad_s[n] = {w[k], w[k+1], w[k+2]};
      }
    }
    const auto& material = fixture.model.materials()[parents[p].material_index].value;
    native_candidate[p] = beam18_force_test::Native(parents[p].reference, material,
        initial ? beam18_force_test::NativeState{} : native_accepted[p].next, motion, initial);
    CompareResult(parents[p], material, results[p], native_candidate[p]);
  }
  return !::testing::Test::HasFailure();
}
} // namespace beam18_resident_test
