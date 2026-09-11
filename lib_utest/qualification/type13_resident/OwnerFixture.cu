// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"

namespace type13_resident_test {
bool Rig::Initialize(const t::ModelInput& input, bool moving) {
  if (!source.Initialize(input)) {
    return false;
  }
  const auto n = source.domain.node_count();
  Snapshot initial(n);
  mass.assign(n, 0);
  inertia.assign(n, 0);
  loads.assign(6*n, 0);
  for (const auto& record : source.contributions.records()) {
    const auto node = record.value.global_node;
    mass[node] += record.value.coefficients.mass_kg;
    // Native total J already includes added J. It is added exactly once.
    inertia[node] += record.value.coefficients.isotropic_inertia_kg_m2;
  }
  std::vector<double> inverse(n), inverse_j(n);
  std::vector<std::uint8_t> fixed(n);
  for (std::size_t i = 0; i < n; ++i) {
    const auto p = source.domain.nodes()[i].position;
    initial.x[3*i] = p.x;
    initial.x[3*i+1] = p.y;
    initial.x[3*i+2] = p.z;
    initial.v[3*i] = moving ? 8 : 0;
    initial.q[4*i] = 1;
    if (!(mass[i] > 0) || !(inertia[i] > 0)) {
      return false;
    }
    inverse[i] = 1/mass[i];
    inverse_j[i] = 1/inertia[i];
  }
  auto requested = Config(n);
  fe::NodalStateConfig settings;
  settings.node_count = n;
  settings.max_nodes = std::max<std::size_t>(2048, n);
  settings.max_device_bytes = 32u << 20;
  settings.fixed_dt = requested.owner.fixed_dt;
  settings.temporal_scheme = fe::NodalTemporalScheme::StaggeredHalfKickStart;
  if (!Good(owner.Initialize(settings,
      {initial.x.data(), initial.v.data(), initial.w.data(), n, initial.q.data()},
      inverse.data(), fe::NodalDofConfig{fixed.data(), fixed.data(), inverse_j.data()}))) {
    return false;
  }
  config = Config(n);
  config.owner = owner.accepted();
  if (moving) {
    config.startup = {fe::ShellBatchStartupKind::ReferenceUniformTranslation, {8, 0, 0}};
  }
  if (!Good(batch.InitializeJoined(config, source.contributions))) {
    return false;
  }
  Startup startup;
  if (!Good(startup.Initialize(config, source))) {
    return false;
  }
  native.resize(source.model.connection_count());
  for (std::size_t e = 0; e < native.size(); ++e) {
    const auto& element = startup.header.model.elements[e];
    t::NativeEndpointKinematics nodes[2];
    for (unsigned local = 0; local < 2; ++local) {
      nodes[local].position = element.original_position_native[local];
      nodes[local].velocity = {moving ? 8000. : 0, 0, 0};
    }
    native[e] = type13_recurrence_test::NativeEvaluate(*source.model.property(element.property),
        element.reference, Virgin(element.reference), nodes, 0, true);
  }
  fe::NodalTrialToken token;
  fe::NodalAssemblyView view;
  if (!Begin(token, view)) {
    return false;
  }
  CompareAssembly(view);
  Discard();
  return true;
}

bool Rig::Begin(fe::NodalTrialToken& token, fe::NodalAssemblyView& view, double acceleration) {
  if (!Good(owner.BeginTrial(&token, &view))) {
    return false;
  }
  const auto n = source.domain.node_count();
  double* destinations[] = {view.forces.force_x, view.forces.force_y, view.forces.force_z,
                            view.forces.couple_x, view.forces.couple_y, view.forces.couple_z};
  for (unsigned axis = 0; axis < 3; ++axis) {
    for (std::size_t i = 0; i < n; ++i) {
      const double pattern = static_cast<double>(i % 7) - 3;
      loads[axis*n+i] = mass[i] * acceleration * pattern * (axis+1);
      loads[(axis+3)*n+i] = inertia[i] * acceleration * .01 * pattern * (axis+1);
    }
  }
  for (unsigned c = 0; c < 6; ++c) {
    if (cudaMemcpyAsync(destinations[c], loads.data()+c*n, n*sizeof(double),
                         cudaMemcpyHostToDevice, view.stream) != cudaSuccess) {
      return false;
    }
  }
  return Good(batch.AssembleAccepted(owner, token, view));
}

bool Rig::Prepare(const fe::NodalTrialToken& token, const fe::NodalAssemblyView& view,
                   fe::NodalPreparedView& prepared) {
  if (!Good(owner.SealAssembly(token))) {
    return false;
  }
  if (!Good(fe::AdvanceStaggeredHistory(owner, token,
      {view.owner_id, view.accepted.base_epoch, view.attempt,
       config.owner.fixed_dt, .2, config.qualification_id}))) {
    return false;
  }
  return Good(owner.BorrowPrepared(token, &prepared));
}
bool Rig::Read(std::vector<t::Evaluation>& out, t::BatchDiagnostics& diagnostics) {
  out.resize(source.model.connection_count());
  return Good(batch.CopyAcceptedResults(owner.accepted(), out.data(), out.size(), &diagnostics));
}
} // namespace type13_resident_test
