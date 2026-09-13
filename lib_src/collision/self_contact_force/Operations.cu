// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "../penalty_pair/Values.h"
#include "lib_src/solvers/NodalNativePhysicalCoefficients.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

#include <algorithm>

namespace tlfea::contact {
namespace scf = self_contact_force;
namespace fe = tl::fea;
namespace {

constexpr unsigned Threads = 128;
constexpr unsigned MaximumBlocks = 256;

unsigned Blocks(std::size_t count) noexcept {
  if (!count) return 0;
  const auto blocks = 1 + (count - 1) / Threads;
  return static_cast<unsigned>(
      blocks > MaximumBlocks ? MaximumBlocks : blocks);
}

__device__ void Fail(scf::Control& control,
                     SelfContactForceStatus status,
                     std::uint32_t event = UINT32_MAX,
                     std::uint64_t source_order = UINT64_MAX,
                     std::uint32_t node = UINT32_MAX,
                     SurfacePenaltyStatus pair =
                         SurfacePenaltyStatus::InvalidInput) {
  if (control.status != SelfContactForceStatus::Ok) return;
  control.status = status;
  control.event = event;
  control.source_order = source_order;
  control.node = node;
  control.pair_status = pair;
}

__device__ bool ValidAssembly(const fe::NodalAssemblyView& view,
                              const fe::NodalCinAssemblyView& cin) {
  if (!view.accepted.position_xyz || !view.accepted.velocity_xyz ||
      !view.translation_fixed_bits || !view.result || !view.bounds ||
      !view.forces.force_x || !view.forces.force_y ||
      !view.forces.force_z || !view.forces.couple_x ||
      !view.forces.couple_y || !view.forces.couple_z ||
      !cin.translational_stiffness || !cin.rotational_stiffness ||
      view.accepted.node_count != view.forces.node_count ||
      view.accepted.node_count != cin.node_count ||
      view.forces.base_epoch != view.accepted.base_epoch ||
      view.result->base_epoch != view.accepted.base_epoch ||
      view.result->attempt != view.attempt ||
      view.bounds->base_epoch != view.accepted.base_epoch ||
      view.bounds->attempt != view.attempt ||
      !view.bounds->initialized || !view.bounds->valid ||
      view.bounds->sealed || view.result->status != Status::kOk ||
      cin.owner_id != view.owner_id ||
      cin.base_epoch != view.accepted.base_epoch ||
      cin.attempt != view.attempt || cin.stream != view.stream)
    return false;
  double* arrays[] = {
      view.forces.force_x, view.forces.force_y, view.forces.force_z,
      view.forces.couple_x, view.forces.couple_y, view.forces.couple_z};
  for (unsigned i = 0; i < 6; ++i)
    for (unsigned j = 0; j < i; ++j)
      if (arrays[i] == arrays[j]) return false;
  return cin.translational_stiffness != cin.rotational_stiffness;
}

__global__ void Begin(scf::Buffers buffers,
                      std::size_t event_count,
                      const SelfContactForceConfig config,
                      fe::NodalAssemblyView view,
                      fe::NodalCinAssemblyView cin) {
  *buffers.control = {};
  *buffers.diagnostics = {};
  auto& diagnostics = *buffers.diagnostics;
  diagnostics.event_count = event_count;
  diagnostics.owner_id = view.owner_id;
  diagnostics.base_epoch = view.accepted.base_epoch;
  diagnostics.attempt = view.attempt;
  diagnostics.configuration_id = config.configuration_id;
  diagnostics.qualification_id = config.qualification_id;
  diagnostics.temporal_scheme = view.temporal_scheme;
  diagnostics.velocity_phase = view.velocity_phase;
  diagnostics.position_time = view.position_time;
  diagnostics.velocity_time = view.velocity_time;
  if (!ValidAssembly(view, cin))
    Fail(*buffers.control, SelfContactForceStatus::AssemblyFailure);
}

__global__ void EvaluateEvents(scf::Buffers buffers,
                               std::size_t event_count,
                               double stiffness_per_area,
                               fe::NodalAssemblyView view) {
  for (std::size_t event = blockIdx.x * blockDim.x + threadIdx.x;
       event < event_count; event += blockDim.x * gridDim.x) {
    auto& status = buffers.event_status[event];
    status = {};
    if (buffers.control->status != SelfContactForceStatus::Ok) continue;
    const auto& source = buffers.events[event];
    double stiffness = 0;
    if (!RepresentedSelfContactStiffness(
            stiffness_per_area,
            source.classification.admitted_force_area_m2,
            &stiffness)) {
      status.status = SelfContactForceStatus::EventFailure;
      status.pair_status = SurfacePenaltyStatus::Unrepresentable;
      continue;
    }
    SurfacePenaltyInput input;
    input.positions = {
        view.accepted.position_xyz,
        static_cast<std::uint32_t>(view.accepted.node_count), 3, 1};
    input.velocities = {
        view.accepted.velocity_xyz,
        static_cast<std::uint32_t>(view.accepted.node_count), 3, 1};
    input.stiffness_n_m = stiffness;
    for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
      auto& target = endpoint ? input.b : input.a;
      target.point = source.endpoints[endpoint];
      target.reference_half_thickness_m =
          source.classification.reference_half_thickness_m[endpoint];
      for (std::uint32_t slot = 0; slot < target.point.count; ++slot) {
        const auto node = target.point.nodes[slot];
        const auto mask = view.translation_fixed_bits[node];
        if (mask > 7) {
          status.status = SelfContactForceStatus::EventFailure;
          status.pair_status = SurfacePenaltyStatus::InvalidInput;
        }
        target.translation_fixed_bits[slot] = mask;
      }
    }
    if (status.status != SelfContactForceStatus::Ok) continue;
    SurfacePenaltyPacket packet;
    const auto pair = EvaluateSurfacePenaltyPair(input, &packet);
    if (pair != SurfacePenaltyStatus::Ok) {
      status.status = SelfContactForceStatus::EventFailure;
      status.pair_status = pair;
      continue;
    }
    buffers.packets[event] = packet;
  }
}

__global__ void CheckEvents(scf::Buffers buffers,
                            std::size_t event_count) {
  if (buffers.control->status != SelfContactForceStatus::Ok) return;
  for (std::size_t event = 0; event < event_count; ++event) {
    const auto status = buffers.event_status[event];
    if (status.status != SelfContactForceStatus::Ok) {
      Fail(*buffers.control, status.status,
           static_cast<std::uint32_t>(event),
           buffers.events[event].source_order, UINT32_MAX,
           status.pair_status);
      return;
    }
  }
}

__device__ bool AddChecked(Vec3 term, Vec3& sum) {
  sum = Add(sum, term);
  return IsFinite(sum);
}

__device__ bool AddChecked(double term, double& sum) {
  sum += term;
  return IsFinite(sum);
}

__device__ Vec3 Moment(Vec3 position, Vec3 force) {
  return {
      position.y * force.z - position.z * force.y,
      position.z * force.x - position.x * force.z,
      position.x * force.y - position.y * force.x};
}

__global__ void ReduceDiagnostics(scf::Buffers buffers,
                                  std::size_t event_count,
                                  double stiffness_per_area,
                                  fe::NodalAssemblyView view,
                                  const void* source_identity) {
  if (buffers.control->status != SelfContactForceStatus::Ok) return;
  auto& out = *buffers.diagnostics;
  out.active_use_identity = source_identity;
  if (event_count) {
    out.first_source_order = buffers.events[0].source_order;
    out.last_source_order = buffers.events[event_count - 1].source_order;
  }
  for (std::size_t event = 0; event < event_count; ++event) {
    const auto& packet = buffers.packets[event];
    const auto& source = buffers.events[event];
    bool valid = packet.valid;
    double represented = 0;
    valid = valid && RepresentedSelfContactStiffness(
        stiffness_per_area,
        source.classification.admitted_force_area_m2, &represented);
    if (packet.active) ++out.active_count;
    valid = valid && AddChecked(packet.force_a_n,
                                out.endpoint_a_resultant_n);
    valid = valid && AddChecked(packet.force_b_n,
                                out.endpoint_b_resultant_n);
    valid = valid && AddChecked(packet.elastic_energy_j,
                                out.potential_j);
    out.maximum_represented_stiffness_n_m =
        ::fmax(out.maximum_represented_stiffness_n_m, represented);
    for (std::uint32_t i = 0; valid && i < packet.count; ++i) {
      const auto& node = packet.nodes[i];
      const auto position = VectorView{
          view.accepted.position_xyz,
          static_cast<std::uint32_t>(view.accepted.node_count),
          3, 1}.at(node.node);
      const auto moment = Moment(position, node.force_n);
      double norm = 0;
      valid = IsFinite(position) && IsFinite(moment) &&
          mass_detail::UpperNorm(node.force_n, &norm) &&
          AddChecked(moment, out.global_moment_n_m);
      out.maximum_force_norm_n =
          ::fmax(out.maximum_force_norm_n, norm);
    }
    for (std::uint32_t i = 0;
         valid && i < packet.normal_majorant.count; ++i) {
      const double diagonal =
          packet.normal_majorant.nodes[i].diagonal_n_m;
      valid = IsFinite(diagonal) && diagonal >= 0;
      out.maximum_sti_diagonal_n_m =
          ::fmax(out.maximum_sti_diagonal_n_m, diagonal);
    }
    if (!valid || !IsFinite(out.maximum_force_norm_n) ||
        !IsFinite(out.maximum_sti_diagonal_n_m) ||
        !IsFinite(out.maximum_represented_stiffness_n_m)) {
      Fail(*buffers.control, SelfContactForceStatus::EventFailure,
           static_cast<std::uint32_t>(event), source.source_order,
           UINT32_MAX, SurfacePenaltyStatus::NonFiniteResult);
      return;
    }
  }
  out.equal_opposite_residual_n =
      Add(out.endpoint_a_resultant_n, out.endpoint_b_resultant_n);
  if (!IsFinite(out.equal_opposite_residual_n))
    Fail(*buffers.control, SelfContactForceStatus::EventFailure,
         event_count ? static_cast<std::uint32_t>(event_count - 1)
                     : UINT32_MAX,
         event_count ? buffers.events[event_count - 1].source_order
                     : UINT64_MAX,
         UINT32_MAX, SurfacePenaltyStatus::NonFiniteResult);
}

__device__ double* Channel(const fe::DeviceNodalForceView& forces,
                           unsigned channel) {
  double* values[] = {forces.force_x, forces.force_y, forces.force_z,
                      forces.couple_x, forces.couple_y,
                      forces.couple_z};
  return values[channel];
}

__global__ void StageNodes(scf::Buffers buffers,
                           std::size_t touched_nodes,
                           fe::NodalAssemblyView view,
                           fe::NodalCinAssemblyView cin) {
  for (std::size_t compact = blockIdx.x * blockDim.x + threadIdx.x;
       compact < touched_nodes; compact += blockDim.x * gridDim.x) {
    auto& status = buffers.node_status[compact];
    status = {};
    if (buffers.control->status != SelfContactForceStatus::Ok) continue;
    const auto source = buffers.nodes[compact];
    bool valid = source.node < view.accepted.node_count;
    for (unsigned channel = 0; valid && channel < 6; ++channel) {
      const double value = Channel(view.forces, channel)[source.node];
      buffers.staged_channels[6 * compact + channel] = value;
      valid = IsFinite(value);
    }
    double stiffness = valid
        ? cin.translational_stiffness[source.node] : 0;
    valid = valid && IsFinite(stiffness) && stiffness >= 0;
    for (std::uint32_t i = 0; valid && i < source.count; ++i) {
      const auto incidence = buffers.incidences[source.offset + i];
      valid = incidence.node == source.node;
      if (!valid) break;
      const auto& packet = buffers.packets[incidence.event];
      const SurfacePenaltyNode* force = nullptr;
      const SurfaceNodeMajorant* majorant = nullptr;
      for (std::uint32_t n = 0; n < packet.count; ++n)
        if (packet.nodes[n].node == source.node)
          force = &packet.nodes[n];
      for (std::uint32_t n = 0; n < packet.normal_majorant.count; ++n)
        if (packet.normal_majorant.nodes[n].node == source.node)
          majorant = &packet.normal_majorant.nodes[n];
      if (!force || !majorant || !IsFinite(force->force_n) ||
          !IsFinite(majorant->diagonal_n_m) ||
          majorant->diagonal_n_m < 0) {
        valid = false;
        break;
      }
      double* staged = buffers.staged_channels + 6 * compact;
      staged[0] += force->force_n.x;
      staged[1] += force->force_n.y;
      staged[2] += force->force_n.z;
      stiffness += majorant->diagonal_n_m;
      valid = IsFinite(staged[0]) && IsFinite(staged[1]) &&
          IsFinite(staged[2]) && IsFinite(stiffness) &&
          stiffness >= 0;
    }
    buffers.staged_sti[compact] = stiffness;
    if (!valid) status.status = SelfContactForceStatus::AssemblyFailure;
  }
}

__global__ void CheckNodes(scf::Buffers buffers,
                           std::size_t touched_nodes) {
  if (buffers.control->status != SelfContactForceStatus::Ok) return;
  for (std::size_t compact = 0; compact < touched_nodes; ++compact) {
    if (buffers.node_status[compact].status !=
        SelfContactForceStatus::Ok) {
      Fail(*buffers.control, SelfContactForceStatus::AssemblyFailure,
           UINT32_MAX, UINT64_MAX, buffers.nodes[compact].node);
      return;
    }
  }
}

__global__ void PublishNodes(scf::Buffers buffers,
                             std::size_t touched_nodes,
                             fe::NodalAssemblyView view,
                             fe::NodalCinAssemblyView cin) {
  if (buffers.control->status != SelfContactForceStatus::Ok) return;
  for (std::size_t compact = blockIdx.x * blockDim.x + threadIdx.x;
       compact < touched_nodes; compact += blockDim.x * gridDim.x) {
    const auto node = buffers.nodes[compact].node;
    for (unsigned channel = 0; channel < 3; ++channel)
      Channel(view.forces, channel)[node] =
          buffers.staged_channels[6 * compact + channel];
    cin.translational_stiffness[node] =
        buffers.staged_sti[compact];
  }
}

__global__ void Finish(scf::Buffers buffers,
                       fe::NodalAssemblyView view) {
  if (buffers.control->status == SelfContactForceStatus::Ok)
    buffers.diagnostics->valid = true;
  else
    fe::RecordNodalAssemblyFailure(
        view, Status::kInvalidArgument, buffers.control->node);
}

SelfContactForceReport OwnerReport(const fe::NodalReport& report) noexcept {
  return {
      report.status == fe::NodalStatus::DeviceFailure
          ? SelfContactForceStatus::DeviceFailure
          : SelfContactForceStatus::OwnerFailure,
      SIZE_MAX, UINT64_MAX, report.node,
      SurfacePenaltyStatus::InvalidInput, report.status, report.message};
}

}  // namespace

SelfContactForceReport SelfContactForceAssembly::Impl::
ReadControlAndDiagnostics(SelfContactForceDiagnostics& diagnostics) noexcept {
  auto report = Check(cudaGetLastError());
  if (report.status != SelfContactForceStatus::Ok) return report;
  report = Check(cudaMemcpyAsync(
      local.control, remote.control, sizeof(*local.control),
      cudaMemcpyDeviceToHost, stream));
  if (report.status != SelfContactForceStatus::Ok) return report;
  report = Check(cudaMemcpyAsync(
      local.diagnostics, remote.diagnostics,
      sizeof(*local.diagnostics), cudaMemcpyDeviceToHost, stream));
  if (report.status != SelfContactForceStatus::Ok) return report;
  report = Check(cudaStreamSynchronize(stream));
  if (report.status != SelfContactForceStatus::Ok) return report;
  control = *local.control;
  if (control.status != SelfContactForceStatus::Ok)
    return {control.status, control.event, control.source_order,
            control.node, control.pair_status, fe::NodalStatus::Ok,
            control.status == SelfContactForceStatus::EventFailure
                ? "Canonical self-contact event evaluation failed"
                : "Failure-atomic self-contact node assembly failed"};
  if (!local.diagnostics->valid)
    return {SelfContactForceStatus::AssemblyFailure, SIZE_MAX,
            UINT64_MAX, UINT32_MAX,
            SurfacePenaltyStatus::NonFiniteResult,
            fe::NodalStatus::Ok,
            "Self-contact diagnostics were not completed"};
  diagnostics = *local.diagnostics;
  return {};
}

SelfContactForceReport SelfContactForceAssembly::AssembleAccepted(
    fe::FENodalState& owner,
    const fe::NodalTrialToken& token,
    const fe::NodalAssemblyView& view,
    SelfContactForceEventView events,
    SelfContactForceAssemblyReceipt* output) {
  using S = SelfContactForceStatus;
  if (!impl_)
    return {S::NotInitialized, SIZE_MAX, UINT64_MAX, UINT32_MAX,
            SurfacePenaltyStatus::InvalidInput, fe::NodalStatus::Ok,
            "Self-contact force assembly is not initialized"};
  auto& state = *impl_;
  if (!state.usable)
    return {S::DeviceFailure, SIZE_MAX, UINT64_MAX, UINT32_MAX,
            SurfacePenaltyStatus::InvalidInput,
            fe::NodalStatus::DeviceFailure,
            "Self-contact force assembly is CUDA-poisoned"};
  using fe::trial_identity::Disjoint;
  if (&owner != state.owner || !output ||
      events.count > state.storage_forecast.event_capacity ||
      (events.count && (!events.data ||
       events.count > SIZE_MAX / sizeof(SelfContactForceEvent))) ||
      !state.OutputDisjoint(output, sizeof(*output)) ||
      !Disjoint(output, sizeof(*output), this, sizeof(*this)) ||
      !Disjoint(output, sizeof(*output), &token, sizeof(token)) ||
      !Disjoint(output, sizeof(*output), &view, sizeof(view)))
    return {S::InvalidInput, SIZE_MAX, UINT64_MAX, UINT32_MAX,
            SurfacePenaltyStatus::InvalidInput, fe::NodalStatus::Ok,
            "Force events, output or owner alias/capacity is invalid"};
  const std::size_t event_bytes =
      events.count * sizeof(SelfContactForceEvent);
  if (events.count &&
      (!Disjoint(output, sizeof(*output), events.data, event_bytes) ||
       !Disjoint(events.data, event_bytes, state.host.data(),
                 state.host.bytes()) ||
       !state.OutputDisjoint(events.data, event_bytes)))
    return {S::InvalidInput, SIZE_MAX, UINT64_MAX, UINT32_MAX,
            SurfacePenaltyStatus::InvalidInput, fe::NodalStatus::Ok,
            "Force events overlap output or retained staging"};

  const auto stamp = owner.accepted();
  if (!fe::native_physical_coefficients::SameOwnerScope(
          state.config.owner, stamp) ||
      view.attempt <= state.last_attempt ||
      (!state.last_attempt && stamp.epoch != 0) ||
      (state.last_attempt &&
       (stamp.epoch < state.last_base_epoch ||
        stamp.epoch - state.last_base_epoch > 1)))
    return {S::StaleAttempt, SIZE_MAX, UINT64_MAX, UINT32_MAX,
            SurfacePenaltyStatus::InvalidInput, fe::NodalStatus::StaleTrial,
            "Self-contact accepted base is skipped, stale or consumed"};

  SelfContactForceIncidenceSummary incidence_summary;
  if (events.count) {
    std::copy_n(events.data, events.count, state.local.events);
    auto report = BuildSelfContactForceIncidence(
        state.local.events, events.count,
        static_cast<std::uint32_t>(stamp.node_count),
        state.local.incidences,
        state.storage_forecast.incidence_capacity,
        state.local.nodes,
        state.storage_forecast.touched_node_capacity,
        &incidence_summary);
    if (report.status != S::Ok) return report;
    const auto& first = state.local.events[0].classification;
    if (!first.activity_base_identity ||
        !first.activity_current_identity ||
        first.activity_parent_count != state.binding.parents().size() ||
        !Disjoint(output, sizeof(*output),
                  first.activity_base_identity,
                  first.activity_parent_count) ||
        !Disjoint(output, sizeof(*output),
                  first.activity_current_identity,
                  first.activity_parent_count))
      return {S::IdentityMismatch, 0,
              state.local.events[0].source_order, UINT32_MAX,
              SurfacePenaltyStatus::InvalidInput, fe::NodalStatus::Ok,
              "Event activity authority differs from active-use source"};
    for (std::size_t parent = 0;
         parent < first.activity_parent_count; ++parent)
      if (first.activity_base_identity[parent] > 1 ||
          first.activity_current_identity[parent] >
              first.activity_base_identity[parent])
        return {S::StaleAttempt, 0,
                state.local.events[0].source_order, UINT32_MAX,
                SurfacePenaltyStatus::InvalidInput,
                fe::NodalStatus::StaleTrial,
                "Event activity source is stale or invalid"};
    for (std::size_t event = 0; event < events.count; ++event) {
      const auto& classification =
          state.local.events[event].classification;
      if (classification.activity_base_identity !=
              first.activity_base_identity ||
          classification.activity_current_identity !=
              first.activity_current_identity ||
          classification.activity_parent_count !=
              first.activity_parent_count)
        return {S::IdentityMismatch, event,
                state.local.events[event].source_order, UINT32_MAX,
                SurfacePenaltyStatus::InvalidInput, fe::NodalStatus::Ok,
                "Event batch uses multiple activity authorities"};
      const auto report = scf::ValidateEvent(
          state.binding, state.local.events[event], event);
      if (report.status != S::Ok) return report;
    }
  }

  const auto source = state.binding.cin();
  fe::NodalCinAssemblyView cin;
  const auto borrowed = fe::shell_physical_owner::BorrowAssembly(
      owner, token, stamp, view, source.witness_count, &cin);
  if (borrowed.status != fe::NodalStatus::Ok) {
    if (borrowed.status == fe::NodalStatus::DeviceFailure)
      state.usable = false;
    return OwnerReport(borrowed);
  }
  if (cin.qualification_id != state.config.qualification_id)
    return {S::IdentityMismatch, SIZE_MAX, UINT64_MAX, UINT32_MAX,
            SurfacePenaltyStatus::InvalidInput, fe::NodalStatus::Ok,
            "Actual CIN qualification differs from force source identity"};
  state.stream = view.stream;
  state.last_attempt = view.attempt;
  state.last_base_epoch = stamp.epoch;

  auto report = state.Check(cudaMemcpyAsync(
      state.remote.events, state.local.events,
      events.count * sizeof(SelfContactForceEvent),
      cudaMemcpyHostToDevice, state.stream));
  if (report.status != S::Ok) return report;
  if (incidence_summary.incidences) {
    report = state.Check(cudaMemcpyAsync(
        state.remote.incidences, state.local.incidences,
        incidence_summary.incidences * sizeof(SelfContactForceIncidence),
        cudaMemcpyHostToDevice, state.stream));
    if (report.status != S::Ok) return report;
  }
  if (incidence_summary.touched_nodes) {
    report = state.Check(cudaMemcpyAsync(
        state.remote.nodes, state.local.nodes,
        incidence_summary.touched_nodes *
            sizeof(SelfContactForceNodeIncidence),
        cudaMemcpyHostToDevice, state.stream));
    if (report.status != S::Ok) return report;
  }

  Begin<<<1, 1, 0, state.stream>>>(
      state.remote, events.count, state.config, view, cin);
  report = state.Check(cudaGetLastError());
  if (report.status != S::Ok) return report;
  if (events.count) {
    EvaluateEvents<<<Blocks(events.count), Threads, 0, state.stream>>>(
        state.remote, events.count,
        state.config.stiffness_per_area_n_m3, view);
    report = state.Check(cudaGetLastError());
    if (report.status != S::Ok) return report;
  }
  CheckEvents<<<1, 1, 0, state.stream>>>(state.remote, events.count);
  report = state.Check(cudaGetLastError());
  if (report.status != S::Ok) return report;
  ReduceDiagnostics<<<1, 1, 0, state.stream>>>(
      state.remote, events.count,
      state.config.stiffness_per_area_n_m3, view,
      state.binding.identity());
  report = state.Check(cudaGetLastError());
  if (report.status != S::Ok) return report;
  if (incidence_summary.touched_nodes) {
    StageNodes<<<Blocks(incidence_summary.touched_nodes), Threads,
                 0, state.stream>>>(
        state.remote, incidence_summary.touched_nodes, view, cin);
    report = state.Check(cudaGetLastError());
    if (report.status != S::Ok) return report;
  }
  CheckNodes<<<1, 1, 0, state.stream>>>(
      state.remote, incidence_summary.touched_nodes);
  report = state.Check(cudaGetLastError());
  if (report.status != S::Ok) return report;
  if (incidence_summary.touched_nodes) {
    PublishNodes<<<Blocks(incidence_summary.touched_nodes), Threads,
                   0, state.stream>>>(
        state.remote, incidence_summary.touched_nodes, view, cin);
    report = state.Check(cudaGetLastError());
    if (report.status != S::Ok) return report;
  }
  Finish<<<1, 1, 0, state.stream>>>(state.remote, view);
  report = state.Check(cudaGetLastError());
  if (report.status != S::Ok) return report;

  SelfContactForceDiagnostics diagnostics;
  report = state.ReadControlAndDiagnostics(diagnostics);
  if (report.status != S::Ok) return report;
  SelfContactForceAssemblyReceipt next;
  next.assembler_identity_ = state.device;
  next.diagnostics_ = diagnostics;
  *output = next;
  return {};
}

}  // namespace tlfea::contact
