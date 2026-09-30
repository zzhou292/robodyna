// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "../../beam_common/EndpointFields.h"
#include "../../beam_common/EndpointAssembly.h"

namespace tl::fea::beam18::batch_detail {
namespace {
__device__ bool CheckNodes(Storage& s, const NodalAssemblyView& view) {
  for (std::size_t p = 0; p < s.count; ++p) {
    const auto& parent = s.parents[p];
    beam_endpoint::Motion motion;
    if (!beam_endpoint::Gather(parent.domain_nodes, view.accepted, motion)) {
      s.control.status = BatchStatus::InvalidInput;
      s.control.parent = p;
      return false;
    }
    for (auto node : parent.domain_nodes) {
      // Rigid dependent inverse coefficients may be zero. Presence/fixed
      // policy is distinct from membership and from positive physical M/J.
      if (view.mass.fixed[node] || view.translation_fixed_bits[node] || view.rotation_fixed[node] ||
          (view.rotation_present && view.rotation_present[node] != 1) ||
          !tl::math::Finite(view.mass.inverse_mass[node]) || view.mass.inverse_mass[node] < 0 ||
          !tl::math::Finite(view.inverse_inertia[node]) || view.inverse_inertia[node] < 0) {
        s.control.status = BatchStatus::InvalidInput;
        s.control.parent = p;
        s.control.node = node;
        return false;
      }
    }
  }
  return true;
}
__global__ void Assemble(Storage* s, unsigned accepted, NodalAssemblyView view, NodalCinAssemblyView cin) {
  s->control = {};
  if (view.result->base_epoch != view.accepted.base_epoch || view.result->attempt != view.attempt ||
      view.bounds->base_epoch != view.accepted.base_epoch || view.bounds->attempt != view.attempt ||
      !view.bounds->initialized || !view.bounds->valid || view.bounds->sealed ||
      view.result->status != tlfea::contact::Status::kOk) {
    s->control.status = BatchStatus::AssemblyFailure;
  } else if (CheckNodes(*s, view)) {
    // One writer retains original parent/endpoint additions, including prior
    // producers' scalar values. The common owner discards on any failure.
    for (std::size_t p = 0; p < s->count; ++p) {
      const auto& value = s->slab[accepted][p];
      const auto status = beam_endpoint::Accumulate(s->parents[p].domain_nodes,
          value.rhs_force_n, value.rhs_couple_nm, value.diagnostics.translation_stiffness_n_m,
          value.diagnostics.rotation_stiffness_nm, view.forces,
          cin.translational_stiffness, cin.rotational_stiffness);
      if (status != NodalForceAssemblyStatus::Success) {
        s->control.status = BatchStatus::AssemblyFailure;
        s->control.parent = p;
        break;
      }
    }
  }
  if (s->control.status != BatchStatus::Success)
    RecordNodalAssemblyFailure(view, tlfea::contact::Status::kInvalidArgument,
        static_cast<std::uint32_t>(s->control.node));
}
} // namespace
void LaunchAssembly(Storage* s, unsigned accepted, NodalAssemblyView view, NodalCinAssemblyView cin) {
  Assemble<<<1, 1, 0, view.stream>>>(s, accepted, view, cin);
}
} // namespace tl::fea::beam18::batch_detail
