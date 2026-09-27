// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "DeviceFamilies.h"
#include "../../ShellBatchFields.h"
#include "../../../solvers/NodalForceAssembly.h"
#include "../../../solvers/NodalRepeatedForceAssembly.h"
#include "../../../solvers/NodalRepeatedStiffness.h"

namespace tl::fea::solids::batch_detail {
namespace assembly_serial {
template<class Traits> __device__ bool CheckNodes(Storage& state, NodalAssemblyView view) {
  const auto& family = FamilyStorage<Traits>(state);
  for (std::size_t p = 0; p < family.count; ++p) {
    for (unsigned n = 0; n < Traits::nodes; ++n) {
      const auto node = family.parents[p].domain_nodes[n];
      if (node >= view.accepted.node_count ||
          !tl::math::fixed3::Finite(shell_batch_fields::ReadVector(view.accepted.position_xyz, node)) ||
          !tl::math::fixed3::Finite(shell_batch_fields::ReadVector(view.accepted.velocity_xyz, node))) {
        state.control.status = BatchStatus::InvalidInput;
        state.control.family = Traits::family;
        state.control.parent = p;
        state.control.node = node;
        return false;
      }
    }
  }
  return true;
}
template<class Traits> __device__ bool AddFamily(Storage& state, unsigned slab,
    NodalAssemblyView view, NodalCinAssemblyView cin) {
  const auto& family = FamilyStorage<Traits>(state);
  for (std::size_t p = 0; p < family.count; ++p) {
    const auto& parent = family.parents[p];
    const auto& cache = family.slab[slab][p].cache;
    if constexpr(std::is_same_v<Traits,Traits24>) {
      bool repeated=false;
      for(unsigned a=0;a<8;++a)for(unsigned b=0;b<a;++b)repeated|=parent.domain_nodes[a]==parent.domain_nodes[b];
      if(repeated) {
        double increments[8];for(double& value:increments)value=cache.stiffness.translation_n_m;
        if(AccumulateRepeatedNodalStiffness<8>(parent.domain_nodes,increments,cin.translational_stiffness,
             view.accepted.node_count)!=NodalForceAssemblyStatus::Success||
           AccumulateRepeatedNodalTranslationalForces<8>(parent.domain_nodes,cache.rhs_force_n,view.forces)!=NodalForceAssemblyStatus::Success) {
          state.control.family=Traits::family;state.control.parent=p;return false;
        }
        continue;
      }
    }
    double translation[Traits::nodes];
    for (unsigned n = 0; n < Traits::nodes; ++n) {
      const auto node = parent.domain_nodes[n];
      const double prior = cin.translational_stiffness[node];
      translation[n] = prior + cache.stiffness.translation_n_m;
      if (!tl::math::Finite(prior) || prior < 0 ||
          !tl::math::Finite(cache.stiffness.translation_n_m) || cache.stiffness.translation_n_m <= 0 ||
          !tl::math::Finite(translation[n]) || translation[n] < 0) {
        state.control.family = Traits::family;
        state.control.parent = p;
        state.control.node = node;
        return false;
      }
    }
    if (AccumulateNodalTranslationalForces<Traits::nodes>(parent.domain_nodes,
        cache.rhs_force_n, view.forces) != NodalForceAssemblyStatus::Success) {
      state.control.family = Traits::family;
      state.control.parent = p;
      return false;
    }
    for (unsigned n = 0; n < Traits::nodes; ++n)
      cin.translational_stiffness[parent.domain_nodes[n]] = translation[n];
    // Solids write neither couples nor STIFR, including their signed zero bits.
  }
  return true;
}
// The collapsed rear family retains eight native source slots. Both force and
// stiffness additions use that original slot order, including repeated NIDs.
__device__ inline bool AddRearFamily(Storage& state, unsigned slab,
    NodalAssemblyView view, NodalCinAssemblyView cin) {
  const auto& family = state.solid18_law44;
  for (std::size_t p = 0; p < family.count; ++p) {
    const auto& parent = family.parents[p];
    const auto& cache = family.slab[slab][p].cache;
    double increments[8];
    for (auto& value : increments) value = cache.stiffness.translation_n_m;
    if (AccumulateRepeatedNodalStiffness<8>(parent.domain_nodes, increments,
        cin.translational_stiffness, view.accepted.node_count) != NodalForceAssemblyStatus::Success ||
        AccumulateRepeatedNodalTranslationalForces<8>(parent.domain_nodes,
        cache.rhs_force_n, view.forces) != NodalForceAssemblyStatus::Success) {
      state.control.family = Family::Solid18Law44;
      state.control.parent = p;
      return false;
    }
  }
  return true;
}
__device__ inline void Assemble(Storage* storage, unsigned accepted,
    NodalAssemblyView view, NodalCinAssemblyView cin) {
  auto& state = *storage;
  state.control = {};
  if (view.result->base_epoch != view.accepted.base_epoch || view.result->attempt != view.attempt ||
      view.bounds->base_epoch != view.accepted.base_epoch || view.bounds->attempt != view.attempt ||
      !view.bounds->initialized || !view.bounds->valid || view.bounds->sealed ||
      view.result->status != tlfea::contact::Status::kOk) {
    state.control.status = BatchStatus::AssemblyFailure;
  } else if (CheckNodes<Traits18>(state, view) && CheckNodes<Traits24>(state, view) &&
      CheckNodes<Traits6z>(state, view) && CheckNodes<Traits18Law44>(state, view) &&
      CheckNodes<Traits18Law90>(state, view)) {
    // One writer preserves family18/24/6z/44/90, retained parent and original-slot
    // additive SI order. An error invalidates the entire owner's trial.
    if (!AddFamily<Traits18>(state, accepted, view, cin) ||
        !AddFamily<Traits24>(state, accepted, view, cin) ||
        !AddFamily<Traits6z>(state, accepted, view, cin) ||
        !AddRearFamily(state, accepted, view, cin) ||
        !AddFamily<Traits18Law90>(state, accepted, view, cin))
      state.control.status = BatchStatus::AssemblyFailure;
  }
  if (state.control.status != BatchStatus::Success)
    RecordNodalAssemblyFailure(view, tlfea::contact::Status::kInvalidArgument,
        static_cast<std::uint32_t>(state.control.node));
}
} // namespace assembly_serial
} // namespace tl::fea::solids::batch_detail
