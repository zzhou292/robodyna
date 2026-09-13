// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "../SurfaceMaterialMeasure.h"
#include <array>
#include <cstdint>

namespace tlfea::contact::active_use {
namespace {
using S = SelfContactActiveUseStatus;
SelfContactActiveUseReport Fail(S status, const char* message) noexcept {
  return {status, SIZE_MAX, SIZE_MAX, message};
}
bool ValidRange(const void* pointer, std::size_t count, std::size_t width) noexcept {
  if (!count) return pointer == nullptr;
  if (!pointer || count > SIZE_MAX/width) return false;
  const auto address = reinterpret_cast<std::uintptr_t>(pointer);
  return count*width <= UINTPTR_MAX-address;
}
bool LimitsValid(const SelfContactActiveUseLimits& value) noexcept {
  const auto hard = SelfContactActiveUseLimits::Vehicle();
  return value.max_parents && value.max_parents <= hard.max_parents &&
      value.max_facets && value.max_facets <= hard.max_facets &&
      value.max_vertices && value.max_vertices <= hard.max_vertices &&
      value.max_edges && value.max_edges <= hard.max_edges &&
      value.max_vertex_uses && value.max_vertex_uses <= hard.max_vertex_uses &&
      value.max_edge_uses && value.max_edge_uses <= hard.max_edge_uses &&
      value.max_nodes && value.max_nodes <= hard.max_nodes &&
      value.max_cin_rows && value.max_cin_rows <= hard.max_cin_rows &&
      value.max_cin_witnesses && value.max_cin_witnesses <= hard.max_cin_witnesses &&
      value.max_host_bytes && value.max_host_bytes <= hard.max_host_bytes;
}
}

SelfContactActiveUseReport MakeLayout(const FixedContactFacetBinding& facets,
    SelfContactActiveUseSource source, SelfContactActiveUseLimits limits,
    std::size_t implementation_bytes, Layout& output) noexcept {
  if (!facets.prepared() || !facets.surface() || !facets.surface()->prepared() ||
      !facets.surface()->physical() || !facets.surface()->physical()->domain())
    return Fail(S::InvalidInput, "Prepared fixed facets and complete S0 source are required");
  if (!LimitsValid(limits))
    return Fail(S::ResourceLimit, "Active-use limits exceed the fixed hard profile");
  const auto& surface = *facets.surface();
  std::size_t q4 = 0, t3 = 0;
  for (const auto& parent : surface.parents()) {
    if (parent.arity == 4) ++q4;
    else if (parent.arity == 3) ++t3;
    else return Fail(S::IdentityMismatch, "S0 parent arity differs from fixed facets");
  }
  Layout next;
  SelfContactActiveUseCounts counts;
  if (!CountSelfContactActiveUses({q4, t3, surface.vertices().size(),
          surface.edges().size(), facets.config().level}, &counts))
    return Fail(S::ResourceLimit, "Checked fixed-feature count arithmetic overflowed");
  static_cast<SelfContactActiveUseCounts&>(next.forecast) = counts;
  next.forecast.node_roles = surface.physical()->domain()->node_count();
  if (counts.parents != surface.parents().size() ||
      counts.facets != facets.forecast().facets)
    return Fail(S::IdentityMismatch, "Facet forecast differs from S0 topology");
  if (counts.parents > limits.max_parents || counts.facets > limits.max_facets ||
      counts.vertices > limits.max_vertices || counts.edges > limits.max_edges ||
      counts.vertex_uses > limits.max_vertex_uses ||
      counts.edge_uses > limits.max_edge_uses ||
      next.forecast.node_roles > limits.max_nodes)
    return Fail(S::ResourceLimit, "Complete active-use inventory exceeds a declared count cap");

  if (source.rigid) {
    if (!source.rigid->prepared() || !source.rigid->domain() ||
        !source.rigid->domain()->Matches(*surface.physical()->domain()))
      return Fail(S::IdentityMismatch, "Rigid binding does not use the complete S0 domain");
  }
  if (const auto* execution = surface.physical()->execution()) {
    const auto* actual = execution->rigid();
    if (actual && actual->prepared()) {
      if ((!source.rigid && actual->groups().size()) ||
          (source.rigid &&
           (actual->groups().data() != source.rigid->groups().data() ||
            actual->members().data() != source.rigid->members().data())))
        return Fail(S::IdentityMismatch,
            "Supplied rigid source is not S0 execution's actual binding");
    }
  }
  const bool any_cin = source.cin.model || source.cin.ranges || source.cin.witnesses ||
      source.cin.range_count || source.cin.witness_count;
  if (any_cin) {
    if (!source.cin.model || !source.cin.model->prepared() ||
        !source.cin.model->domain() ||
        !source.cin.model->domain()->Matches(*surface.physical()->domain()) ||
        source.cin.range_count != source.cin.model->rows().count)
      return Fail(S::IdentityMismatch, "CIN source does not match S0's complete domain and row roster");
    next.forecast.cin_rows = source.cin.range_count;
    next.forecast.cin_witnesses = source.cin.witness_count;
    // Hard counts precede all borrowed range/witness address and payload reads.
    if (next.forecast.cin_rows > limits.max_cin_rows ||
        next.forecast.cin_witnesses > limits.max_cin_witnesses)
      return Fail(S::ResourceLimit, "Complete CIN roster exceeds active-use caps");
    if (!ValidRange(source.cin.ranges, source.cin.range_count,
            sizeof(*source.cin.ranges)) ||
        !ValidRange(source.cin.witnesses, source.cin.witness_count,
            sizeof(*source.cin.witnesses)))
      return Fail(S::InvalidInput, "CIN borrowed source range is absent or overflows");
  }

  tl::util::BoundedArenaLayout arena(limits.max_host_bytes);
  if (!arena.Append<SelfContactParentUse>(counts.parents, next.parents) ||
      !arena.Append<SelfContactFacetUse>(counts.facets, next.facets) ||
      !arena.Append<SelfContactVertexFeature>(counts.vertices, next.vertices) ||
      !arena.Append<SelfContactEdgeFeature>(counts.edges, next.edges) ||
      !arena.Append<SelfContactFacetVertexUse>(counts.vertex_uses, next.vertex_uses) ||
      !arena.Append<SelfContactFacetEdgeUse>(counts.edge_uses, next.edge_uses) ||
      !arena.Append<NodeRole>(next.forecast.node_roles, next.node_roles) ||
      !arena.Append<tl::constraints::tied_shell::cin::WitnessRange>(
          next.forecast.cin_rows, next.cin_ranges) ||
      !arena.Append<tl::constraints::tied_shell::cin::ActiveWitness>(
          next.forecast.cin_witnesses, next.cin_witnesses))
    return Fail(S::ResourceLimit, "Exact immutable active-use arena exceeds the byte cap");
  next.forecast.arena_bytes = arena.bytes();

  const auto facet_bytes = facets.forecast().owned_payload_bytes;
  if (facet_bytes < sizeof(FixedContactFacetBinding))
    return Fail(S::ResourceLimit, "Retained fixed-facet forecast is invalid");
  next.forecast.retained_facet_bytes = facet_bytes-sizeof(FixedContactFacetBinding);
  if (source.rigid) {
    bool already_retained = false;
    const auto* execution = surface.physical()->execution();
    if (execution && execution->rigid()) {
      const auto& existing = *execution->rigid();
      already_retained = existing.groups().data() == source.rigid->groups().data() &&
          existing.members().data() == source.rigid->members().data();
    }
    const auto rigid_bytes = source.rigid->owned_payload_bytes();
    if (rigid_bytes < sizeof(tl::fea::NodalRigidAssemblyBinding))
      return Fail(S::ResourceLimit, "Retained rigid forecast is invalid");
    if (!already_retained)
      next.forecast.retained_rigid_bytes =
          rigid_bytes-sizeof(tl::fea::NodalRigidAssemblyBinding);
  }
  if (any_cin) {
    const auto cin = source.cin.model->forecast();
    if (cin.model_payload_bytes < sizeof(tl::constraints::tied_shell::TiedCinAttachmentModel))
      return Fail(S::ResourceLimit, "Retained CIN forecast is invalid");
    next.forecast.retained_cin_bytes =
        cin.model_payload_bytes-sizeof(tl::constraints::tied_shell::TiedCinAttachmentModel);
    if (!source.cin.model->domain()->SharesStorage(*surface.physical()->domain())) {
      if (cin.domain_payload_bytes > SIZE_MAX-next.forecast.retained_cin_bytes)
        return Fail(S::ResourceLimit, "Retained CIN domain bytes overflow");
      next.forecast.retained_cin_bytes += cin.domain_payload_bytes;
    }
    if (cin.post_kinchk_payload_bytes > SIZE_MAX-next.forecast.retained_cin_bytes)
      return Fail(S::ResourceLimit, "Retained CIN classification bytes overflow");
    next.forecast.retained_cin_bytes += cin.post_kinchk_payload_bytes;
  }

  tl::util::BoundedArenaLayout budget(limits.max_host_bytes);
  tl::util::ArenaRegion ignored;
  if (!budget.Append<std::byte>(sizeof(SelfContactActiveUseBinding) +
          implementation_bytes + 64, ignored) ||
      !budget.Append<std::byte>(next.forecast.retained_facet_bytes, ignored) ||
      !budget.Append<std::byte>(next.forecast.retained_rigid_bytes, ignored) ||
      !budget.Append<std::byte>(next.forecast.retained_cin_bytes, ignored) ||
      !budget.Append<std::byte>(next.forecast.arena_bytes, ignored))
    return Fail(S::ResourceLimit, "Active-use binding and retained source exceed the byte cap");
  next.forecast.owned_payload_bytes = budget.bytes();
  constexpr std::size_t scratch =
      sizeof(Layout) + sizeof(FixedContactFacet) +
      MaxLocalVertices*sizeof(LocalVertex) + MaxLocalEdges*sizeof(LocalEdge) +
      sizeof(Q4MaterialMeasure) + sizeof(T3MaterialMeasure) +
      12*sizeof(double) + 4*sizeof(std::uint32_t);
  if (!budget.Append<std::byte>(scratch, ignored))
    return Fail(S::ResourceLimit, "Typed active-use startup staging exceeds the byte cap");
  next.forecast.startup_payload_bytes = budget.bytes();
  output = next;
  return {};
}
} // namespace tlfea::contact::active_use
