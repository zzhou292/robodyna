// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "lib_utils/SourceIdentityIndex.h"
#include <cmath>

namespace tlfea::contact::self_contact {
namespace fe = tl::fea;
SelfContactSurfaceReport ReadParent(const fe::ShellPhysicalBinding& physical,
    const SelfContactParentSelection& selected, std::size_t ordinal,
    SelfContactSurfaceParent& out) noexcept {
  using S = SelfContactSurfaceStatus;
  const auto* source = physical.catalog()->parent(selected.catalog_row);
  if (!source || !selected.source_parent_id || !selected.source_part_id ||
      source->family != selected.family || source->family_index != selected.family_index ||
      source->source_parent_id != selected.source_parent_id || source->source_part_id != selected.source_part_id)
    return {S::IdentityMismatch, ordinal, "Selected catalog row, family, index, EID or PID differs"};
  SelfContactSurfaceParent next;
  next.source = *source;
  if (!physical.catalog()->Law(source->family, source->family_index, &next.law) ||
      !physical.catalog()->MaterialPointCount(source->family, source->family_index, &next.material_points))
    return {S::IdentityMismatch, ordinal, "Selected execution role is unavailable"};
  const auto& shells = *physical.shells();
  const auto index = source->family_index;
  double thickness = 0;
  auto placement = fe::ShellReferencePlacement::Centered;
  const std::size_t* local_nodes = nullptr;
  std::uint64_t native_id = 0;
  if (source->family == fe::ShellBindingFamily::T3 && index < shells.t3_count()) {
    const auto& reference = shells.t3_reference(index);
    thickness = reference.input.thickness;
    placement = reference.input.placement;
    local_nodes = shells.t3_nodes(index).data();
    native_id = shells.t3_source_id(index);
    next.arity = 3;
    next.t3.feature_id = next.t3.parent_element_id = native_id;
    next.t3.interpolation = SurfaceInterpolation::kLinearTriangle;
  } else if (source->family == fe::ShellBindingFamily::Qeph && index < shells.qeph_count()) {
    const auto& reference = shells.qeph_reference(index);
    thickness = reference.input.thickness;
    placement = reference.input.placement;
    local_nodes = shells.qeph_nodes(index).data();
    native_id = shells.qeph_source_id(index);
    next.arity = 4;
  } else if (source->family == fe::ShellBindingFamily::Qbat && index < shells.qbat_count()) {
    const auto& reference = shells.qbat_reference(index);
    thickness = reference.quadrilateral().input.thickness;
    placement = reference.quadrilateral().input.placement;
    local_nodes = shells.qbat_nodes(index).data();
    native_id = shells.qbat_source_id(index);
    next.arity = 4;
    if (reference.input().options.offset_ratio != 0)
      return {S::UnsupportedReferencePlane, ordinal, "QBAT offset requires a qualified offset surface"};
  } else {
    return {S::IdentityMismatch, ordinal, "Selected native family reference is unavailable"};
  }
  if (native_id != source->source_parent_id)
    return {S::IdentityMismatch, ordinal, "Selected EID differs from native reference"};
  if (placement != fe::ShellReferencePlacement::Centered)
    return {S::UnsupportedReferencePlane, ordinal, "Only centered reference midsurfaces are admitted"};
  if (!std::isfinite(thickness) || thickness <= 0 || !std::isfinite(.5 * thickness) || .5 * thickness <= 0)
    return {S::InvalidInput, ordinal, "Positive represented reference half-thickness required"};
  next.reference_half_thickness_m = .5 * thickness;
  if (next.arity == 4) next.q4.feature_id = next.q4.parent_element_id = native_id;
  for (unsigned local = 0; local < next.arity; ++local) {
    const auto node = physical.mapping()->owner_index(local_nodes[local]);
    if (node >= physical.domain()->node_count() || node > UINT32_MAX)
      return {S::IdentityMismatch, ordinal, "Selected node is outside the exact physical domain"};
    if (next.arity == 4) next.q4.nodes[local] = static_cast<std::uint32_t>(node);
    else next.t3.nodes[local] = static_cast<std::uint32_t>(node);
  }
  out = next;
  return {};
}
SelfContactSurfaceReport ValidateSources(const fe::ShellPhysicalBinding& physical,
    const SelfContactSurfaceInput& input) {
  // Report source discrepancies in selection order before constructing an index.
  for (std::size_t i = 0; i < input.parent_count; ++i) {
    SelfContactSurfaceParent parent;
    const auto report = ReadParent(physical, input.parents[i], i, parent);
    if (report.status != SelfContactSurfaceStatus::Ok) return report;
  }
  tl::util::SourceIdentityIndex<0> index;
  index.Prepare(input.parent_count, [&](std::size_t i) { return input.parents[i].source_parent_id; });
  for (std::size_t i = 0; i < input.parent_count; ++i)
    if (index.First(input.parents[i].source_parent_id) != i)
      return {SelfContactSurfaceStatus::DuplicateParent, i, "Selected source parent occurs twice"};
  return {};
}
} // namespace tlfea::contact::self_contact
