// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "../Type13Math.h"
#include "../../../solvers/NodalNativePhysicalCoefficients.h"
#include <limits>

namespace tl::fea::type13::batch_detail {
BatchReport SourceGeometryPreflight(const BatchConfig& config,
                            const Type13NodeContributions& source, bool physical_constrained) noexcept {
  const auto& owner = config.owner;
  if (!owner.owner_id || owner.epoch || owner.time != 0 ||
      owner.velocity_time != 0 || !owner.has_rotations ||
      owner.temporal_scheme != NodalTemporalScheme::StaggeredHalfKickStart ||
      owner.velocity_phase != NodalVelocityPhase::Collocated ||
      !detail::Positive(owner.fixed_dt) || owner.reactions_valid ||
      owner.reaction_base_epoch || owner.reaction_time != 0 ||
      owner.reaction_kick_dt != 0 || !config.configuration_id ||
      !config.qualification_id || !shell_startup_detail::ValidStartup(config.startup, true, physical_constrained) ||
      (config.assembly != BatchAssembly::OrdinaryForces &&
       config.assembly != BatchAssembly::CinNativeStiffness)) {
    return {BatchStatus::InvalidInput, "TYPE13 requires explicit fresh staggered startup"};
  }
  if (!source.prepared() || !source.model() || !source.domain() ||
      !source.Matches(*source.model(), *source.domain()) ||
      source.domain()->node_count() != owner.node_count ||
      source.model()->global_node_count() != owner.node_count ||
      source.record_count() != 2 * source.model()->connection_count()) {
    return {BatchStatus::InvalidInput, "TYPE13 endpoint model/domain extent differs from owner"};
  }
  return {};
}

BatchReport SourcePreflight(const BatchConfig& config,
                            const Type13NodeContributions& source) noexcept {
  if (!native_physical_coefficients::ValidScope(config.owner.rigid_groups,
                                               config.owner.node_count)) {
    return {BatchStatus::InvalidInput, "TYPE13 requires explicit fresh staggered startup"};
  }
  const auto checked = SourceGeometryPreflight(config, source);
  if (!checked) return checked;
  if (!native_physical_coefficients::Empty(config.owner.rigid_groups) &&
      config.owner.rigid_groups.source_instance_id != source.model()->source_instance_id()) {
    return {BatchStatus::InvalidInput, "TYPE13 endpoint model/domain extent differs from owner"};
  }
  return {};
}
BatchReport BuildStartup(const BatchConfig& config,
    const Type13NodeContributions& source, util::HostArena& arena,
    const ArenaLayout& layout, Storage& header, BatchDiagnostics& diagnostics) {
  const auto checked = SourcePreflight(config, source);
  if (!checked) return checked;
  return BuildSourceValues(config, source, arena, layout, header, diagnostics);
}

BatchReport BuildSourceValues(const BatchConfig& config,
                          const Type13NodeContributions& source,
                          util::HostArena& arena, const ArenaLayout& layout,
                          Storage& header, BatchDiagnostics& diagnostics) {
  auto* storage = arena.Construct<Storage>(layout.header);
  if (!storage || !arena.Construct<Property>(layout.properties) ||
      !arena.Construct<DeviceElement>(layout.elements) ||
      !arena.Construct<Evaluation>(layout.slab[0]) ||
      !arena.Construct<Evaluation>(layout.slab[1]) ||
      !arena.Construct<Status>(layout.status) || !mapped_connector::Construct(arena, layout.assembly)) {
    return {BatchStatus::ResourceLimit, "TYPE13 arena cannot construct admitted records"};
  }
  const auto& model = *source.model();
  auto next = RebasedHeader(arena.data(), layout);
  next.model.config = config;
  next.model.units = model.units();
  next.model.source_instance_id = model.source_instance_id();
  next.model.element_count = model.connection_count();
  for (std::size_t p = 0; p < model.property_count(); ++p) {
    next.model.properties[p] = *model.property(p);
  }
  BatchDiagnostics d;
  d.source_instance_id = model.source_instance_id();
  d.owner_id = config.owner.owner_id;
  d.configuration_id = config.configuration_id;
  d.qualification_id = config.qualification_id;
  d.phase = BatchPhase::Accepted;
  d.element_count = model.connection_count();
  d.minimum_native_dt_s = std::numeric_limits<double>::max();
  const auto units = model.units();
  const auto velocity = tl::math::fixed3::Scale(config.startup.uniform_velocity,
                                               units.time_to_s / units.length_to_m);
  for (std::size_t e = 0; e < model.connection_count(); ++e) {
    const auto& connection = model.connections()[e];
    auto& element = next.model.elements[e];
    element.reference = model.startup(e)->reference;
    element.property = connection.property;
    NativeEndpointKinematics nodes[2];
    for (unsigned local = 0; local < 2; ++local) {
      const auto& record = source.records()[2 * e + local];
      element.nodes[local] = record.value.global_node;
      element.original_position_native[local] =
          model.nodes()[connection.node[local]].position_native;
      nodes[local] = {element.original_position_native[local], velocity, {}};
    }
    Evaluation value;
    const auto status = InitializeForce(next.model.properties[element.property],
                                        element.reference, nodes, value);
    if (status != Status::Success) {
      return {BatchStatus::ElementFailure, "TYPE13 native TT0 initialization rejected",
              e, SIZE_MAX, status};
    }
    next.slab[0][e] = value;
    next.slab[1][e] = value;
    d.active_count += value.native_history.active;
    d.newly_failed_count += value.newly_failed;
    for (unsigned k = 0; k < ChannelCount; ++k) {
      d.internal_work_J[k] += value.signed_work_J[k];
      if (!tl::math::Finite(d.internal_work_J[k])) {
        return {BatchStatus::NonfiniteResult, "TYPE13 initial work reduction overflow", e};
      }
    }
    d.minimum_native_dt_s = ::fmin(d.minimum_native_dt_s, value.stability.critical_dt_s);
  }
  if (!mapped_connector::Build(next.model.elements, next.model.element_count,
          config.owner.node_count, layout.assembly, next.assembly))
    return {BatchStatus::InvalidInput, "TYPE13 mapped incidence differs from source endpoints"};
  d.valid = true;
  next.control.diagnostics = d;
  *storage = next;
  header = next;
  diagnostics = d;
  return {};
}
} // namespace tl::fea::type13::batch_detail
