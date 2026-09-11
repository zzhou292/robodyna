// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"

namespace tl::fea::shell_execution_detail {
Report Forecast(const ShellBatchPlasticityBinding& catalog, const NodalCoefficientLedger& ledger,
    const NodalRigidAssemblyBinding& rigid, ShellExecutionLimits limits,
    std::size_t header, Layout& output) noexcept {
  const auto hard = ShellExecutionLimits::Vehicle();
  if (!limits.max_parents || limits.max_parents > hard.max_parents ||
      !limits.max_nodes || limits.max_nodes > hard.max_nodes ||
      !limits.max_host_bytes || limits.max_host_bytes > hard.max_host_bytes) {
    return Error(Status::ResourceLimit, "Shell execution limits exceed explicit host scope");
  }
  if (!catalog.execution_sections() || !ledger.prepared() || !rigid.prepared() ||
      !ledger.shells() || !ledger.domain()) {
    return Error(Status::InvalidInput, "Explicit execution catalog and complete physical sources are required");
  }
  if (catalog.parent_count() > limits.max_parents || ledger.domain()->node_count() > limits.max_nodes) {
    return Error(Status::ResourceLimit, "Shell execution counts exceed host scope");
  }
  if (!catalog.Matches(*ledger.shells()->shells()) || !rigid.coefficients()->Matches(ledger)) {
    return Error(Status::IdentityMismatch, "Shell execution requires the exact catalog and PART coefficient ledger");
  }
  const auto& shells = *ledger.shells()->shells();
  const auto& topology = *rigid.parts()->topology();
  const auto catalog_bytes = catalog.host_bytes();
  const auto rigid_bytes = rigid.owned_payload_bytes();
  if (catalog_bytes < sizeof(catalog) || rigid_bytes < sizeof(rigid)) {
    return Error(Status::ResourceLimit, "Invalid retained execution source accounting");
  }
  Layout next;
  util::BoundedArenaLayout arena(limits.max_host_bytes), owned(limits.max_host_bytes);
  util::BoundedArenaLayout startup(limits.max_host_bytes);
  util::ArenaRegion ignored;
  // The rigid handle retains the ledger/domain/shell backing once. Catalog
  // inventory backing is conservatively charged again; no size-based alias guess.
  if (!arena.Append<ShellExecutionParent>(catalog.parent_count(), next.parents) ||
      !arena.Append<std::size_t>(shells.qeph_count(), next.qeph) ||
      !arena.Append<std::size_t>(shells.t3_count(), next.t3) ||
      !arena.Append<std::size_t>(shells.qbat_count(), next.qbat) ||
      !owned.Append<unsigned char>(header, ignored) ||
      !owned.Append<unsigned char>(catalog_bytes - sizeof(catalog), ignored) ||
      !owned.Append<unsigned char>(rigid_bytes - sizeof(rigid), ignored) ||
      !owned.Append<unsigned char>(arena.bytes(), ignored) ||
      !startup.Append<unsigned char>(owned.bytes(), ignored) ||
      !startup.Append<unsigned char>(Index::Bytes(topology.part_count()), ignored) ||
      !startup.Append<unsigned char>(Index::Bytes(topology.member_count()), ignored)) {
    return Error(Status::ResourceLimit, "Complete shell execution sources and startup indexes exceed byte cap");
  }
  next.arena_bytes = arena.bytes();
  next.forecast = {owned.bytes(), startup.bytes()};
  output = next;
  return {};
}
} // namespace tl::fea::shell_execution_detail
