// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"

namespace tl::fea::shell_execution_detail {
namespace {
template<std::size_t N>
bool OriginalMembers(const std::array<std::size_t, N>& nodes, const ShellNodeMap& map,
    const rigid::NodalRigidPartAssemblyModel& parts, const rigid::PartTopologyPart& part,
    const Index& members) noexcept {
  const auto& shells = *map.shells();
  for (const auto local : nodes) {
    const auto nid = shells.nodes()[local].source_id;
    const auto original = members.First(nid);
    if (original < part.member_offset || original - part.member_offset >= part.member_count ||
        parts.RootForDomainNode(map.owner_index(local)) != part.root_index) return false;
  }
  return true;
}
} // namespace

Report Bind(Storage& output) {
  const auto& catalog = output.catalog;
  const auto& parts = *output.rigid.parts();
  const auto& topology = *parts.topology();
  const auto& map = *output.rigid.coefficients()->shells();
  const auto& shells = *map.shells();
  Index pids, members;
  pids.Prepare(topology.part_count(), [&](std::size_t i) { return topology.parts()[i].source_part_id; });
  members.Prepare(topology.member_count(), [&](std::size_t i) { return topology.original_members()[i]; });
  for (std::size_t i = 0; i < catalog.parent_count(); ++i) {
    const auto& source = *catalog.parent(i);
    ShellExecutionParent row;
    row.source = source;
    if (!catalog.Law(source.family, source.family_index, &row.law) ||
        !catalog.MaterialPointCount(source.family, source.family_index, &row.material_points)) {
      return Error(Status::InvalidParent, "Missing complete execution catalog role", i, source.family);
    }
    const auto part_index = pids.First(source.source_part_id);
    if (row.law == ShellSectionLaw::RigidSkin) {
      if (part_index == SIZE_MAX) {
        return Error(Status::IdentityMismatch, "Rigid skin PID has no original prepared PART", i, source.family);
      }
      const auto& part = topology.parts()[part_index];
      const bool member_match = source.family == ShellBindingFamily::Qeph ?
          OriginalMembers(shells.qeph_nodes(source.family_index), map, parts, part, members) :
          source.family == ShellBindingFamily::T3 &&
          OriginalMembers(shells.t3_nodes(source.family_index), map, parts, part, members);
      const auto& group = output.rigid.groups()[part.root_index];
      const auto primary_pid = topology.parts()[topology.roots()[part.root_index].part_index].source_part_id;
      if (!member_match || group.source_kind != RigidBindingSourceKind::Part || group.source_id != primary_pid) {
        return Error(Status::IdentityMismatch, "Rigid skin nodes differ from original PART and merged primary membership", i, source.family);
      }
      row.part_index = part_index;
      row.root_index = part.root_index;
      ++output.counts.rigid_skin;
    } else {
      if (part_index != SIZE_MAX) {
        return Error(Status::IdentityMismatch, "Original PART PID cannot be disguised as a constitutive shell", i, source.family);
      }
      ++output.counts.constitutive;
    }
    output.counts.material_points += row.material_points;
    output.parents[i] = row;
    switch (source.family) {
      case ShellBindingFamily::Qeph: output.qeph[source.family_index] = i; break;
      case ShellBindingFamily::T3: output.t3[source.family_index] = i; break;
      case ShellBindingFamily::Qbat: output.qbat[source.family_index] = i; break;
      default: return Error(Status::InvalidParent, "Unknown execution family", i, source.family);
    }
  }
  return {};
}
} // namespace tl::fea::shell_execution_detail
