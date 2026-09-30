// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TiedCinAttachmentInternal.h"

namespace tl::constraints::tied_shell::cin_detail {
Report Map(const CinAttachmentDeclaration& in,const PostKinChkSlave& classified,
        const fea::NodalNodeDomain& domain,std::size_t row,CinAttachmentRow& out) noexcept {
  if (!in.secondary_source_id || in.secondary_source_id != classified.before.source_id ||
      !in.ordered_master_rank || in.master_source.kind != CinMasterSourceKind::DeclaredShellElement ||
      !in.master_source.element_id || !in.master_source.part_id)
    return Fail(ResultStatus::SourceMismatch,row);
  if (classified.before.irupt != 0 || classified.kinet != classified.before.kinematics.conditions ||
      classified.before.kinematics.translation != 7 || classified.before.kinematics.rotation != 7 ||
      classified.repeated_condition || classified.mixed_incompatible_conditions)
    return Fail(ResultStatus::UnsupportedDisposition,row);
  if (in.topology != CinMasterTopology::Quad && in.topology != CinMasterTopology::TriangleRepeatedThird)
    return Fail(ResultStatus::InvalidTopology,row);
  for (std::size_t a = 0; a < 4; ++a) {
    if (!in.master_source_ids[a] || in.master_source_ids[a] == in.secondary_source_id)
      return Fail(ResultStatus::InvalidTopology,row,a);
    for (std::size_t b = 0; b < a; ++b) {
      const bool repeated = in.topology == CinMasterTopology::TriangleRepeatedThird && a == 3 && b == 2;
      if ((in.master_source_ids[a] == in.master_source_ids[b]) != repeated)
        return Fail(ResultStatus::InvalidTopology,row,a);
    }
  }
  PatchInput patch;
  for (std::size_t slot = 0; slot < 5; ++slot) {
    const auto id = slot ? in.master_source_ids[slot-1] : in.secondary_source_id;
    const auto found = domain.Find(id);
    if (found == SIZE_MAX || found > UINT32_MAX) return Fail(ResultStatus::MissingDomainNode,row,slot);
    const auto position = domain.nodes()[found].position;
    if (!fea::nodal_domain_detail::SamePosition(position,in.reference_positions[slot]))
      return Fail(ResultStatus::PositionMismatch,row,slot);
    if (slot) {
      out.master_domain_nodes[slot-1] = static_cast<std::uint32_t>(found);
      patch.master_position[slot-1] = position;
    } else {
      out.secondary_domain_node = static_cast<std::uint32_t>(found);
      patch.secondary_position = position;
    }
  }
  const auto status = PreparePatch(patch,out.reference_patch);
  if (status != Status::Success) return {ResultStatus::ReferencePatchRejected,row,SIZE_MAX,status};
  out.original_nsv_row = in.original_nsv_row;
  out.ordered_master_rank = in.ordered_master_rank;
  out.master_source = in.master_source;
  out.topology = in.topology;
  return {};
}
bool SameMaster(const CinAttachmentRow& a,const CinAttachmentRow& b) noexcept {
  return a.master_source.kind == b.master_source.kind &&
         a.master_source.element_id == b.master_source.element_id && a.master_source.part_id == b.master_source.part_id &&
         a.topology == b.topology && a.master_domain_nodes == b.master_domain_nodes;
}
}
