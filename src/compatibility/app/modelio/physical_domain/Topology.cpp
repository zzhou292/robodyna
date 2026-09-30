#include "Internal.h"

namespace crash::modelio::physical_domain::detail {
void PrepareTopology(const tl::fea::rigid::NodalRigidPartTopology& original, const Selection& selection,
                     std::size_t cap, tl::fea::rigid::NodalRigidPartTopology& output) {
    namespace native = tl::fea::rigid;
    std::vector<native::PartTopologyPartInput> parts;
    std::vector<native::PartTopologyExtraInput> extras;
    std::vector<std::uint64_t> other;
    parts.reserve(original.part_count());
    extras.reserve(original.extra_count());
    other.reserve(selection.counts.plain_members);
    for (std::size_t p = 0; p < original.part_count(); ++p) {
        const auto& part = original.parts()[p];
        parts.push_back({part.source_part_id, original.original_members() + part.member_offset, part.member_count});
    }
    for (std::size_t e = 0; e < original.extra_count(); ++e) {
        const auto& extra = original.extras()[e];
        extras.push_back({original.parts()[extra.part_index].source_part_id, extra.source_node_set_id,
            original.original_members() + extra.member_offset, extra.member_count});
    }
    for (const auto& group : selection.groups) other.insert(other.end(), group.members.begin(), group.members.end());
    native::PartTopologyInput input;
    input.source_instance_id = original.source_instance_id();
    input.parts = parts.data(); input.part_count = parts.size();
    input.extras = extras.data(); input.extra_count = extras.size();
    input.merges = original.merges(); input.merge_count = original.merge_count();
    input.expected_members = original.expected_members(); input.expected_member_count = original.member_count();
    input.other_rigid_members = other.data(); input.other_rigid_member_count = other.size();
    input.limits.max_host_bytes = cap;
    const auto report = output.Initialize(input);
    output::Require(bool(report), report.message);
}
} // namespace crash::modelio::physical_domain::detail
