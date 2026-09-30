#include "Internal.h"
#include "MaterialSlots.h"
#include <algorithm>

namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail {
void PackControls(const seed::PreCorrectionNodalSource& input, const Context& context,
    std::vector<c::Solid>& solids, std::vector<std::uint64_t>& source_ids) {
    const auto& source = input.physical().source_domain().source().solid_source().data();
    const auto nodes = input.physical().source_domain().domain().node_count();
    std::vector<unsigned char> seen(source.rows.size(), 0);
    solids.reserve(source.rows.size());
    source_ids.reserve(source.rows.size());
    for (const auto& contributor : input.contributors()) {
        if (contributor.channel != seed::Channel::Volume) continue;
        if (contributor.source_index >= source.rows.size() || seen[contributor.source_index] || contributor.slots != 8)
            Reject(Status::InvalidInput, "Incomplete or repeated controlled solid source coverage");
        const auto& row = source.rows[contributor.source_index];
        if (row.element_id != contributor.original_id || row.part_id != contributor.part_id ||
            row.part_index >= source.parts.size())
            Reject(Status::InvalidInput, "Controlled solid source identity differs from pre-correction");
        const auto& material = source.parts[row.part_index];
        const auto part = std::lower_bound(context.parts.begin(), context.parts.end(), row.part_id,
            [](const auto& p, auto id) { return p.part_id < id; });
        if (part == context.parts.end() || part->part_id != row.part_id ||
            part->section_id != material.section_id || part->material_id != material.material_id)
            Reject(Status::InvalidInput, "Controlled material/property association differs from physical source");
        seen[contributor.source_index] = 1;
        c::Solid output;
        output.control = part->effective_control ? 1 : 0;
        for (unsigned slot = 0; slot < 8; ++slot) {
            if (contributor.nodes[slot] >= nodes) Reject(Status::InvalidInput, "Controlled raw solid node is outside physical domain");
            output.nodes[slot] = contributor.nodes[slot];
        }
        if (output.control == 1) {
            if (!part->native_property_id)
                Reject(Status::NeedsNativePropertyMapping, "Effective controlled property has no native identity");
            const auto slots = Slots(material, input.provenance().units);
            output.bulk = slots.bulk;
            output.controlled_bulk = slots.controlled_bulk;
        }
        // Disabled material channels stay API-zero and are unconsumed by
        // correction/certificate. Their real source rows/nodes remain present.
        solids.push_back(output);
        source_ids.push_back(row.element_id);
    }
    if (solids.size() != source.rows.size() || std::find(seen.begin(), seen.end(), 0) != seen.end())
        Reject(Status::InvalidInput, "Controlled source does not cover every physical solid");
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail
