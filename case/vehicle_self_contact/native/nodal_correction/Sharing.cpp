#include "Internal.h"
#include <algorithm>
#include <set>

namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail {
std::vector<PartControl> ResolveSharing(const std::vector<Part>& parts,
    const std::vector<Section>& sections, const std::vector<std::uint64_t>& requested) {
    std::map<std::uint64_t, const Part*> by_part;
    std::map<std::uint64_t, const Section*> by_section;
    std::map<std::uint64_t, std::set<std::uint64_t>> materials;
    for (const auto& section : sections) {
        if (!section.id || !by_section.emplace(section.id, &section).second)
            Reject(Status::InvalidInput, "Duplicate or invalid source section identity");
    }
    for (const auto& part : parts) {
        if (!part.id || !part.material || !by_section.count(part.section) ||
            !by_part.emplace(part.id, &part).second)
            Reject(Status::InvalidInput, "Part/property source identity is missing or duplicated", part.member, part.line);
        materials[part.section].insert(part.material);
    }
    std::set<std::uint64_t> direct, controlled_sections;
    for (const auto id : requested) {
        const auto found = by_part.find(id);
        if (found == by_part.end()) Reject(Status::InvalidInput, "CONTACT_INTERIOR requested part is absent");
        const auto& part = *found->second;
        if (by_section.at(part.section)->keyword != "*SECTION_SOLID")
            Reject(Status::UnsupportedSource, "Controlled property is not an admitted ordinary solid section", part.member, part.line);
        if (materials.at(part.section).size() != 1)
            Reject(Status::NeedsNativePropertyMapping,
                "Controlled multi-MID section requires its native property-cloning map", part.member, part.line);
        direct.insert(id);
        controlled_sections.insert(part.section);
    }
    std::vector<PartControl> result;
    result.reserve(parts.size());
    for (const auto& [id, value] : by_part) {
        const auto& part = *value;
        const bool single_material = materials.at(part.section).size() == 1;
        // Source converter reuses a native property by SID only in its
        // single-MID branch. Other unrequested cloning remains unavailable,
        // but the closed source profile proves no control was assigned there.
        result.push_back({id, part.section, part.material,
            single_material ? part.section : 0, direct.count(id) != 0,
            controlled_sections.count(part.section) != 0});
    }
    return result;
}
} // namespace crash::cases::vehicle_self_contact::native::nodal_correction::detail
