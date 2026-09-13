#include "Internal.h"

#include "output/BoundedArrayIO.h"
#include <algorithm>
#include <map>

namespace crash::modelio::self_contact::detail {
namespace {

std::map<SourceId, std::size_t> Count(
    const source::CanonicalData& canonical, const char* name,
    std::size_t columns) {
    const auto& array = source::FindArray(canonical, name);
    const auto values = output::arrays::Decode<SourceId>(
        array.descriptor, array.bytes);
    output::Require(values.size() == columns * array.descriptor.layout.rows,
        "Original self-contact canonical element array extent differs");
    std::map<SourceId, std::size_t> counts;
    for (std::size_t row = 0; row < array.descriptor.layout.rows; ++row) {
        output::Require(values[columns * row] &&
                values[columns * row + 1],
            "Original self-contact canonical element identity is zero");
        ++counts[values[columns * row + 1]];
    }
    return counts;
}

std::size_t Value(const std::map<SourceId, std::size_t>& counts,
    SourceId id) {
    const auto found = counts.find(id);
    return found == counts.end() ? 0 : found->second;
}

bool Contains(const std::vector<SourceId>& values, SourceId id) {
    return std::binary_search(values.begin(), values.end(), id);
}

}  // namespace

void BuildCensus(const source::CanonicalData& canonical,
    Draft& draft, Limits limits) {
    const auto shells = Count(canonical, "shells_records", 6);
    const auto solids = Count(canonical, "solids_records", 10);
    const auto beams = Count(canonical, "beams_records", 10);
    auto& data = draft.data;
    data.parts.reserve(data.selected_part_ids.size());
    for (const auto id : data.selected_part_ids) {
        output::Require(data.parts.size() < limits.parts,
            "Original self-contact part disposition exceeds cap");
        const auto& declaration = source::FindPart(canonical, id);
        PartDisposition part;
        part.part_id = id;
        part.material_id = declaration.material;
        part.section_id = declaration.section;
        part.shell_section = declaration.shell_section;
        part.shells = Value(shells, id);
        part.solids = Value(solids, id);
        part.beams = Value(beams, id);
        part.retained_shell_part = Contains(canonical.selected_parts, id);
        part.excluded_shell_part = Contains(canonical.excluded_parts, id);
        output::Require(!(part.retained_shell_part &&
                part.excluded_shell_part) &&
                (!part.shells ||
                    part.retained_shell_part || part.excluded_shell_part) &&
                (!part.shells || part.shell_section),
            "Original self-contact shell selection/disposition differs");

        ++data.counts.selected_parts;
        data.counts.shell_parts += part.shells != 0;
        data.counts.retained_shell_parts += part.retained_shell_part;
        data.counts.excluded_shell_parts += part.excluded_shell_part;
        data.counts.non_shell_parts += !part.shell_section;
        data.counts.shells += part.shells;
        data.counts.retained_shells +=
            part.retained_shell_part ? part.shells : 0;
        data.counts.excluded_shells +=
            part.excluded_shell_part ? part.shells : 0;
        data.counts.solids += part.solids;
        data.counts.beams += part.beams;
        data.parts.push_back(part);
    }
    output::Require(data.counts.selected_parts ==
            data.selected_part_ids.size() &&
            data.counts.shells ==
                data.counts.retained_shells + data.counts.excluded_shells,
        "Original self-contact census partition differs");
}

}  // namespace crash::modelio::self_contact::detail
