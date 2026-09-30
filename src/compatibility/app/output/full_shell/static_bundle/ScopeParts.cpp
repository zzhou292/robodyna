#include "InputChecks.h"
#include <algorithm>

namespace crash::output::full_shell::source::detail {
void CheckScopeParts(const CanonicalData& d, const Value& rows) {
    using namespace array_json;
    Require(rows.IsArray() && rows.Size() == d.parts.size(), "Scope source part inventory differs");
    std::vector<std::uint64_t> ids;
    ids.reserve(rows.Size());
    for (const auto& row : rows.GetArray()) {
        const auto id = UInt(Field(row, "source_part_id"));
        const auto& p = FindPart(d, id);
        const auto& raw = Field(row, "raw_fields");
        const auto& retained = Field(row, "retained_shell_part");
        Require(UInt(Field(row, "source_material_id")) == p.material &&
            UInt(Field(row, "source_section_id")) == p.section && raw.IsArray() && raw.Size() == 8 &&
            UInt(raw[0]) == p.part && UInt(raw[1]) == p.section && UInt(raw[2]) == p.material &&
            retained.IsBool() && retained.GetBool() ==
                std::binary_search(d.selected_parts.begin(), d.selected_parts.end(), id),
            "Scope source PID/MID/SECID/selection differs from canonical declarations");
        ids.push_back(id);
    }
    std::sort(ids.begin(), ids.end());
    Require(std::adjacent_find(ids.begin(), ids.end()) == ids.end(), "Duplicate scope source part");
}
} // namespace crash::output::full_shell::source::detail
