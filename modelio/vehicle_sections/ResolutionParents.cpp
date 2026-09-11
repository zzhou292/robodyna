#include "Internal.h"

namespace crash::modelio::vehicle::resolution {
std::vector<SectionParentResolution> ReadParents(const VehicleSourcePlan& plan) {
    const auto& array = source::FindArray(plan.canonical().data(), "shells_records");
    const auto records = output::arrays::Decode<std::uint64_t>(array.descriptor, array.bytes);
    std::vector<SectionParentResolution> result;
    result.reserve(plan.parents().size());
    std::uint32_t q4 = 0, t3 = 0;
    for (const auto& parent : plan.parents()) {
        const auto offset = 6 * std::size_t(parent.canonical_parent);
        Require(offset <= records.size() && records.size() - offset >= 6 &&
                parent.part_index < plan.parts().size() &&
                records[offset + 1] == plan.parts()[parent.part_index].part_id,
                "Original parent source association changed");
        const bool quad = records[offset + 4] != records[offset + 5];
        result.push_back({records[offset], parent.canonical_parent, parent.part_index,
                          quad ? q4++ : t3++, quad ? SourceShellTopology::Q4 : SourceShellTopology::T3});
    }
    Require(q4 == plan.counts().q4 && t3 == plan.counts().t3, "Complete source topology count changed");
    return result;
}
} // namespace crash::modelio::vehicle::resolution
