#include "Internal.h"
namespace crash::cases::vehicle_startup::tied_assessment_detail {
std::size_t SelectedDeclarationRow(const tied::SearchGeometryData& g,
        const native_search::SearchDriverResult& result, std::size_t s) {
    const auto& choice = result.rows.at(s).choice;
    if (!choice.matched) return SIZE_MAX;
    output::Require(choice.ordered_master && choice.ordered_master <= g.masters.size(),
                    "Tied assessment selected rank is invalid");
    return g.masters.at(choice.ordered_master-1).declaration_row;
}
void CheckResult(const tied::Data& declaration, const tied::SearchGeometryData& g,
        const native_search::SearchDriverResult& result) {
    output::Require(result.rows.size() == declaration.slave_nodes.size(), "Incomplete tied assessment NSV rows");
    std::size_t matched = 0, singular = 0;
    for (std::size_t s = 0; s < result.rows.size(); ++s) {
        const auto row = SelectedDeclarationRow(g, result, s);
        if (row == SIZE_MAX) continue;
        output::Require(row < declaration.masters.size(), "Tied assessment lost declared master");
        ++matched;
        const auto& value = result.rows[s];
        singular += value.force_patch_status == native_search::Status::SingularPatch;
        output::Require(value.force_patch.prepared() == (value.force_patch_status == native_search::Status::Success),
                        "Tied assessment patch readiness mismatch");
    }
    output::Require(matched == result.matched_count && singular == result.singular_patch_count,
                    "Tied assessment reduction count mismatch");
}
} // namespace crash::cases::vehicle_startup::tied_assessment_detail
