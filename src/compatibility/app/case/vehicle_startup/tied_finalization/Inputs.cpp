#include "Internal.h"
#include <algorithm>
namespace crash::cases::vehicle_startup::tied_finalization_detail {
native_search::FinalizationInput Inputs::View(const tied::SearchGeometryData& geometry) const {
    return {masters.data(), geometry.secondary_working_nodes.data(), main_nodes.data(), choices.data(),
        geometry.working_positions.size(), masters.size(), choices.size(), main_nodes.size()};
}
Inputs Pack(const tied::Data& declaration, const tied::PackingData& packing,
        const tied::SearchGeometryData& geometry, const native_search::SearchDriverResult& result) {
    using output::Require;
    tied_assessment_detail::CheckResult(declaration, geometry, result);
    Require(packing.master_rows.size() == geometry.masters.size() &&
            geometry.canonical_nodes.size() == geometry.working_positions.size(),
            "Tied finalization geometry extent changed");
    Inputs out;
    out.masters.reserve(geometry.masters.size());
    out.main_nodes.reserve(declaration.master_nodes.size());
    out.choices.reserve(result.rows.size());
    for (std::size_t m = 0; m < geometry.masters.size(); ++m) {
        const auto& master = geometry.masters[m];
        Require(master.declaration_row == packing.master_rows[m], "Tied finalization IRECT identity changed");
        out.masters.push_back(master.working_nodes);
    }
    for (const auto original : declaration.master_nodes) {
        const auto found = std::lower_bound(geometry.canonical_nodes.begin(), geometry.canonical_nodes.end(), original);
        Require(found != geometry.canonical_nodes.end() && *found == original,
                "Tied finalization original MSR node is absent from working geometry");
        out.main_nodes.push_back(static_cast<std::uint32_t>(found-geometry.canonical_nodes.begin()));
    }
    for (std::size_t s = 0; s < result.rows.size(); ++s) {
        const auto working = geometry.secondary_working_nodes[s];
        Require(working < geometry.canonical_nodes.size() &&
                geometry.canonical_nodes[working] == declaration.slave_nodes[s].canonical_index,
                "Tied finalization original NSV association changed");
        out.choices.push_back(result.rows[s].choice);
    }
    return out;
}
}
