#pragma once
#include "../Internal.h"
#include "../../tied_search/tests/Fixture.h"
namespace crash::cases::vehicle_startup::finalization_test {
namespace detail = tied_finalization_detail;
struct Fixture : test::Fixture {
    native_search::SearchDriverResult result;
    Fixture() {
        // Add independent keyword census to the existing tiny authenticated
        // block fixture. Original-source tests consume the real compiler census.
        AlterCanonical([](auto& doc) {
            for (auto& file : doc["source_files"].GetObject()) {
                output::Value counts(rapidjson::kObjectType);
                for (const auto& row : file.value["blocks"].GetArray()) {
                    const auto* keyword = row["keyword"].GetString();
                    if (!counts.HasMember(keyword)) {
                        output::Value name(keyword, doc.GetAllocator());
                        output::Value zero(0u);
                        counts.AddMember(name, zero, doc.GetAllocator());
                    }
                    counts[keyword].SetUint(counts[keyword].GetUint()+1);
                }
                file.value.AddMember("keyword_counts", counts, doc.GetAllocator());
            }
        });
        result.rows.resize(declaration.slave_nodes.size());
        for (std::size_t s = 0; s < result.rows.size(); ++s) {
            auto& row = result.rows[s];
            row.choice.matched = true;
            row.choice.ordered_master = s%2+1;
            row.choice.projection.s = -0.0;
            row.choice.projection.t = .1;
            row.choice.projection.selection_distance = .3+s;
            row.force_patch_status = native_search::Status::SingularPatch;
        }
        result.matched_count = result.rows.size();
        result.singular_patch_count = result.rows.size();
    }
    void AlterCanonical(const std::function<void(output::Document&)>& change) {
        output::Document doc;
        doc.Parse(canonical.canonical_bytes.c_str());
        change(doc);
        canonical.canonical_bytes = tied::test::Json(doc);
    }
    TiedFinalizationReceipt Receipt(TiedFinalizationLimits limits = {}) const {
        return detail::Receipt(canonical, declaration, packing, limits);
    }
    TiedFinalizationForecast Forecast(TiedFinalizationLimits limits = {}) const {
        return detail::Preflight(canonical, declaration, packing, geometry, result, limits);
    }
    detail::Inputs Inputs() const { return detail::Pack(declaration, packing, geometry, result); }
};
}
