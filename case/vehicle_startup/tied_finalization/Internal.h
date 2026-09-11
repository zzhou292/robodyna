#pragma once
#include "../TiedSearchFinalized.h"
#include "../tied_search/Internal.h"
namespace crash::cases::vehicle_startup::tied_finalization_detail {
struct Inputs {
    std::vector<std::array<std::uint32_t,4>> masters;
    std::vector<std::uint32_t> main_nodes;
    std::vector<native_search::SearchChoice> choices;
    native_search::FinalizationInput View(const tied::SearchGeometryData&) const;
};
TiedFinalizationForecast Preflight(const tied::source::CanonicalData&, const tied::Data&,
    const tied::PackingData&, const tied::SearchGeometryData&, const native_search::SearchDriverResult&,
    TiedFinalizationLimits);
TiedFinalizationReceipt Receipt(const tied::source::CanonicalData&, const tied::Data&,
                               const tied::PackingData&, TiedFinalizationLimits);
Inputs Pack(const tied::Data&, const tied::PackingData&, const tied::SearchGeometryData&,
            const native_search::SearchDriverResult&);
}
