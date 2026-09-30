#pragma once
#include "../TiedSearchClassification.h"

namespace crash::cases::vehicle_startup::tied_classification_detail {
struct Inputs {
    std::vector<native_search::ClassificationNode> nodes;
    std::vector<std::uint32_t> slaves, masters;
    native_search::ClassificationInterface interface;
    std::uint64_t source_instance = 0;
    native_search::ClassificationInput View();
};
Inputs Pack(const tied::source::CanonicalData&, const tied::Data&, const tied::SearchGeometryData&,
    const native_search::FinalizationMaps&, const TiedFinalizationReceipt&);
TiedClassificationData Observed(const native_search::ClassificationResult&, const tied::Data&,
    const native_search::FinalizationMaps&);
TiedClassificationForecast Preflight(const TiedSearchFinalized&, const tied::TiedClassificationContext&,
    TiedClassificationLimits);
}
