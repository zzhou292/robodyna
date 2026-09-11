#include "tied_classification/Internal.h"

namespace crash::cases::vehicle_startup {
struct TiedSearchClassification::Storage {
    Storage(const TiedSearchFinalized& f, const tied::TiedClassificationContext& c) : finalized(f), context(c) {}
    TiedSearchFinalized finalized;
    tied::TiedClassificationContext context;
    TiedClassificationData data;
    TiedClassificationForecast forecast;
};
TiedClassificationForecast TiedSearchClassification::Forecast(const TiedSearchFinalized& finalized,
        const tied::TiedClassificationContext& context, TiedClassificationLimits limits) {
    return tied_classification_detail::Preflight(finalized,context,limits);
}
TiedSearchClassification TiedSearchClassification::Prepare(const TiedSearchFinalized& finalized,
        const tied::TiedClassificationContext& context, TiedClassificationLimits limits) {
    const auto forecast = Forecast(finalized,context,limits);
    const auto& geometry = finalized.assessment().geometry();
    const auto& declaration = geometry.packing().declaration();
    auto inputs = tied_classification_detail::Pack(declaration.canonical().data(),declaration.data(),
        geometry.data(),finalized.data(),finalized.receipt());
    native_search::ClassificationResult result;
    const auto report = native_search::Classify(inputs.View(),&result,limits.native);
    output::Require(bool(report), "Native tied classification rejected the projected source context");
    auto next = std::make_shared<Storage>(finalized,context);
    next->data = tied_classification_detail::Observed(result,declaration.data(),finalized.data());
    next->forecast = forecast;
    return TiedSearchClassification(std::move(next));
}
const TiedSearchFinalized& TiedSearchClassification::finalized() const noexcept { return storage_->finalized; }
const tied::TiedClassificationContext& TiedSearchClassification::context() const noexcept { return storage_->context; }
const TiedClassificationData& TiedSearchClassification::data() const noexcept { return storage_->data; }
const TiedClassificationForecast& TiedSearchClassification::forecast() const noexcept { return storage_->forecast; }
}
