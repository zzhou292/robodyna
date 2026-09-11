#include "TiedSearchFinalized.h"
#include "tied_finalization/Internal.h"
namespace crash::cases::vehicle_startup {
struct TiedSearchFinalized::Storage {
    explicit Storage(const TiedSearchAssessment& value) : assessment(value) {}
    TiedSearchAssessment assessment;
    native_search::FinalizedSearch finalized;
    TiedFinalizationReceipt receipt;
    TiedFinalizationForecast forecast;
};
TiedFinalizationForecast TiedSearchFinalized::Forecast(const TiedSearchAssessment& assessment,
        TiedFinalizationLimits limits) {
    const auto& geometry = assessment.geometry();
    const auto& packing = geometry.packing();
    const auto& declaration = packing.declaration();
    return tied_finalization_detail::Preflight(declaration.canonical().data(), declaration.data(),
        packing.data(), geometry.data(), assessment.result(), limits);
}
TiedSearchFinalized TiedSearchFinalized::Prepare(const TiedSearchAssessment& assessment,
        TiedFinalizationLimits limits) {
    static_assert(sizeof(Storage) == sizeof(TiedSearchAssessment) + sizeof(native_search::FinalizedSearch) +
        sizeof(TiedFinalizationReceipt) + sizeof(TiedFinalizationForecast),
        "Update finalized tied search storage accounting");
    const auto forecast = Forecast(assessment, limits);
    const auto& geometry = assessment.geometry();
    const auto& packing = geometry.packing();
    const auto& declaration = packing.declaration();
    const auto receipt = tied_finalization_detail::Receipt(declaration.canonical().data(),
        declaration.data(), packing.data(), limits);
    const auto inputs = tied_finalization_detail::Pack(declaration.data(), packing.data(),
        geometry.data(), assessment.result());
    auto draft = std::make_shared<Storage>(assessment);
    draft->forecast = forecast;
    draft->receipt = receipt;
    const auto report = native_search::FinalizeSearch(inputs.View(geometry.data()), &draft->finalized, limits.native);
    output::Require(bool(report), report.message);
    return TiedSearchFinalized(std::move(draft));
}
const TiedSearchAssessment& TiedSearchFinalized::assessment() const noexcept { return storage_->assessment; }
const native_search::FinalizationMaps& TiedSearchFinalized::data() const noexcept { return *storage_->finalized.data(); }
const TiedFinalizationReceipt& TiedSearchFinalized::receipt() const noexcept { return storage_->receipt; }
const TiedFinalizationForecast& TiedSearchFinalized::forecast() const noexcept { return storage_->forecast; }
const tied::Node& TiedSearchFinalized::secondary(std::size_t compact) const {
    return assessment().secondary(data().slaves.at(compact));
}
const tied::Element& TiedSearchFinalized::selected_master(std::size_t compact) const {
    const auto* master = assessment().selected_master(data().slaves.at(compact));
    output::Require(master != nullptr, "Finalized tied slave has no original master association");
    return *master;
}
}
