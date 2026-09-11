#include "tied_post_kinchk/Internal.h"
#include "modelio/tied_shell/classification/Internal.h"

namespace crash::cases::vehicle_startup {
struct TiedSearchPostKinChk::Storage {
    explicit Storage(const TiedSearchClassification& c) : classification(c) {}
    TiedSearchClassification classification;
    TiedPostKinChkReceipt receipt;
    native_search::PostKinChkResult result;
    TiedPostKinChkForecast forecast;
};
TiedPostKinChkForecast TiedSearchPostKinChk::Forecast(const TiedSearchClassification& classified,
        TiedPostKinChkLimits limits) {
    const auto& declaration = classified.context().auxiliary().declaration();
    output::Require(&declaration.data() == &classified.finalized().assessment().geometry().packing().declaration().data(),
                    "Post-KINCHK immutable classification/source backing differs");
    const auto& source = declaration.data().sources.at(declaration.data().contact_source).block;
    std::size_t bytes = sizeof(Storage) + sizeof(post_kinchk_detail::Inputs);
    tied::classification_detail::Add(bytes,source.filename.size()+source.keyword.size()+source.sha256.size()+3,
                                    1,limits.host_bytes);
    return post_kinchk_detail::Budget(classified.forecast(),classified.data().slaves.size(),bytes,limits);
}
TiedSearchPostKinChk TiedSearchPostKinChk::Prepare(const TiedSearchClassification& classified,
        TiedPostKinChkLimits limits) {
    const auto forecast = Forecast(classified,limits);
    const auto& declaration = classified.context().auxiliary().declaration();
    auto receipt = post_kinchk_detail::Receipt(declaration.canonical().data(),declaration.data(),
        classified.finalized().receipt(),classified.context().receipt());
    const auto input = post_kinchk_detail::Pack(classified.data(),receipt);
    native_search::PostKinChkResult result;
    output::Require(bool(native_search::PostKinChk(input.View(classified.data(),receipt),&result,limits.native)),
                    "Native post-KINCHK observed context rejected");
    auto next = std::make_shared<Storage>(classified);
    next->receipt = std::move(receipt);
    next->result = std::move(result);
    next->forecast = forecast;
    return TiedSearchPostKinChk(std::move(next));
}
const TiedSearchClassification& TiedSearchPostKinChk::classification() const noexcept { return storage_->classification; }
const TiedPostKinChkReceipt& TiedSearchPostKinChk::receipt() const noexcept { return storage_->receipt; }
const native_search::PostKinChkResult& TiedSearchPostKinChk::result() const noexcept { return storage_->result; }
const TiedPostKinChkForecast& TiedSearchPostKinChk::forecast() const noexcept { return storage_->forecast; }
}
