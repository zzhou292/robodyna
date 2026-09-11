#include "tied_cin/Internal.h"
namespace crash::cases::vehicle_startup {
namespace {
constexpr std::size_t StorageControlReserveBytes = 64;
}
struct TiedCinAttachments::Storage {
    explicit Storage(const TiedSearchPostKinChk& value) : post(value) {}
    TiedSearchPostKinChk post;
    native_search::TiedCinAttachmentModel model;
    TiedCinAttachmentForecast forecast;
};
TiedCinAttachmentForecast TiedCinAttachments::Forecast(const TiedSearchPostKinChk& post,
        const tl::fea::NodalNodeDomain& domain,TiedCinAttachmentLimits limits) {
    const auto& declaration = post.classification().context().auxiliary().declaration();
    output::Require(post.phase() == TiedPostKinChkPhase::ObservedKinetAfterKinChk &&
                    post.result().source_instance_id() == domain.source_instance_id(),
                    "CIN source/domain phase or identity differs");
    return tied_cin_detail::Budget(post.forecast(),post.result(),domain,declaration.canonical().data().canonical_nodes,
                                  sizeof(Storage)+sizeof(tied_cin_detail::Inputs)+StorageControlReserveBytes,limits);
}
TiedCinAttachments TiedCinAttachments::Prepare(const TiedSearchPostKinChk& post,
        const tl::fea::NodalNodeDomain& domain,TiedCinAttachmentLimits limits) {
    const auto forecast = Forecast(post,domain,limits);
    const auto& classified = post.classification();
    const auto& finalized = classified.finalized();
    const auto& geometry = finalized.assessment().geometry();
    const auto& packing = geometry.packing();
    const auto& declaration = packing.declaration();
    const auto inputs = tied_cin_detail::Pack(declaration.canonical().data(),declaration.data(),packing.data(),
        geometry.data(),finalized.data(),classified.data(),post.result());
    native_search::TiedCinAttachmentModel model;
    const auto report = native_search::PrepareCinAttachments(post.result(),domain,{inputs.data(),inputs.size()},&model,limits.native);
    if (!report) {
        const auto* row = report.row < inputs.size() ? &inputs[report.row] : nullptr;
        throw TiedCinAttachmentError(report,row ? row->secondary_source_id : 0,row ? row->master_source.element_id : 0);
    }
    auto next = std::make_shared<Storage>(post);
    next->model = std::move(model);
    next->forecast = forecast;
    return TiedCinAttachments(std::move(next));
}
const TiedSearchPostKinChk& TiedCinAttachments::post_kinchk() const noexcept { return storage_->post; }
const native_search::TiedCinAttachmentModel& TiedCinAttachments::model() const noexcept { return storage_->model; }
const TiedCinAttachmentForecast& TiedCinAttachments::forecast() const noexcept { return storage_->forecast; }
}
