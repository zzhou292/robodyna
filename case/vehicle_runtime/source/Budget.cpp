#include "Storage.h"
#include "../Forecast.h"
#include "lib_utils/BoundedArena.h"
#include "../Reports.h"
namespace crash::cases::vehicle_runtime {
std::size_t Source::retained_host_upper_bound(std::size_t cap) const {
    const auto* old=data_->original();
    const auto bytes=old?detail::SourceBytes(old->execution,old->attachments,cap):
        data_->environment()->retained_host_upper_bound(cap);
    // Owning source APIs report retained graph bounds separately from their
    // historical constructor peaks. Never derive private layout discounts here.
    tl::util::BoundedArenaLayout layout(cap);tl::util::ArenaRegion unused;
    output::Require(layout.Append<std::byte>(bytes,unused) &&
        layout.Append<std::byte>(sizeof(Source)+sizeof(Data)+128,unused),
        "Complete runtime source handle exceeds host cap");
    return layout.bytes();
}
std::size_t Source::additional_joint_source_bytes() const noexcept {
    if(const auto* old=data_->original()) return old->joints?old->joints->additional_owned_payload_bytes():0;
    return 0; // Complete environment owner-source forecast already includes it.
}
std::array<std::size_t,3> Source::construction_peaks() const noexcept {
    if(const auto* old=data_->original()) return {old->execution.model().forecast().total_bytes,
        old->execution.forecast().total_bytes,old->attachments.forecast().total_bytes};
    const auto& next=*data_->environment();
    return {next.execution_source().mechanical().forecast().peak_bytes,
        next.execution_source().forecast().total_bytes,next.forecast().peak_bytes};
}
}
