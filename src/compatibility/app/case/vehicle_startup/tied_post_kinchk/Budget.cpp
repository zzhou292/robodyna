#include "Internal.h"
#include "modelio/tied_shell/classification/Internal.h"

namespace crash::cases::vehicle_startup::post_kinchk_detail {
TiedPostKinChkForecast Budget(const TiedClassificationForecast& prior, std::size_t count,
        std::size_t receipt_bytes, TiedPostKinChkLimits limits) {
    using output::Require;
    using tied::classification_detail::Add;
    const TiedPostKinChkLimits hard;
    Require(limits.host_bytes && limits.host_bytes <= hard.host_bytes && limits.native.max_slaves &&
            limits.native.max_slaves <= hard.native.max_slaves && limits.native.max_host_bytes &&
            limits.native.max_host_bytes <= hard.native.max_host_bytes,"Invalid post-KINCHK limits");
    TiedPostKinChkForecast out;
    // Reuse the retained phases; prior input staging/native scratch has ended.
    // The source-context reservation is deliberately conservative, because it
    // includes its retired parse phase. It is not described as current RSS.
    for (const auto bytes : {prior.source_context_reservation_bytes,prior.distinct_finalized_payload_bytes,
                            prior.result_payload_bytes})
        Add(out.retained_classification_reservation_bytes,bytes,1,limits.host_bytes);
    Add(out.input_staging_bytes,count,sizeof(native_search::KinChkSlave),limits.host_bytes);
    out.receipt_payload_bytes = receipt_bytes;
    Require(bool(native_search::ForecastPostKinChk(count,0,&out.native,limits.native)),
            "Post-KINCHK native count/byte forecast rejected");
    for (const auto bytes : {out.retained_classification_reservation_bytes,out.input_staging_bytes,
                            out.receipt_payload_bytes,out.native.startup_payload_bytes})
        Add(out.total_host_bytes,bytes,1,limits.host_bytes);
    return out;
}
}
