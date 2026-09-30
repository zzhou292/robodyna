#include "Internal.h"
#include "modelio/tied_shell/classification/Internal.h"
namespace crash::cases::vehicle_startup::tied_cin_detail {
TiedCinAttachmentForecast Budget(const TiedPostKinChkForecast& prior,const native_search::PostKinChkResult& post,
        const tl::fea::NodalNodeDomain& domain,std::size_t nodes,std::size_t fixed_bytes,TiedCinAttachmentLimits limits) {
    using tied::classification_detail::Add;
    using output::Require;
    Require(limits.host_bytes && limits.host_bytes <= TiedCinAttachmentLimits{}.host_bytes,"Invalid CIN source byte limit");
    TiedCinAttachmentForecast out;
    Add(out.post_kinchk_reservation_bytes,prior.retained_classification_reservation_bytes,1,limits.host_bytes);
    Add(out.post_kinchk_reservation_bytes,prior.receipt_payload_bytes,1,limits.host_bytes);
    Add(out.post_kinchk_reservation_bytes,post.forecast().owned_payload_bytes,1,limits.host_bytes);
    // Decode temporarily holds both native byte string and vector. While the
    // position array is decoded, the completed original-NID vector remains.
    Add(out.input_staging_bytes,nodes,sizeof(tied::SourceId)+6*sizeof(double),limits.host_bytes);
    Add(out.input_staging_bytes,post.slaves().count,sizeof(native_search::CinAttachmentDeclaration),limits.host_bytes);
    out.fixed_bytes = fixed_bytes;
    Require(bool(native_search::ForecastCinAttachments(post,domain,post.slaves().count,0,&out.model,limits.native)),
            "CIN complete domain/model forecast rejected");
    // The post-KINCHK backing is already retained above. The model explicitly
    // copies that same handle; only its additional backing/domain/scratch follow.
    for (const auto bytes : {out.post_kinchk_reservation_bytes,out.input_staging_bytes,out.fixed_bytes,
                            out.model.model_payload_bytes,out.model.domain_payload_bytes,out.model.scratch_bytes})
        Add(out.total_host_bytes,bytes,1,limits.host_bytes);
    return out;
}
}
