#include "Internal.h"
#include "modelio/tied_shell/classification/Internal.h"
#include <iterator>

namespace crash::cases::vehicle_startup::tied_classification_detail {
TiedClassificationForecast Preflight(const TiedSearchFinalized& finalized,
        const tied::TiedClassificationContext& context, TiedClassificationLimits limits) {
    using output::Require;
    using tied::classification_detail::Add;
    const TiedClassificationLimits hard;
    const std::size_t supplied[] = {limits.host_bytes,limits.native.max_nodes,limits.native.max_interfaces,
        limits.native.max_roles,limits.native.max_occurrences,limits.native.max_host_bytes};
    const std::size_t maximum[] = {hard.host_bytes,hard.native.max_nodes,hard.native.max_interfaces,
        hard.native.max_roles,hard.native.max_occurrences,hard.native.max_host_bytes};
    for (std::size_t i = 0; i < std::size(supplied); ++i)
        Require(supplied[i] && supplied[i] <= maximum[i], "Invalid tied classification limits");
    const auto& geometry = finalized.assessment().geometry();
    const auto& packing = geometry.packing();
    const auto& declaration = packing.declaration();
    Require(&declaration.data() == &context.auxiliary().declaration().data(),
            "Classification and finalized declaration backing differ");
    const auto& canonical = declaration.canonical().data();
    const auto& maps = finalized.data();
    Require(geometry.data().working_positions.size() <= limits.native.max_nodes &&
            maps.slaves.size() <= limits.native.max_occurrences &&
            maps.main_nodes.size() <= limits.native.max_occurrences-maps.slaves.size(),
            "Classification complete source count exceeds cap");
    TiedClassificationForecast out;
    out.source_context_reservation_bytes = context.receipt().startup_budget_bytes;
    // Context already reserves canonical, declaration and rigid/auxiliary
    // backing. Only distinct packing/geometry/assessment/finalizer data follow.
    for (const auto bytes : {packing.data().owned_payload_bytes,geometry.data().owned_payload_bytes,
            finalized.forecast().retained_assessment_bytes,maps.owned_payload_bytes})
        Add(out.distinct_finalized_payload_bytes,bytes,1,limits.host_bytes);
    Add(out.input_staging_bytes,canonical.canonical_nodes,2*sizeof(tied::SourceId),limits.host_bytes);
    Add(out.input_staging_bytes,geometry.data().working_positions.size(),sizeof(native_search::ClassificationNode),limits.host_bytes);
    Add(out.input_staging_bytes,maps.slaves.size()+maps.main_nodes.size(),sizeof(std::uint32_t),limits.host_bytes);
    Add(out.result_payload_bytes,maps.slaves.size(),sizeof(ClassifiedTiedSlave),limits.host_bytes);
    Add(out.result_payload_bytes,sizeof(TiedClassificationData)+sizeof(TiedClassificationForecast)+
        sizeof(TiedSearchFinalized)+sizeof(tied::TiedClassificationContext)+sizeof(Inputs),1,limits.host_bytes);
    out.native_reservation_bytes = limits.native.max_host_bytes;
    for (const auto bytes : {out.source_context_reservation_bytes,out.distinct_finalized_payload_bytes,
            out.input_staging_bytes,out.result_payload_bytes,out.native_reservation_bytes})
        Add(out.total_host_bytes,bytes,1,limits.host_bytes);
    return out;
}
}
