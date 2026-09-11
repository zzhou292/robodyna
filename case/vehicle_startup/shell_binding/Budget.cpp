#include "Internal.h"
#include <algorithm>

namespace crash::cases::vehicle_startup::shell_binding_detail {
VehicleShellBindingForecast Forecast(const VehicleShellReferences& refs,VehicleShellBindingLimits limits,
                                     std::size_t fixed) {
    using output::Require;
    const auto* resolved=refs.resolution();
    Require(resolved && resolved->resolution_key().profile==
        modelio::vehicle::ResolutionProfile::OriginalRigidPartsV1,
        "Complete shell binding requires the explicit original rigid-part resolution");
    const auto& counts=refs.counts();
    const auto& source=refs.source();
    const auto& native=resolved->native_counts();
    Require(counts.parents==source.counts().parents && counts.parents==refs.rows().size() &&
        counts.succeeded==counts.parents && !counts.rejected && !counts.unresolved &&
        counts.qeph_succeeded==native.qeph && counts.t3_succeeded==native.t3 &&
        counts.qbat_succeeded==native.qbat && native.qbat,
        "Complete shell binding cannot omit unresolved or rejected source parents");
    const VehicleShellBindingLimits maximum;
    const auto& cap=limits.native;
    Require(limits.host_bytes && limits.host_bytes<=maximum.host_bytes &&
        cap.max_parents && cap.max_parents<=maximum.native.max_parents &&
        cap.max_nodes && cap.max_nodes<=maximum.native.max_nodes &&
        cap.max_owned_bytes>=sizeof(tl::fea::ShellBatchBinding) &&
        cap.max_owned_bytes<=maximum.native.max_owned_bytes &&
        cap.max_startup_scratch_bytes && cap.max_startup_scratch_bytes<=maximum.native.max_startup_scratch_bytes &&
        counts.parents<=cap.max_parents && source.counts().nodes<=cap.max_nodes,
        "Invalid complete shell binding count or allocation limits");
    VehicleShellBindingForecast f;
    auto add=[&](std::size_t count,std::size_t width) {
        Require(width && count<=(limits.host_bytes-f.total_bytes)/width,
            "Complete shell binding startup byte cap exceeded");
        const auto bytes=count*width;f.total_bytes+=bytes;return bytes;
    };
    f.source_reference_bound=add(refs.forecast().total_bytes,1);
    // Data already embeds the TL handle. Reserve its remaining complete backing
    // and its declared scratch limit without reproducing TL's private layout.
    f.fixed_bytes=add(fixed,1);
    f.native_owned_reservation=add(cap.max_owned_bytes-sizeof(tl::fea::ShellBatchBinding),1);
    f.native_scratch_reservation=add(cap.max_startup_scratch_bytes,1);
    f.input_bytes=add(native.qeph,sizeof(tl::fea::ShellQephBindingInput));
    f.input_bytes+=add(native.t3,sizeof(tl::fea::ShellT3BindingInput));
    f.input_bytes+=add(native.qbat,sizeof(tl::fea::ShellQbatBindingInput));
    const auto& canonical=source.canonical().data();
    f.mapping_bytes=add(canonical.canonical_nodes,sizeof(std::size_t));
    for(const auto* name:{"node_ids","shells_node_indices"}) {
        const auto bytes=modelio::vehicle::source::FindArray(canonical,name).bytes.size();
        f.decode_bytes+=add(bytes,1);
        f.decode_temporary_bytes=std::max(f.decode_temporary_bytes,bytes+1);
    }
    add(f.decode_temporary_bytes,1);
    return f;
}
}
