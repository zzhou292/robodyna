#include "CaptureState.h"
namespace crash::output::physical_frames {
namespace detail {
void CheckBacking(const Mapping& mapping,const Run& run) {
    if(mapping.environment()) {
        const auto& actual=run.source();
        Require(actual.environment() && mapping.physical().Matches(actual.physical()) &&
            mapping.physical().execution()->parents().data()==actual.physical().execution()->parents().data() &&
            mapping.physical().failure()->parent(0)==actual.physical().failure()->parent(0),
            "Combined accepted capture belongs to another actual physical execution");
        return;
    }
    const auto& source=mapping.execution();
    Require(source.model().SharesStorage(run.execution().model()) &&
        source.execution().parents().data()==run.execution().execution().parents().data() &&
        source.physical().failure()->parent(0)==run.execution().physical().failure()->parent(0),
        "Accepted capture belongs to another execution backing");
}
records::Context MakeContext(const Mapping& mapping,Run& run,records::Identity id,Limits limits) {
    const auto scope=cases::vehicle_runtime::detail::CaptureAccess::Scope(run);
    CheckBacking(mapping,run);
    Require((!id.owner || id.owner==scope.stamp.owner_id) &&
        (!id.source_instance || id.source_instance==mapping.physical().domain()->source_instance_id()) &&
        (!id.configuration || id.configuration==scope.diagnostics.qeph.configuration_id) &&
        (!id.qualification || id.qualification==scope.diagnostics.qeph.qualification_id),
        "Caller record identity differs from accepted physical scope");
    id.owner=scope.stamp.owner_id;
    id.source_instance=mapping.physical().domain()->source_instance_id();
    id.configuration=scope.diagnostics.qeph.configuration_id;
    id.qualification=scope.diagnostics.qeph.qualification_id;
    auto context=mapping.source_mapping().MakeFrameContext(std::move(id),scope.stamp.fixed_dt,limits.records);
    CheckIdentity(mapping,context,scope);
    return context;
}
} // namespace detail
Forecast PhysicalAcceptedFrames::Preflight(const Mapping& mapping,const records::Context& context,Limits limits) {
    const auto counts=mapping.render_counts();
    Require(context.identity().source_mapping_sha256==mapping.source_mapping().digest() &&
        context.parents().size()==mapping.parents().size() && context.nodes()==mapping.physical_nodes().size(),
        "Capture forecast context differs from source mapping");
    if(mapping.environment())return detail::PlanBuffersWithEnvironment(context,mapping.physical_node_count(),
        mapping.physical_counts(),counts,mapping.payload_bytes(),limits);
    return detail::PlanBuffers(context,mapping.physical_node_count(),counts.qeph,counts.t3,counts.qbat,
        mapping.payload_bytes(),limits);
}
PhysicalAcceptedFrames::PhysicalAcceptedFrames(const Mapping& mapping,Run& run,records::Identity id,Limits limits) {
    auto context=detail::MakeContext(mapping,run,std::move(id),limits);
    const auto forecast=Preflight(mapping,context,limits);
    impl_=std::make_unique<Impl>(mapping,std::move(context),forecast);
}
PhysicalAcceptedFrames::~PhysicalAcceptedFrames()=default;
const Mapping& PhysicalAcceptedFrames::mapping() const noexcept {return impl_->mapping;}
const records::Context& PhysicalAcceptedFrames::context() const noexcept {return impl_->context;}
const Forecast& PhysicalAcceptedFrames::forecast() const noexcept {return impl_->forecast;}
const records::FrameRecord* PhysicalAcceptedFrames::frame() const noexcept {
    return impl_->frames.available?&impl_->frames.frames[impl_->frames.selected]:nullptr;
}
const records::activity::ActivityRecord* PhysicalAcceptedFrames::activity() const noexcept {
    return impl_->frames.available?&*impl_->frames.activity[impl_->frames.selected]:nullptr;
}
} // namespace crash::output::physical_frames
