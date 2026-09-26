#include "execution/Internal.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>

namespace crash::cases::vehicle_wall::native {
namespace d = execution_detail;
struct EnvelopeExecutionSource::Data {
    Data(const EnvelopePhysicalSource& source, modelio::assembly::Law1ExecutionPolicy policy)
        : mechanical(source), policy(policy) {}
    EnvelopePhysicalSource mechanical;
    modelio::assembly::Law1ExecutionPolicy policy;
    tl::fea::ShellPhysicalBinding physical;
    EnvelopeExecutionForecast forecast;
};
EnvelopeExecutionForecast EnvelopeExecutionSource::Preflight(const EnvelopePhysicalSource& source,
    EnvelopeExecutionLimits limits) {
    const auto& resolution=d::Check(source);
    d::original::detail::CheckLimits(limits);
    (void)d::original::detail::ResolvePolicy(source.vehicle_references(),
        modelio::assembly::Law1ExecutionProfile::NativeA62OrdinaryNpt0);
    d::Require(source.domain().node_count() <= limits.execution.max_nodes &&
        source.domain().node_count() <= limits.physical.max_nodes &&
        source.shells().node_count() <= limits.catalog.max_nodes,
        "Combined execution node domain exceeds native bounds");
    const auto fixed=sizeof(EnvelopeExecutionSource)+sizeof(Data)+sizeof(d::original::detail::Packing)+
        sizeof(d::fe::ShellBatchPlasticityBinding)+sizeof(d::fe::ShellBatchFailureBinding)+
        sizeof(d::fe::ShellExecutionBinding)+6*64;
    const auto current=d::original::detail::ForecastPayload(source.retained_host_upper_bound(limits.host_bytes),fixed,
        resolution.parts().size()+1,resolution.parents().size()+1,resolution.parts().size(),limits);
    EnvelopeExecutionForecast result{current.source_bytes,current.fixed_bytes,current.packing_bytes,current.native_reservation,
        source.forecast().peak_bytes,current.total_bytes,std::max(source.forecast().peak_bytes,current.total_bytes)};
    d::Require(result.total_bytes<=limits.host_bytes,"Complete combined execution source chain exceeds cap");
    return result;
}
EnvelopeExecutionSource EnvelopeExecutionSource::Prepare(const EnvelopePhysicalSource& source,
    EnvelopeExecutionLimits limits) {
    const auto forecast=Preflight(source,limits);
    const auto& refs=source.vehicle_references();
    const auto& resolution=*refs.resolution();
    const auto policy=d::original::detail::ResolvePolicy(refs,modelio::assembly::Law1ExecutionProfile::NativeA62OrdinaryNpt0);
    d::original::detail::Packing packed;
    packed.Reserve(resolution.parts().size()+1,resolution.parents().size()+1,resolution.parts().size());
    d::Require(packed.capacity_bytes() <= forecast.packing_bytes,"Combined execution packing capacity exceeds preflight");
    d::original::detail::PackOriginalReferenceSource(refs,packed,policy);
    d::AppendWall(packed,source,policy);
    d::Require(packed.capacity_bytes() <= forecast.packing_bytes,"Combined execution packing grew beyond preflight");
    d::fe::ShellBatchPlasticityBinding catalog;
    auto status=catalog.InitializeExecutionCatalog(source.shells(),packed.input(),limits.catalog);
    d::Require(status.status==d::fe::ShellPlasticityBindingStatus::Success,status.message);
    d::fe::ShellBatchFailureBinding failure;
    status=failure.InitializeExecution(catalog,packed.failure.data(),packed.failure.size(),limits.failure);
    d::Require(status.status==d::fe::ShellPlasticityBindingStatus::Success,status.message);
    d::fe::ShellExecutionBinding execution;
    status=execution.Initialize(catalog,source.coefficients(),source.rigid_assembly(),limits.execution);
    d::Require(status.status==d::fe::ShellPlasticityBindingStatus::Success,status.message);
    d::Require(execution.counts().rigid_skin==5102 && execution.counts().constitutive==344544,
        "Combined execution changed original roles or omitted the declared environment");
    auto next=std::make_shared<Data>(source,policy);
    const d::fe::ShellFormulationScope scope{&source.shells(),&catalog,&failure,nullptr};
    const auto physical=next->physical.InitializeExecution(scope,source.coefficients(),execution,limits.physical);
    d::Require(bool(physical),physical.message);
    next->forecast=forecast;
    return EnvelopeExecutionSource(std::move(next));
}
const EnvelopePhysicalSource& EnvelopeExecutionSource::mechanical() const noexcept { return data_->mechanical; }
const tl::fea::ShellPhysicalBinding& EnvelopeExecutionSource::physical() const noexcept { return data_->physical; }
const tl::fea::ShellBatchPlasticityBinding& EnvelopeExecutionSource::catalog() const noexcept { return *data_->physical.catalog(); }
const tl::fea::ShellBatchFailureBinding& EnvelopeExecutionSource::failure() const noexcept { return *data_->physical.failure(); }
const tl::fea::ShellExecutionBinding& EnvelopeExecutionSource::execution() const noexcept { return *data_->physical.execution(); }
const modelio::assembly::Law1ExecutionPolicy& EnvelopeExecutionSource::law1_policy() const noexcept { return data_->policy; }
const EnvelopeExecutionForecast& EnvelopeExecutionSource::forecast() const noexcept { return data_->forecast; }
std::size_t EnvelopeExecutionSource::retained_host_upper_bound(std::size_t cap) const {
    tl::util::BoundedArenaLayout bytes(cap);tl::util::ArenaRegion unused;
    // The complete physical graph is reported by the owning TL type. The
    // mechanical bound remains conservative even where its native inputs are
    // also held by that graph; no unauthenticated overlap is subtracted.
    for(const auto count:{mechanical().retained_host_upper_bound(cap),physical().owned_payload_bytes(),
            sizeof(EnvelopeExecutionSource)+sizeof(Data)+std::size_t{128}})
        d::Require(bytes.Append<std::byte>(count,unused),"Retained combined execution source exceeds cap");
    return bytes.bytes();
}

}
