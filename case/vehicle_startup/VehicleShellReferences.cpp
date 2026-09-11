#include "ReferenceStorage.h"
#include <algorithm>
#include <optional>

namespace crash::cases::vehicle_startup {
struct VehicleShellReferences::Data {
    Data(const detail::DeclarationView& view,ReferenceForecast value)
        : source(view.source),forecast(value) {
        if (view.resolution) resolution.emplace(*view.resolution);
    }
    VehicleSourcePlan source;
    std::optional<VehicleSectionResolution> resolution;
    ReferenceForecast forecast;
    detail::ReferenceStorage references;
    static std::shared_ptr<const Data> Prepare(const detail::DeclarationView& view,ReferenceForecast forecast) {
        auto next=std::make_shared<Data>(view,forecast);
        auto& refs=next->references;
        refs.rows.reserve(view.source.counts().parents);
        refs.qeph.reserve(forecast.qeph_capacity);
        refs.t3.reserve(forecast.t3_capacity);
        refs.qbat.reserve(forecast.qbat_capacity);
        const detail::Geometry geometry(view.source.canonical().data());
        detail::PrepareRows(view,geometry,refs);
        output::Require(refs.counts.parents==view.source.counts().parents &&
                        refs.counts.attempted==view.Available(),
                        "Vehicle reference assessment lost source coverage");
        return next;
    }
};
namespace {
ReferenceForecast Forecast(const detail::DeclarationView& declarations,ReferenceLimits limits,std::size_t fixed) {
    const ReferenceLimits legacy;
    const auto& source=declarations.source;
    const auto& count=source.counts();
    const bool resolved=limits.profile==ReferenceProfile::ResolvedSections;
    const auto maximum=resolved ? ReferenceLimits::ResolvedSections() : legacy;
    output::Require((limits.profile==ReferenceProfile::Legacy || resolved) &&
                    (!resolved || declarations.resolution) &&
                    limits.parents && limits.parents<=maximum.parents &&
                    limits.nodes && limits.nodes<=maximum.nodes &&
                    limits.host_bytes && limits.host_bytes<=maximum.host_bytes,
                    "Invalid vehicle reference limits");
    output::Require(count.parents<=limits.parents && count.nodes<=limits.nodes,
                    "Vehicle reference count cap exceeded");
    ReferenceForecast forecast;
    auto add=[&](std::size_t count,std::size_t width) {
        output::Require(count<=(limits.host_bytes-forecast.total_bytes)/width,
                        "Vehicle reference startup byte cap exceeded");
        const auto bytes=count*width;
        forecast.total_bytes+=bytes;
        return bytes;
    };
    forecast.source_bound_bytes=add(declarations.SourceBound(),1);
    // Complete control/geometry objects and the unchanged conservative 32 KiB
    // envelope for one parent's packing/native startup stack. This payload
    // allowance excludes allocator bookkeeping and process RSS.
    forecast.fixed_bytes=add(fixed,1);
    forecast.row_bytes=add(count.parents,sizeof(ReferenceRow));
    forecast.qeph_capacity=std::min(count.q4,declarations.Available());
    forecast.t3_capacity=std::min(count.t3,declarations.Available());
    if (declarations.resolution && declarations.resolution->resolution_key().profile ==
        modelio::vehicle::ResolutionProfile::OriginalMidlayerV1) {
        const auto& native=declarations.resolution->native_counts();
        forecast.qeph_capacity=native.qeph;
        forecast.t3_capacity=native.t3;
        forecast.qbat_capacity=native.qbat;
    }
    forecast.reference_capacity_bytes=add(forecast.qeph_capacity,sizeof(tl::fea::qeph::ReferenceData));
    forecast.reference_capacity_bytes+=add(forecast.t3_capacity,sizeof(tl::fea::t3::ReferenceData));
    forecast.reference_capacity_bytes+=add(forecast.qbat_capacity,sizeof(tl::fea::qbat::Reference));
    // All retained decode vectors plus the largest temporary native-byte string.
    for (const auto* name : {"node_ids","shells_records","node_positions","shells_node_indices","shells_source_lines"}) {
        const auto bytes=modelio::vehicle::source::FindArray(source.canonical().data(),name).bytes.size();
        forecast.decode_bytes+=add(bytes,1);
        forecast.decode_temporary_bytes=std::max(forecast.decode_temporary_bytes,bytes+1);
    }
    add(forecast.decode_temporary_bytes,1);
    return forecast;
}
} // namespace
ReferenceForecast ForecastReferences(const VehicleSourcePlan& source,ReferenceLimits limits) {
    return Forecast(detail::DeclarationView(source),limits,sizeof(VehicleShellReferences::Data)+
        sizeof(VehicleShellReferences)+sizeof(detail::Geometry)+2*sizeof(void*)+32768);
}
ReferenceForecast ForecastReferences(const VehicleSectionResolution& resolution,ReferenceLimits limits) {
    return Forecast(detail::DeclarationView(resolution),limits,sizeof(VehicleShellReferences::Data)+
        sizeof(VehicleShellReferences)+sizeof(detail::Geometry)+2*sizeof(void*)+32768);
}
VehicleShellReferences VehicleShellReferences::Prepare(const VehicleSourcePlan& source,ReferenceLimits limits) {
    const auto forecast=ForecastReferences(source,limits);
    return VehicleShellReferences(Data::Prepare(detail::DeclarationView(source),forecast));
}
VehicleShellReferences VehicleShellReferences::Prepare(const VehicleSectionResolution& resolution,ReferenceLimits limits) {
    const auto forecast=ForecastReferences(resolution,limits);
    return VehicleShellReferences(Data::Prepare(detail::DeclarationView(resolution),forecast));
}
const VehicleSourcePlan& VehicleShellReferences::source() const noexcept {return data_->source;}
const VehicleSectionResolution* VehicleShellReferences::resolution() const noexcept {
    return data_->resolution ? &*data_->resolution : nullptr;
}
const std::vector<ReferenceRow>& VehicleShellReferences::rows() const noexcept {return data_->references.rows;}
const ReferenceCounts& VehicleShellReferences::counts() const noexcept {return data_->references.counts;}
const ReferenceForecast& VehicleShellReferences::forecast() const noexcept {return data_->forecast;}
const ReferenceRow* VehicleShellReferences::first_error() const noexcept {
    const auto i=data_->references.first_error;return i==SIZE_MAX?nullptr:&rows()[i];
}
const tl::fea::qeph::ReferenceData* VehicleShellReferences::qeph(std::size_t i) const noexcept {
    if(i>=rows().size()||rows()[i].family!=ReferenceFamily::Qeph||rows()[i].status!=ReferenceStatus::Success)return nullptr;
    return &data_->references.qeph[rows()[i].reference_index];
}
const tl::fea::t3::ReferenceData* VehicleShellReferences::t3(std::size_t i) const noexcept {
    if(i>=rows().size()||rows()[i].family!=ReferenceFamily::T3||rows()[i].status!=ReferenceStatus::Success)return nullptr;
    return &data_->references.t3[rows()[i].reference_index];
}
const tl::fea::qbat::Reference* VehicleShellReferences::qbat(std::size_t i) const noexcept {
    if(i>=rows().size()||rows()[i].family!=ReferenceFamily::Qbat||rows()[i].status!=ReferenceStatus::Success)return nullptr;
    return &data_->references.qbat[rows()[i].reference_index];
}
} // namespace crash::cases::vehicle_startup
