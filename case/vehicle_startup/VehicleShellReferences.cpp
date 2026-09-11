#include "ReferenceStorage.h"
#include <algorithm>

namespace crash::cases::vehicle_startup {
struct VehicleShellReferences::Data {
    explicit Data(const VehicleSourcePlan& s,ReferenceForecast f):source(s),forecast(f) {}
    VehicleSourcePlan source;
    ReferenceForecast forecast;
    detail::ReferenceStorage references;
};
ReferenceForecast ForecastReferences(const VehicleSourcePlan& source,ReferenceLimits limits) {
    const ReferenceLimits maximum;const auto& count=source.counts();
    output::Require(limits.parents&&limits.parents<=maximum.parents&&limits.nodes&&limits.nodes<=maximum.nodes&&
        limits.host_bytes&&limits.host_bytes<=maximum.host_bytes,"Invalid vehicle reference limits");
    output::Require(count.parents<=limits.parents&&count.nodes<=limits.nodes,"Vehicle reference count cap exceeded");
    ReferenceForecast f;
    auto add=[&](std::size_t count,std::size_t width) {
        output::Require(count<=(limits.host_bytes-f.total_bytes)/width,"Vehicle reference startup byte cap exceeded");
        const auto bytes=count*width;f.total_bytes+=bytes;return bytes;
    };
    f.source_bound_bytes=add(source.startup_budget_bytes(),1);
    // Includes the complete control/geometry objects and a conservative 32 KiB
    // envelope for one parent's packing/native startup stack. Allocator/driver
    // bookkeeping and process RSS remain outside this owned-payload budget.
    f.fixed_bytes=add(sizeof(VehicleShellReferences::Data)+sizeof(VehicleShellReferences)+
        sizeof(detail::Geometry)+2*sizeof(void*)+32768,1);
    f.row_bytes=add(count.parents,sizeof(ReferenceRow));
    f.qeph_capacity=std::min(count.q4,count.supported_parents);
    f.t3_capacity=std::min(count.t3,count.supported_parents);
    f.reference_capacity_bytes=add(f.qeph_capacity,sizeof(tl::fea::qeph::ReferenceData));
    f.reference_capacity_bytes+=add(f.t3_capacity,sizeof(tl::fea::t3::ReferenceData));
    // Decode creates one temporary native-byte string and one typed vector.
    // All retained decode vectors + the largest transient overlap is bounded.
    for(const auto* name:{"node_ids","shells_records","node_positions","shells_node_indices","shells_source_lines"}) {
        const auto n=modelio::vehicle::source::FindArray(source.canonical().data(),name).bytes.size();
        f.decode_bytes+=add(n,1);f.decode_temporary_bytes=std::max(f.decode_temporary_bytes,n+1);
    }
    add(f.decode_temporary_bytes,1);
    return f;
}
VehicleShellReferences VehicleShellReferences::Prepare(const VehicleSourcePlan& source,ReferenceLimits limits) {
    const auto forecast=ForecastReferences(source,limits); // Before decoded/backing allocations.
    auto next=std::make_shared<Data>(source,forecast);auto& refs=next->references;
    refs.rows.reserve(source.counts().parents);refs.qeph.reserve(forecast.qeph_capacity);refs.t3.reserve(forecast.t3_capacity);
    const detail::Geometry geometry(source.canonical().data());
    detail::PrepareRows(source,geometry,refs);
    output::Require(refs.counts.parents==source.counts().parents&&refs.counts.attempted==source.counts().supported_parents,
                    "Vehicle reference assessment lost source coverage");
    return VehicleShellReferences(std::move(next));
}
const VehicleSourcePlan& VehicleShellReferences::source() const noexcept {return data_->source;}
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
} // namespace crash::cases::vehicle_startup
