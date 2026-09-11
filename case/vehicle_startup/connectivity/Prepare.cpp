#include "Internal.h"

namespace crash::cases::vehicle_startup::connectivity {
struct VehicleConnectivity::Storage {
    explicit Storage(const VehiclePhysicalAttachments& input) : source(input) {}
    VehiclePhysicalAttachments source;
    Data data;
    Forecast forecast;
};
Forecast VehicleConnectivity::Preflight(const VehiclePhysicalAttachments& source, Limits limits) {
    const auto count = detail::Extents(source);
    return detail::Budget(source,count,limits,sizeof(Storage)+sizeof(VehicleConnectivity)+64+8192);
}
VehicleConnectivity VehicleConnectivity::Prepare(const VehiclePhysicalAttachments& source, Limits limits) {
    const auto forecast = Preflight(source,limits);
    auto next = std::make_shared<Storage>(source);
    next->forecast = forecast;
    auto& data = next->data;
    data.element_label.resize(forecast.extents.nodes);
    data.transfer_label.resize(forecast.extents.nodes);
    data.node_roles.resize(forecast.extents.nodes);
    detail::Relations relations(data,forecast.extents);
    detail::VisitShells(source,relations);
    detail::VisitElements(source,relations);
    detail::VisitConstraints(source,relations);
    relations.Complete();
    detail::Partition(source.physical().source_domain().domain().nodes(),data);
    detail::Require(data.counts.by_kind == forecast.extents.by_kind,
                    "Connectivity typed family count differs after traversal");
    return VehicleConnectivity(std::move(next));
}
const VehiclePhysicalAttachments& VehicleConnectivity::source() const noexcept { return storage_->source; }
const Data& VehicleConnectivity::data() const noexcept { return storage_->data; }
const Forecast& VehicleConnectivity::forecast() const noexcept { return storage_->forecast; }
} // namespace crash::cases::vehicle_startup::connectivity
