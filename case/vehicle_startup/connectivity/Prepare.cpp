#include "Internal.h"
#include <optional>

namespace crash::cases::vehicle_startup::connectivity {
struct VehicleConnectivity::Storage {
    Storage(const VehiclePhysicalAttachments& input, const joints::VehicleJointModel* model) : source(input) {
        if (model) joints.emplace(*model);
    }
    VehiclePhysicalAttachments source;
    std::optional<joints::VehicleJointModel> joints;
    Data data;
    Forecast forecast;
};
Forecast VehicleConnectivity::Preflight(const VehiclePhysicalAttachments& source, Limits limits) {
    return PreflightImpl(source,nullptr,limits);
}
VehicleConnectivity VehicleConnectivity::Prepare(const VehiclePhysicalAttachments& source, Limits limits) {
    return PrepareImpl(source,nullptr,limits);
}
Forecast VehicleConnectivity::PreflightWithJoints(const VehiclePhysicalAttachments& source,
    const joints::VehicleJointModel& joints, Limits limits) {
    return PreflightImpl(source,&joints,limits);
}
VehicleConnectivity VehicleConnectivity::PrepareWithJoints(const VehiclePhysicalAttachments& source,
    const joints::VehicleJointModel& joints, Limits limits) {
    return PrepareImpl(source,&joints,limits);
}
Forecast VehicleConnectivity::PreflightImpl(const VehiclePhysicalAttachments& source,
    const joints::VehicleJointModel* joints, Limits limits) {
    const auto count = detail::Extents(source,joints);
    return detail::Budget(source,count,limits,sizeof(Storage)+sizeof(VehicleConnectivity)+64+8192,joints);
}
VehicleConnectivity VehicleConnectivity::PrepareImpl(const VehiclePhysicalAttachments& source,
    const joints::VehicleJointModel* joints, Limits limits) {
    const auto forecast = PreflightImpl(source,joints,limits);
    auto next = std::make_shared<Storage>(source,joints);
    next->forecast = forecast;
    auto& data = next->data;
    data.element_label.resize(forecast.extents.nodes);
    data.transfer_label.resize(forecast.extents.nodes);
    data.node_roles.resize(forecast.extents.nodes);
    detail::Relations relations(data,forecast.extents);
    detail::VisitShells(source,relations);
    detail::VisitElements(source,relations);
    detail::VisitConstraints(source,relations);
    if (joints) detail::VisitJoints(*joints,relations);
    relations.Complete();
    detail::Partition(source.physical().source_domain().domain().nodes(),data);
    detail::Require(data.counts.by_kind == forecast.extents.by_kind,
                    "Connectivity typed family count differs after traversal");
    return VehicleConnectivity(std::move(next));
}
const VehiclePhysicalAttachments& VehicleConnectivity::source() const noexcept { return storage_->source; }
const joints::VehicleJointModel* VehicleConnectivity::joint_model() const noexcept {
    return storage_->joints ? &*storage_->joints : nullptr;
}
const Data& VehicleConnectivity::data() const noexcept { return storage_->data; }
const Forecast& VehicleConnectivity::forecast() const noexcept { return storage_->forecast; }
} // namespace crash::cases::vehicle_startup::connectivity
