#include "shell_binding/Internal.h"

namespace crash::cases::vehicle_startup {
struct VehicleShellBinding::Data {
    Data(const VehicleShellReferences& source,VehicleShellBindingForecast f):references(source),forecast(f) {}
    VehicleShellReferences references;
    VehicleShellBindingForecast forecast;
    tl::fea::ShellBatchBinding shells;
};
VehicleShellBindingForecast ForecastShellBinding(const VehicleShellReferences& refs,VehicleShellBindingLimits limits) {
    return shell_binding_detail::Forecast(refs,limits,sizeof(VehicleShellBinding::Data)+
        sizeof(VehicleShellBinding)+64+sizeof(shell_binding_detail::Inputs)+3*sizeof(std::vector<std::size_t>)+32768);
}
VehicleShellBinding VehicleShellBinding::Prepare(const VehicleShellReferences& refs,VehicleShellBindingLimits limits) {
    const auto forecast=ForecastShellBinding(refs,limits);
    auto next=std::make_shared<Data>(refs,forecast);
    const auto inputs=shell_binding_detail::Pack(refs);
    const auto report=next->shells.InitializeFormulations(inputs.Borrow(refs.source().counts().nodes),limits.native);
    output::Require(report.status==tl::fea::ShellBindingStatus::Success,report.message);
    output::Require(next->shells.qeph_count()==refs.counts().qeph_succeeded &&
        next->shells.t3_count()==refs.counts().t3_succeeded &&
        next->shells.qbat_count()==refs.counts().qbat_succeeded &&
        next->shells.node_count()==refs.source().counts().nodes,
        "Native shell binding lost complete source coverage");
    return VehicleShellBinding(std::move(next));
}
const VehicleShellReferences& VehicleShellBinding::references() const noexcept {return data_->references;}
const tl::fea::ShellBatchBinding& VehicleShellBinding::shells() const noexcept {return data_->shells;}
const VehicleShellBindingForecast& VehicleShellBinding::forecast() const noexcept {return data_->forecast;}
}
