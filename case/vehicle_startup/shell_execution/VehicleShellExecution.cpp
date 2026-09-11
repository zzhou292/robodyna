#include "Internal.h"

namespace crash::cases::vehicle_startup::shell_execution {
namespace {
void RequireCatalog(tl::fea::ShellPlasticityBindingReport report, const char* stage) {
    detail::Require(report.status == tl::fea::ShellPlasticityBindingStatus::Success,
                    (std::string(stage) + ": " + report.message).c_str());
}
}
VehicleShellExecution VehicleShellExecution::Prepare(const physical_model::VehiclePhysicalModel& model, Limits limits) {
    const auto forecast = Preflight(model, limits);
    const auto& resolution = *model.shell_source().references().resolution();
    detail::Packing packed;
    packed.Reserve(resolution.parts().size(), resolution.parents().size(), resolution.parts().size());
    detail::Require(packed.capacity_bytes() <= forecast.packing_bytes, "Execution vector capacities exceed forecast");
    detail::PackSource(model, packed);
    detail::Require(packed.capacity_bytes() <= forecast.packing_bytes, "Execution packing grew beyond forecast");
    tl::fea::ShellBatchPlasticityBinding catalog;
    RequireCatalog(catalog.InitializeExecutionCatalog(model.shell_source().shells(), packed.input(), limits.catalog),
                   "Complete vehicle execution catalog");
    tl::fea::ShellBatchFailureBinding failure;
    RequireCatalog(failure.InitializeExecution(catalog, packed.failure.data(), packed.failure.size(), limits.failure),
                   "Complete vehicle failure declarations");
    tl::fea::ShellExecutionBinding execution;
    RequireCatalog(execution.Initialize(catalog, model.coefficients(), model.rigid_assembly(), limits.execution),
                   "Complete vehicle PART execution");
    detail::Require(execution.counts().rigid_skin == 5102 && execution.counts().constitutive == 344543,
                    "Vehicle execution roles differ from complete original source");
    auto next = std::make_shared<Storage>(model);
    const tl::fea::ShellFormulationScope scope{&model.shell_source().shells(), &catalog, &failure, nullptr};
    const auto report = next->physical.InitializeExecution(scope, model.coefficients(), execution, limits.physical);
    detail::Require(bool(report), (std::string("Complete vehicle physical execution: ") + report.message).c_str());
    next->forecast = forecast;
    return VehicleShellExecution(std::move(next));
}
const physical_model::VehiclePhysicalModel& VehicleShellExecution::model() const noexcept { return storage_->model; }
const VehicleSectionResolution& VehicleShellExecution::resolution() const noexcept {
    return *model().shell_source().references().resolution();
}
const tl::fea::ShellBatchPlasticityBinding& VehicleShellExecution::catalog() const noexcept {
    return *storage_->physical.catalog();
}
const tl::fea::ShellBatchFailureBinding& VehicleShellExecution::failure() const noexcept {
    return *storage_->physical.failure();
}
const tl::fea::ShellExecutionBinding& VehicleShellExecution::execution() const noexcept {
    return *storage_->physical.execution();
}
const tl::fea::ShellPhysicalBinding& VehicleShellExecution::physical() const noexcept { return storage_->physical; }
const Forecast& VehicleShellExecution::forecast() const noexcept { return storage_->forecast; }
} // namespace crash::cases::vehicle_startup::shell_execution
