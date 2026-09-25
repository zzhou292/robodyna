#include "Internal.h"

namespace crash::cases::vehicle_startup::shell_execution {

Forecast VehicleShellExecution::Preflight(const physical_model::VehiclePhysicalModel& model, Limits limits) {
    return Preflight(model,Law1ExecutionProfile::LegacyLayered,limits);
}
Forecast VehicleShellExecution::Preflight(const physical_model::VehiclePhysicalModel& model,Law1ExecutionProfile profile,Limits limits) {
    detail::CheckLimits(limits);
    const auto& resolution = detail::CheckSource(model);
    (void)detail::ResolvePolicy(model,profile);
    const auto nodes = model.source_domain().domain().node_count();
    detail::Require(nodes <= limits.execution.max_nodes && nodes <= limits.physical.max_nodes &&
        model.shell_source().shells().node_count() <= limits.catalog.max_nodes,
        "Complete shell execution node domain exceeds native bounds");
    // Exact app objects plus explicit bounded shared-control/allocation allowance.
    const auto fixed = sizeof(VehicleShellExecution) + sizeof(Storage) + sizeof(detail::Packing) +
        sizeof(tl::fea::ShellBatchPlasticityBinding) + sizeof(tl::fea::ShellBatchFailureBinding) +
        sizeof(tl::fea::ShellExecutionBinding) + 6 * 64;
    return detail::ForecastPayload(model.forecast().total_bytes, fixed, resolution.parts().size(),
                                  resolution.parents().size(), resolution.parts().size(), limits);
}
} // namespace crash::cases::vehicle_startup::shell_execution
