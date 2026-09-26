#include "Config.h"
#include "output/ArtifactIO.h"
#include <cmath>
namespace crash::cases::vehicle_runtime::detail {
void CheckConfig(const Config& config) {
    const Limits hard;
    output::Require(config.configuration_id && config.qualification_id &&
        std::isfinite(config.reserved_step_s) && config.reserved_step_s >= 1e-12 &&
        config.limits.host_bytes && config.limits.host_bytes <= hard.host_bytes &&
        config.limits.device_bytes && config.limits.device_bytes <= Limits::maximum_device_bytes,
        "Invalid bounded initial-only vehicle configuration");
}
tl::fea::ShellBatchStartup InitialTranslation() noexcept {
    return {tl::fea::ShellBatchStartupKind::ReferenceUniformTranslation,{InitialSpeedMps,0,0}};
}
tl::fea::NodalStateConfig OwnerConfig(const Config& config,std::size_t nodes) noexcept {
    tl::fea::NodalStateConfig result;
    result.node_count = nodes;
    result.max_nodes = tl::fea::MaxActiveNodalStateNodes;
    result.max_device_bytes = tl::fea::MaxActiveNodalStateDeviceBytes;
    result.fixed_dt = config.reserved_step_s;
    result.temporal_scheme = tl::fea::NodalTemporalScheme::StaggeredHalfKickStart;
    result.rigid_limits = tl::fea::NodalRigidOwnerLimits::VehicleAssembly();
    return result;
}
} // namespace crash::cases::vehicle_runtime::detail
