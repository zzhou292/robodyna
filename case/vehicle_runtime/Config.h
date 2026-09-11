#pragma once
#include "lib_src/elements/ShellBatchPublication.h"
#include "lib_src/solvers/NodalCinRuntime.h"
#include "lib_src/elements/type45/resident/Batch.h"

namespace crash::cases::vehicle_runtime {
inline constexpr double InitialSpeedMps = 35.0 * 0.44704;
struct Limits {
    std::size_t host_bytes = std::size_t{20} * 1000 * 1000 * 1000;
    std::size_t device_bytes = std::size_t{4} << 30;
    tl::fea::ShellResidentLimits shells = tl::fea::ShellResidentLimits::Vehicle();
    std::size_t shell_device_bytes = tl::fea::MaxVehicleShellResidentDeviceBytes;
    tl::fea::ShellBatchFailureLimits failure = tl::fea::ShellBatchFailureLimits::Vehicle();
    tl::fea::ShellPublicationLimits publisher = tl::fea::ShellPublicationLimits::Vehicle();
    tl::fea::type13::BatchMappedLimits beams = tl::fea::type13::BatchMappedLimits::Vehicle();
    tl::fea::type45::BatchLimits joints;
};
struct Config {
    std::uint64_t configuration_id = 0x594152495330ULL;
    std::uint64_t qualification_id = 0x494e495449414cULL;
    // Reserved constructor descriptor only. This factory grants no interval
    // admission, numerical stability certificate or app Advance operation.
    double reserved_step_s = 1e-8;
    Limits limits;
};
namespace detail {
void CheckConfig(const Config&);
tl::fea::ShellBatchStartup InitialTranslation() noexcept;
tl::fea::NodalStateConfig OwnerConfig(const Config&,std::size_t nodes) noexcept;
} // namespace detail
} // namespace crash::cases::vehicle_runtime
