#include "Storage.h"
#include "output/ArtifactIO.h"
#include "output/full_shell/FixedStepHorizon.h"
#include <cmath>
namespace crash::cases::vehicle_native_contact {
Config::Config() {
    dynamics.startup.reserved_step_s = 2e-7;
    dynamics.structural = {tl::fea::NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8, true};
    for (auto& limits : transaction) {
        limits.max_host_bytes = std::size_t{2} << 30;
        limits.max_device_bytes = std::size_t{6} << 30;
    }
    transaction[0].inventory.max_pairs = std::size_t{4} << 20;
    transaction[0].inventory.max_tasks = std::size_t{1} << 20;
    transaction[0].optimized_candidates = std::size_t{1} << 18;
    transaction[0].sliding_entries = std::size_t{1} << 20;
    transaction[1].inventory.max_pairs = std::size_t{1} << 20;
    transaction[1].inventory.max_tasks = std::size_t{1} << 19;
    transaction[1].optimized_candidates = std::size_t{1} << 16;
    transaction[1].sliding_entries = std::size_t{1} << 18;
    // A one-Q4/two-side wall has at most 2*N source pairs under the admitted
    // physical node cap. Self retains the qualified general producer ceiling.
    initialization[1].max_pairs = std::size_t{1} << 20;
    initialization[1].max_tasks = std::size_t{1} << 20;
}
PreparationError::PreparationError(const char* stage, const n::initial_source::Report& report)
    : std::runtime_error(std::string(stage) + " rejected: initial-source status " +
          std::to_string(static_cast<unsigned>(report.status))), initial(report) {}
PreparationError::PreparationError(const char* stage, const n::TransactionReport& report)
    : std::runtime_error(std::string(stage) + " rejected: " + report.message), transaction(report) {}
namespace detail {
std::size_t RoleIndex(Role role) {
    output::Require(role == Role::Self || role == Role::MeshWall, "Unknown native vehicle interface role");
    return role == Role::Self ? 0 : 1;
}
std::size_t AddBytes(std::size_t a, std::size_t b) {
    output::Require(b <= SIZE_MAX - a, "Native vehicle complete forecast overflows");
    return a + b;
}
void CheckConfig(const Config& config, const ControlsSource& controls) {
    output::Require(config.activity == n::ContactActivityPolicy::AllActivePrefix ||
                        config.activity == n::ContactActivityPolicy::ShellRemoval,
                    "Unknown native vehicle contact activity policy");
    std::uint64_t intervals = 0;
    output::Require(config.dynamics.structural.profile == tl::fea::NodalCinStructuralProfile::NativeOrdinaryRigidTrace &&
                        tl::fea::ValidCinStructuralStep(config.dynamics.structural) &&
                        output::full_shell::PlanFixedStepHorizon(config.dynamics.startup.reserved_step_s,
                            config.requested_duration_s, intervals) && intervals <= 1000000 &&
                        config.peak_device_bytes && config.peak_device_bytes <= (std::size_t{6} << 30),
                    "Invalid bounded native vehicle horizon/physical step/peak device policy");
    const auto time_scale = controls.main().provenance().units.time_s;
    const auto native_horizon = config.requested_duration_s / time_scale;
    output::Require(std::isfinite(native_horizon) && native_horizon > 0 &&
                        controls.raw_controls().sensor_disabled && controls.raw_controls().stop_nonnegative &&
                        native_horizon < controls.raw_controls().stop_time_lower_bound_native,
                    "Requested case horizon exceeds the authenticated original contact lifetime");
    const auto* wall_raw = controls.wall_raw_controls();
    const auto* wall_controls = controls.wall_controls();
    output::Require(controls.wall() && wall_raw && wall_controls &&
                        wall_controls->level == 1 && wall_controls->partitions == 1 &&
                        wall_controls->starter_workers == 1 && wall_raw->sensor_disabled &&
                        wall_raw->stop_nonnegative && native_horizon < wall_raw->stop_time_lower_bound_native,
                    "Requested case horizon exceeds the authenticated declared wall contact lifetime/profile");
    // Both source-backed lower bounds establish activity at time zero and over
    // the requested horizon. Neither is represented as an exact native TSTOP.
    for (const auto& limits : config.transaction)
        output::Require(limits.max_host_bytes && limits.max_host_bytes <= (std::size_t{2} << 30) &&
                            limits.max_device_bytes && limits.max_device_bytes <= (std::size_t{6} << 30),
                        "Individual native plan ceiling exceeds the declared complete-case allowance");
}
}
} // namespace crash::cases::vehicle_native_contact
