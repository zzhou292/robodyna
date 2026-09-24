#include "../SelfContactFactories.h"

#include "../RuntimeBudget.h"
#include "../RuntimeData.h"
#include "Stages.h"
#include "case/vehicle_dynamics/Storage.h"
#include "case/vehicle_runtime/ParticipantConfigs.h"
#include "case/vehicle_wall/loaded/Stages.h"
#include "output/ArtifactIO.h"
#include "output/full_shell/FixedStepHorizon.h"

namespace crash::cases::vehicle_self_contact {
namespace {

struct PhysicalSource {
    const tl::fea::ShellPhysicalBinding* physical = nullptr;
    tl::fea::ShellPhysicalParticipants participants;
    tl::fea::ShellPhysicalPublicationIdentity identity;
};

PhysicalSource Source(
    vehicle_dynamics::ExecutionAccess::State& state) noexcept {
    return {
        &state.execution.physical(),
        {&state.qeph, &state.t3, &state.qbat, &state.type25,
         &state.type13, &state.solids, state.type45.get(),
         state.beam18.get()},
        {state.config.configuration_id, state.config.qualification_id,
         vehicle_runtime::detail::InitialTranslation()}};
}

RuntimeForecast AddSelfStages(
    RuntimeForecast result, RuntimeLimits limits) {
    const auto extra = sizeof(detail::SelfContactStages) + 256;
    output::Require(
        extra <= limits.host_bytes &&
            result.peak_host_upper_bound <= limits.host_bytes - extra,
        "Self-contact stage storage exceeds the complete host cap");
    result.retained_host_upper_bound += extra;
    result.peak_host_upper_bound += extra;
    return result;
}

WallSelfContactLimits Normalize(WallSelfContactLimits limits) {
    output::Require(
        limits.host_bytes &&
            limits.host_bytes <=
                std::size_t{20} * 1000 * 1000 * 1000 &&
            limits.device_bytes &&
            limits.device_bytes <= (std::size_t{8} << 30),
        "Invalid wall+self runtime cap");
    limits.wall.host_bytes = limits.host_bytes;
    limits.wall.device_bytes = limits.device_bytes;
    limits.wall.participation = limits.participation;
    limits.self_contact.host_bytes = limits.host_bytes;
    limits.self_contact.device_bytes = limits.device_bytes;
    limits.self_contact.transaction.participation =
        limits.participation;
    return limits;
}

tl::fea::ShellPhysicalScratchParticipationForecast
CombinedParticipation(
    std::uint64_t wall_source, std::uint64_t self_source,
    tl::fea::ShellPhysicalScratchParticipationLimits limits) {
    output::Require(
        wall_source && self_source && wall_source != self_source,
        "Wall and self-contact require distinct fixed roster identities");
    tl::fea::ShellPhysicalScratchParticipation wall;
    tl::fea::ShellPhysicalScratchParticipation self;
    tl::fea::ShellPhysicalScratchParticipationForecast result;
    const auto report =
        tl::fea::ShellBatchPublication::
            ForecastPhysicalScratchParticipation(
                {{&wall, wall_source}, {&self, self_source}},
                limits, result);
    output::Require(
        report.status == tl::fea::ShellPublicationStatus::Success,
        report.message);
    return result;
}

WallSelfContactForecast ComposeCombined(
    const vehicle_wall::VehicleWallSetup& wall_setup,
    const VehicleSelfContactSetup& self_setup,
    vehicle_wall::RuntimeForecast wall,
    RuntimeForecast self_contact,
    RuntimeConfig config, WallSelfContactLimits limits,
    std::size_t self_fixed) {
    output::Require(
        &wall_setup.execution().physical() ==
                &self_setup.execution().physical() &&
            &wall_setup.attachments().witnesses() ==
                &self_setup.attachments().witnesses(),
        "Wall and self-contact setups must retain the same vehicle source");
    const auto participation = CombinedParticipation(
        wall_setup.settings().wall_binding_id, config.source_id,
        limits.participation);
    const auto budget = detail::ComposeCombinedBudget(
        wall.retained_host_upper_bound,
        wall.peak_host_upper_bound,
        wall.device_bytes,
        wall.participation.publication_host_bytes,
        self_setup.forecast(), self_contact, participation,
        self_fixed, limits.host_bytes, limits.device_bytes);
    WallSelfContactForecast result;
    result.wall = wall;
    result.self_contact = self_contact;
    result.participation = participation;
    result.retained_host_upper_bound =
        budget.retained_host_upper_bound;
    result.peak_host_upper_bound =
        budget.peak_host_upper_bound;
    result.device_bytes = budget.device_bytes;
    return result;
}

}  // namespace

RuntimeForecast SelfContactOnly::Preflight(
    const VehicleSelfContactSetup& setup,
    vehicle_dynamics::Config dynamics_config,
    RuntimeConfig config, RuntimeLimits limits,
    const vehicle_runtime::JointModel* joints) {
    return AddSelfStages(
        VehicleSelfContactStartup::Preview(
            setup, dynamics_config, config, limits, joints),
        limits);
}

vehicle_dynamics::VehiclePhysicalDynamics
SelfContactOnly::Prepare(
    const VehicleSelfContactSetup& setup,
    vehicle_dynamics::Config dynamics_config,
    RuntimeConfig config, RuntimeLimits limits,
    const vehicle_runtime::JointModel* joints) {
    const auto forecast =
        Preflight(setup, dynamics_config, config, limits, joints);
    auto dynamics =
        vehicle_dynamics::VehiclePhysicalDynamics::Prepare(
            setup.execution(), setup.attachments(),
            dynamics_config, joints);
    auto contact =
        VehicleSelfContactStartup::PrepareUnconfigured(
            setup, dynamics, config, limits);
    const auto entry = contact.roster_entry();
    auto installed_forecast = forecast;
    installed_forecast.identity = contact.forecast().identity;
    installed_forecast.filter_initialization = contact.forecast().filter_initialization;
    auto stages = std::make_unique<detail::SelfContactStages>(
        std::move(contact), installed_forecast);
    auto& state = dynamics.storage_->state();
    const auto source = Source(state);
    const auto configured =
        state.publication.ConfigurePhysicalScratchParticipation(
            state.owner, *source.physical, source.participants,
            source.identity, {{}, entry},
            limits.transaction.participation);
    output::Require(
        configured.status == tl::fea::ShellPublicationStatus::Success,
        configured.message);
    dynamics.storage_->self_contact = std::move(stages);
    dynamics.storage_->forecast.peak_host_upper_bound =
        forecast.peak_host_upper_bound;
    output::Require(
        dynamics.allocations().device_bytes ==
            detail::EffectiveCombinedDeviceBytes(forecast.device_bytes,
                forecast.transaction.device_bytes, dynamics.self_contact_allocations().device.device_bytes),
        "Self-contact factory allocation differs from exact forecast");
    return dynamics;
}

WallSelfContactForecast LoadedWallSelfContact::Preflight(
    const vehicle_wall::VehicleWallSetup& wall_setup,
    const VehicleSelfContactSetup& self_setup,
    RuntimeConfig config, WallSelfContactLimits limits,
    vehicle_dynamics::Config dynamics_config,
    const vehicle_runtime::JointModel* joints) {
    limits = Normalize(limits);
    output::Require(
        &wall_setup.execution().physical() ==
                &self_setup.execution().physical() &&
            &wall_setup.attachments().witnesses() ==
                &self_setup.attachments().witnesses(),
        "Loaded wall+self setups do not share one physical source");
    const auto wall = vehicle_wall::LoadedWall::Preflight(
        wall_setup, dynamics_config, limits.wall, joints);
    const auto self_contact = SelfContactOnly::Preflight(
        self_setup, dynamics_config, config,
        limits.self_contact, joints);
    return ComposeCombined(
        wall_setup, self_setup, wall, self_contact,
        config, limits,
        sizeof(VehicleSelfContactStartup::Data) +
            sizeof(VehicleSelfContactStartup) + 256 +
            sizeof(detail::SelfContactStages) + 256);
}

vehicle_dynamics::VehiclePhysicalDynamics
LoadedWallSelfContact::Prepare(
    const vehicle_wall::VehicleWallSetup& wall_setup,
    const VehicleSelfContactSetup& self_setup,
    RuntimeConfig config, WallSelfContactLimits limits,
    vehicle_dynamics::Config dynamics_config,
    const vehicle_runtime::JointModel* joints) {
    limits = Normalize(limits);
    const auto forecast = Preflight(
        wall_setup, self_setup, config, limits,
        dynamics_config, joints);
    std::uint64_t intervals = 0;
    output::Require(
        output::full_shell::PlanFixedStepHorizon(
            dynamics_config.startup.reserved_step_s,
            wall_setup.settings().requested_duration_s,
            intervals),
        "Loaded wall+self fixed horizon is unrepresentable");
    auto dynamics =
        vehicle_dynamics::VehiclePhysicalDynamics::Prepare(
            wall_setup.execution(), wall_setup.attachments(),
            dynamics_config, joints);
    auto wall =
        vehicle_wall::VehicleWallStartup::PrepareUnconfigured(
            wall_setup, dynamics, limits.wall);
    auto self_contact =
        VehicleSelfContactStartup::PrepareUnconfigured(
            self_setup, dynamics, config, limits.self_contact);
    const auto wall_entry = wall.roster_entry();
    const auto self_entry = self_contact.roster_entry();
    auto wall_stages =
        std::make_unique<vehicle_wall::LoadedWall::Stages>(
            std::move(wall), forecast.wall, intervals);
    auto installed_self_forecast = forecast.self_contact;
    installed_self_forecast.identity =
        self_contact.forecast().identity;
    installed_self_forecast.filter_initialization = self_contact.forecast().filter_initialization;
    auto self_stages =
        std::make_unique<detail::SelfContactStages>(
            std::move(self_contact), installed_self_forecast);

    auto& state = dynamics.storage_->state();
    const auto source = Source(state);
    const auto configured =
        state.publication.ConfigurePhysicalScratchParticipation(
            state.owner, *source.physical, source.participants,
            source.identity, {wall_entry, self_entry},
            limits.participation);
    output::Require(
        configured.status == tl::fea::ShellPublicationStatus::Success,
        configured.message);
    dynamics.storage_->wall = std::move(wall_stages);
    dynamics.storage_->self_contact = std::move(self_stages);
    dynamics.storage_->forecast.peak_host_upper_bound =
        forecast.peak_host_upper_bound;
    output::Require(
        dynamics.allocations().device_bytes ==
            detail::EffectiveCombinedDeviceBytes(forecast.device_bytes,
                forecast.self_contact.transaction.device_bytes, dynamics.self_contact_allocations().device.device_bytes),
        "Wall+self factory allocation differs from exact forecast");
    return dynamics;
}

}  // namespace crash::cases::vehicle_self_contact
