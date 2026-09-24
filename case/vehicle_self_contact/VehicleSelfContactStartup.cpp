#include "RuntimeBudget.h"
#include "RuntimeData.h"

#include "case/vehicle_dynamics/Storage.h"
#include "case/vehicle_runtime/ParticipantConfigs.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include "output/ArtifactIO.h"

#include <cmath>

namespace crash::cases::vehicle_self_contact {
namespace {
namespace c = tlfea::contact;

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

tl::fea::ShellPhysicalPublicationIdentity Identity(
    const vehicle_dynamics::Config& config) noexcept {
    return {config.startup.configuration_id,
            config.startup.qualification_id,
            vehicle_runtime::detail::InitialTranslation()};
}

void CheckProfile(const VehicleSelfContactSetup& setup,
                  RuntimeConfig config,
                  const RuntimeLimits& limits) {
    const auto& coefficients = setup.census().runtime_coefficients;
    output::Require(
        setup.config().facet_level == 0 && config.source_id &&
            config.event_capacity && config.broadphase_axis < 3 &&
            std::isfinite(FirstProfileStiffnessPerAreaNPerM3) &&
            FirstProfileStiffnessPerAreaNPerM3 == 2e9 &&
            !coefficients.applied_source_friction_fields &&
            !coefficients.applied_source_damping_fields &&
            !coefficients.applied_source_soft_fields,
        "Self-contact runtime requires the fixed frictionless level-0 first profile");
    const auto& transaction = limits.transaction;
    output::Require(
        transaction.max_candidate_triangles &&
            transaction.max_candidate_pairs &&
            transaction.max_host_bytes &&
            transaction.max_device_bytes &&
            transaction.max_startup_host_bytes &&
            config.event_capacity <= transaction.max_global_events &&
            transaction.force.max_events ==
                transaction.max_global_events &&
            transaction.max_event_hash_slots >=
                transaction.max_global_events &&
            transaction.activity.max_selected_parents &&
            transaction.activity.max_family_parents &&
            transaction.force.max_events &&
            transaction.force.max_nodes &&
            transaction.broadphase.max_parents &&
            transaction.broadphase.max_nodes &&
            transaction.broadphase.max_pairs &&
            transaction.accepted_discovery.max_input_pairs &&
            transaction.candidate_discovery.max_input_pairs &&
            transaction.regularity.max_parents &&
            transaction.regularity.max_facets &&
            transaction.crossing.max_paths &&
            transaction.crossing.max_input_pairs &&
            transaction.crossing.max_results &&
            transaction.participation.max_host_bytes,
        "Self-contact runtime requires explicit nonzero transaction capacities");
}

RuntimeIdentity RuntimeSourceIdentity(
    const VehicleSelfContactSetup& setup, RuntimeConfig config,
    const tl::fea::NodalStamp& owner,
    const tl::fea::ShellPhysicalPublicationIdentity& publication) noexcept {
    return {
        config.source_id,
        owner.owner_id,
        publication.configuration_id,
        publication.qualification_id,
        setup.physical().domain()->source_instance_id(),
        setup.active_uses().identity(),
        1};
}

void CheckSource(
    const VehicleSelfContactSetup& setup,
    vehicle_dynamics::ExecutionAccess::State& state) {
    output::Require(
        &setup.execution().physical() == &state.execution.physical() &&
            &setup.attachments().witnesses() ==
                &state.attachments.witnesses() &&
            setup.selected().MatchesPhysical(state.execution.physical()),
        "Self-contact setup must retain the exact dynamics physical source");
}

void Check(const c::SelfContactTransactionReport& report) {
    output::Require(
        report.status == c::SelfContactTransactionStatus::Ok,
        report.message);
}

}  // namespace

RuntimeForecast VehicleSelfContactStartup::Preview(
    const VehicleSelfContactSetup& setup,
    vehicle_dynamics::Config dynamics_config,
    RuntimeConfig config, RuntimeLimits limits,
    const vehicle_runtime::JointModel* joints) {
    CheckProfile(setup, config, limits);
    const auto dynamics = vehicle_dynamics::VehiclePhysicalDynamics::Preflight(
        setup.execution(), setup.attachments(), dynamics_config, joints);
    const auto identity = Identity(dynamics_config);
    const auto owner = vehicle_runtime::detail::DescriptiveStamp(
        dynamics_config.startup, setup.execution());
    const auto transaction = c::SelfContactTransaction::Forecast(
        detail::TransactionConfig(config, owner, identity),
        setup.active_uses(), identity, limits.transaction);
    Check(transaction.report);
    auto result = detail::ComposeForecast(
        dynamics, setup.forecast(), transaction.forecast,
        sizeof(Data) + sizeof(VehicleSelfContactStartup) + 256, limits);
    result.identity =
        RuntimeSourceIdentity(setup, config, owner, identity);
    return result;
}

RuntimeForecast VehicleSelfContactStartup::Preflight(
    const VehicleSelfContactSetup& setup,
    vehicle_dynamics::VehiclePhysicalDynamics& dynamics,
    RuntimeConfig config, RuntimeLimits limits) {
    CheckProfile(setup, config, limits);
    output::Require(
        bool(dynamics.storage_),
        "Self-contact attachment requires a live dynamics owner");
    auto& state = dynamics.storage_->state();
    output::Require(
        !dynamics.storage_->pending && state.owner.accepted().epoch == 0,
        "Self-contact attachment requires a fresh owner with no pending attempt");
    CheckSource(setup, state);
    const auto source = Source(state);
    const auto transaction = c::SelfContactTransaction::Forecast(
        detail::TransactionConfig(config, state.owner.accepted(), source.identity),
        setup.active_uses(), source.identity, limits.transaction);
    Check(transaction.report);
    output::Require(
        dynamics.allocations().device_bytes ==
            dynamics.forecast().startup.device_bytes,
        "Actual owner allocation differs from retained dynamics forecast");
    auto result = detail::ComposeForecast(
        dynamics.forecast(), setup.forecast(), transaction.forecast,
        sizeof(Data) + sizeof(VehicleSelfContactStartup) + 256, limits);
    result.identity = RuntimeSourceIdentity(
        setup, config, state.owner.accepted(), source.identity);
    return result;
}

VehicleSelfContactStartup
VehicleSelfContactStartup::PrepareUnconfigured(
    const VehicleSelfContactSetup& setup,
    vehicle_dynamics::VehiclePhysicalDynamics& dynamics,
    RuntimeConfig config, RuntimeLimits limits) {
    const auto forecast = Preflight(setup, dynamics, config, limits);
    auto& state = dynamics.storage_->state();
    const auto source = Source(state);
    auto next = std::make_unique<Data>(setup);
    next->forecast = forecast;
    next->initial_stamp = state.owner.accepted();
    cudaStream_t owner_stream = nullptr;
    const auto borrowed = state.owner.BorrowOwnerStream(&owner_stream);
    output::Require(
        borrowed.status == tl::fea::NodalStatus::Ok,
        borrowed.message);
    Check(next->transaction.Initialize(
        detail::TransactionConfig(config, next->initial_stamp, source.identity),
        setup.active_uses(), state.owner, state.publication,
        *source.physical, source.participants, source.identity,
        owner_stream, limits.transaction));
    const auto entry = next->transaction.roster_entry();
    const auto allocations = next->transaction.allocations();
    const auto mode = next->transaction.facet_filter_initialization();
    c::SelfContactTransactionPreflight cpu;
    const c::SelfContactTransactionForecast* cpu_fallback = nullptr;
    if (config.enable_cuda_facet_filters &&
        mode == c::SelfContactFacetFilterInitialization::UnsupportedHostArithmetic) {
        auto cpu_config = config;
        cpu_config.enable_cuda_facet_filters = false;
        cpu = c::SelfContactTransaction::Forecast(
            detail::TransactionConfig(cpu_config, next->initial_stamp, source.identity),
            setup.active_uses(), source.identity, limits.transaction);
        Check(cpu.report);
        cpu_fallback = &cpu.forecast;
    }
    output::Require(
        entry.issuer && entry.source_id == config.source_id &&
            detail::TransactionAllocationsMatch(config, mode, forecast.transaction,
                allocations, cpu_fallback) &&
            tl::fea::trial_identity::SameStamp(
                state.owner.accepted(), next->initial_stamp),
        "Self-contact initialization changed accepted state or disagreed with forecast");
    next->forecast.filter_initialization = mode;
    return VehicleSelfContactStartup(std::move(next));
}

VehicleSelfContactStartup VehicleSelfContactStartup::Prepare(
    const VehicleSelfContactSetup& setup,
    vehicle_dynamics::VehiclePhysicalDynamics& dynamics,
    RuntimeConfig config, RuntimeLimits limits) {
    auto result =
        PrepareUnconfigured(setup, dynamics, config, limits);
    auto& state = dynamics.storage_->state();
    const auto source = Source(state);
    const auto configured =
        state.publication.ConfigurePhysicalScratchParticipation(
            state.owner, *source.physical, source.participants,
            source.identity, {{}, result.roster_entry()},
            limits.transaction.participation);
    output::Require(
        configured.status == tl::fea::ShellPublicationStatus::Success,
        configured.message);
    output::Require(
        tl::fea::trial_identity::SameStamp(
            state.owner.accepted(), result.initial_stamp()),
        "Self-contact roster configuration changed accepted state");
    return result;
}

VehicleSelfContactStartup::VehicleSelfContactStartup(
    std::unique_ptr<Data> value) : data_(std::move(value)) {}
VehicleSelfContactStartup::~VehicleSelfContactStartup() = default;
VehicleSelfContactStartup::VehicleSelfContactStartup(
    VehicleSelfContactStartup&&) noexcept = default;
VehicleSelfContactStartup& VehicleSelfContactStartup::operator=(
    VehicleSelfContactStartup&&) noexcept = default;

const RuntimeForecast&
VehicleSelfContactStartup::forecast() const noexcept {
    return data_->forecast;
}
const VehicleSelfContactSetup&
VehicleSelfContactStartup::setup() const noexcept {
    return data_->setup;
}
const tl::fea::NodalStamp&
VehicleSelfContactStartup::initial_stamp() const noexcept {
    return data_->initial_stamp;
}
tlfea::contact::SelfContactTransactionAllocationInfo
VehicleSelfContactStartup::allocations() const noexcept {
    return data_->transaction.allocations();
}
tl::fea::ShellPhysicalScratchRosterEntry
VehicleSelfContactStartup::roster_entry() noexcept {
    return data_ ? data_->transaction.roster_entry()
                 : tl::fea::ShellPhysicalScratchRosterEntry{};
}

}  // namespace crash::cases::vehicle_self_contact
