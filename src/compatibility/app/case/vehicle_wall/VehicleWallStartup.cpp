#include "RuntimeData.h"
#include "RuntimeBudget.h"
#include "RuntimeIdentity.h"
#include "WallParticipation.h"
#include "case/vehicle_dynamics/Storage.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_wall {
namespace c = tlfea::contact;
namespace {
c::NodalWallMappedSource ContactSource(vehicle_dynamics::ExecutionAccess::State& state) {
    return {&state.source.original_execution().physical(),&state.source.original_execution().model().rigid_assembly(),
        vehicle_runtime::detail::Witnesses(state.source.original_attachments()),&state.publication,
        {&state.qeph,&state.t3,&state.qbat,&state.type25,&state.type13,&state.solids,state.type45.get(),state.beam18.get()},
        {state.config.configuration_id,state.config.qualification_id,vehicle_runtime::detail::InitialTranslation()}};
}
}
RuntimeForecast VehicleWallStartup::Preflight(const VehicleWallSetup& setup,
    vehicle_dynamics::VehiclePhysicalDynamics& dynamics, RuntimeLimits limits) {
    output::Require(bool(dynamics.storage_),"Wall attachment requires a live dynamics owner");
    auto& state=dynamics.storage_->state();
    output::Require(!dynamics.storage_->pending,"Wall attachment requires no pending attempt");
    detail::CheckSharedSource(setup,state.source.original_execution(),state.source.original_attachments());
    output::Require(setup.coverage_report().status==c::PlanarContactStatus::Ok && setup.coverage().covered,
        "Selected finite wall does not cover the complete declared motion envelope");
    const auto source=ContactSource(state);
    const auto stamp=state.owner.accepted();
    const auto config=ContactConfig(setup.settings(),stamp,state.config.configuration_id,
        state.config.qualification_id,setup.placement());
    tl::fea::ShellMappedFootprint contact;
    const auto checked=c::NodalWallMappedContact::Forecast(config,*setup.geometry().weights(),source,contact,limits.contact);
    output::Require(checked.status==c::NodalWallDeviceStatus::Ok,checked.message);
    output::Require(dynamics.allocations().device_bytes==(dynamics.forecast().startup.device_bytes+dynamics.forecast().motion.device_bytes),
        "Actual owner allocation differs from the retained dynamics forecast");
    const auto participation=detail::ForecastWallParticipation(
        config.wall_binding_id,limits.participation);
    return detail::ComposeForecast(dynamics.forecast(),setup.forecast(),contact,participation,
        sizeof(Data)+sizeof(VehicleWallStartup)+256,limits);
}
VehicleWallStartup VehicleWallStartup::PrepareUnconfigured(
    const VehicleWallSetup& setup,
    vehicle_dynamics::VehiclePhysicalDynamics& dynamics, RuntimeLimits limits) {
    const auto forecast=Preflight(setup,dynamics,limits);
    auto& state=dynamics.storage_->state();
    auto next=std::make_unique<Data>(setup);
    next->forecast=forecast;
    next->initial_stamp=state.owner.accepted();
    const auto source=ContactSource(state);
    const auto config=ContactConfig(setup.settings(),next->initial_stamp,state.config.configuration_id,
        state.config.qualification_id,setup.placement());
    const auto prepared=next->contact.Initialize(config,setup.selected_wall_view(),*setup.geometry().weights(),
        source,state.owner,setup.placement().projected_wall_box,limits.contact);
    output::Require(prepared.status==c::NodalWallDeviceStatus::Ok,prepared.message);
    const auto entry=next->contact.roster_entry();
    output::Require(entry.issuer && entry.source_id==config.wall_binding_id,
        "Actual mapped wall did not expose its immutable roster identity");
    output::Require(next->contact.allocations().device_bytes==forecast.contact.device_bytes &&
        tl::fea::trial_identity::SameStamp(state.owner.accepted(),next->initial_stamp),
        "Wall initialization changed accepted state or disagreed with its exact forecast");
    return VehicleWallStartup(std::move(next));
}
VehicleWallStartup VehicleWallStartup::Prepare(const VehicleWallSetup& setup,
    vehicle_dynamics::VehiclePhysicalDynamics& dynamics, RuntimeLimits limits) {
    auto result=PrepareUnconfigured(setup,dynamics,limits);
    auto& state=dynamics.storage_->state();
    const auto source=ContactSource(state);
    const auto entry=result.roster_entry();
    const auto configured=state.publication.ConfigurePhysicalScratchParticipation(
        state.owner,*source.physical,source.participants,source.identity,
        {entry,{}},limits.participation);
    output::Require(configured.status==tl::fea::ShellPublicationStatus::Success,
        configured.message);
    output::Require(tl::fea::trial_identity::SameStamp(
        state.owner.accepted(),result.initial_stamp()),
        "Wall roster configuration changed accepted state");
    return result;
}
VehicleWallStartup::VehicleWallStartup(std::unique_ptr<Data> value) : data_(std::move(value)) {}
VehicleWallStartup::~VehicleWallStartup()=default;
VehicleWallStartup::VehicleWallStartup(VehicleWallStartup&&) noexcept=default;
VehicleWallStartup& VehicleWallStartup::operator=(VehicleWallStartup&&) noexcept=default;
const RuntimeForecast& VehicleWallStartup::forecast() const noexcept { return data_->forecast; }
const VehicleWallSetup& VehicleWallStartup::setup() const noexcept { return data_->setup; }
const tl::fea::NodalStamp& VehicleWallStartup::initial_stamp() const noexcept { return data_->initial_stamp; }
tl::fea::NodalAllocationInfo VehicleWallStartup::contact_allocations() const noexcept {
    return data_->contact.allocations();
}
tl::fea::ShellPhysicalScratchRosterEntry
VehicleWallStartup::roster_entry() noexcept {
    return data_ ? data_->contact.roster_entry()
                 : tl::fea::ShellPhysicalScratchRosterEntry{};
}
} // namespace crash::cases::vehicle_wall
