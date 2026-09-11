#include "Storage.h"
#include "Reports.h"
#include "lib_utils/BoundedArena.h"
#include <cmath>

namespace crash::cases::vehicle_dynamics {
Forecast VehiclePhysicalDynamics::Preflight(const vehicle_runtime::Execution& e,
    const vehicle_runtime::Attachments& a,Config config,const vehicle_runtime::JointModel* joints) {
    output::Require(tl::fea::ValidCinStructuralStep(config.structural),"Invalid physical structural timestep policy");
    output::Require(std::isfinite(config.maximum_rotation_increment) &&
        config.maximum_rotation_increment>0 && config.maximum_rotation_increment<=.2,
        "Physical free-flight rotation bound must be positive and at most 0.2 rad");
    Forecast result;
    result.startup=vehicle_runtime::VehiclePhysicalStartup::Preflight(e,a,config.startup,joints);
    const auto activity=vehicle_startup::TiedCinWitnessActivity::Forecast(a.witnesses());
    const auto n=e.model().coefficients().nodes().size();
    tl::util::BoundedArenaLayout workspace(config.workspace_bytes);
    tl::util::ArenaRegion region;
    output::Require(workspace.Append<unsigned char>(sizeof(Storage),region) &&
        workspace.Append<double>(26*n,region) &&
        workspace.Append<unsigned char>(activity.workspace_bytes,region),
        "Complete physical-step workspace exceeds its host cap");
    result.workspace_bytes=workspace.bytes();
    output::Require(result.workspace_bytes<=config.startup.limits.host_bytes &&
        result.startup.peak_host_upper_bound<=config.startup.limits.host_bytes-result.workspace_bytes,
        "Combined physical startup and step workspace exceeds host cap");
    result.peak_host_upper_bound=result.startup.peak_host_upper_bound+result.workspace_bytes;
    return result;
}
VehiclePhysicalDynamics VehiclePhysicalDynamics::Prepare(const vehicle_runtime::Execution& e,
    const vehicle_runtime::Attachments& a,Config config,const vehicle_runtime::JointModel* joints) {
    const auto forecast=Preflight(e,a,config,joints);
    auto startup=vehicle_runtime::VehiclePhysicalStartup::Prepare(e,a,config.startup,joints);
    return VehiclePhysicalDynamics(std::make_unique<Storage>(std::move(startup),config,forecast));
}
VehiclePhysicalDynamics::Storage::Storage(vehicle_runtime::VehiclePhysicalStartup&& value,Config c,Forecast f)
    :startup(std::move(value)),config(c),forecast(f),
     activity(vehicle_startup::TiedCinWitnessActivity::Create(startup.attachments().witnesses())),
     fields{Fields(startup.accepted().node_count),Fields(startup.accepted().node_count)},stamp(startup.accepted()) {
    tl::fea::NodalStamp read;
    detail::Require(state().owner.CopyAccepted(fields[0].buffer(),&read),"Initial physical fields");
    output::Require(tl::fea::trial_identity::SameStamp(stamp,read),"Initial physical snapshot stamp differs");
}
VehiclePhysicalDynamics::VehiclePhysicalDynamics(std::unique_ptr<Storage> s):storage_(std::move(s)) {}
VehiclePhysicalDynamics::~VehiclePhysicalDynamics()=default;
VehiclePhysicalDynamics::VehiclePhysicalDynamics(VehiclePhysicalDynamics&&) noexcept=default;
VehiclePhysicalDynamics& VehiclePhysicalDynamics::operator=(VehiclePhysicalDynamics&&) noexcept=default;
const Forecast& VehiclePhysicalDynamics::forecast() const noexcept { return storage_->forecast; }
tl::fea::NodalStamp VehiclePhysicalDynamics::accepted() const noexcept { return storage_->stamp; }
tl::fea::NodalAllocationInfo VehiclePhysicalDynamics::allocations() const noexcept {
    auto result=storage_->startup.allocations();
    if(storage_->wall) {
        const auto wall=storage_->wall->allocations();
        result.device_bytes+=wall.device_bytes;
        result.device_allocations+=wall.device_allocations;
    }
    return result;
}
const vehicle_wall::VehicleWallSetup* VehiclePhysicalDynamics::wall_setup() const noexcept {
    return storage_->wall ? &storage_->wall->setup() : nullptr;
}
const vehicle_wall::RuntimeForecast* VehiclePhysicalDynamics::wall_forecast() const noexcept {
    return storage_->wall ? &storage_->wall->forecast() : nullptr;
}
bool VehiclePhysicalDynamics::has_prepared_step() const noexcept { return storage_->pending; }
const StepObservation& VehiclePhysicalDynamics::PrepareStep() {
    auto& s=*storage_;
    output::Require(!s.pending,"Discard or commit the existing prepared step first");
    try { s.Prepare();s.Evaluate();s.Capture();s.pending=true; }
    catch(...) {s.Discard();throw;}
    return s.candidate();
}
void VehiclePhysicalDynamics::CommitStep() {
    auto& s=*storage_;
    output::Require(s.pending,"Physical step has not completed preparation");
    auto& state=s.state();
    const auto& view=s.prepared;
    const auto report=state.publication.CommitPhysical(state.owner,s.token,s.candidate().mechanics,
        {view.owner_id,view.kinematics.base_epoch,view.attempt,s.config.startup.qualification_id,true});
    if(static_cast<int>(report.status)!=0) { s.Discard();detail::Require(report,"Physical publication"); }
    // No allocation, device call, readback or other fallible work after success.
    s.stamp=state.owner.accepted();s.accepted_slot=1-s.accepted_slot;s.pending=false;
}
void VehiclePhysicalDynamics::DiscardStep() noexcept { storage_->Discard(); }
const StepObservation& VehiclePhysicalDynamics::last_accepted_step() const {
    output::Require(storage_->stamp.epoch!=0,"No physical interval has been accepted");
    return storage_->observations[storage_->accepted_slot];
}
void VehiclePhysicalDynamics::Storage::Discard() noexcept {
    if(wall) wall->Discard();
    state().owner.Discard();
    state().publication.DiscardTrial();
    pending=false;
    prepared={};
}
} // namespace crash::cases::vehicle_dynamics
