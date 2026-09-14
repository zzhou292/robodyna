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
    :startup(std::move(value)),config(c),forecast(f),timer(c.timing),
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
StepTimingSnapshot VehiclePhysicalDynamics::timing() const noexcept { return storage_->timer.snapshot(); }
tl::fea::NodalStamp VehiclePhysicalDynamics::accepted() const noexcept { return storage_->stamp; }
tl::fea::NodalAllocationInfo VehiclePhysicalDynamics::allocations() const noexcept {
    auto result=storage_->startup.allocations();
    if(storage_->wall) {
        const auto wall=storage_->wall->allocations();
        result.device_bytes+=wall.device_bytes;
        result.device_allocations+=wall.device_allocations;
    }
    if(storage_->self_contact) {
        const auto self=storage_->self_contact->allocations().device;
        result.device_bytes+=self.device_bytes;
        result.device_allocations+=self.device_allocations;
    }
    return result;
}
const vehicle_wall::VehicleWallSetup* VehiclePhysicalDynamics::wall_setup() const noexcept {
    return storage_->wall ? &storage_->wall->setup() : nullptr;
}
const vehicle_wall::RuntimeForecast* VehiclePhysicalDynamics::wall_forecast() const noexcept {
    return storage_->wall ? &storage_->wall->forecast() : nullptr;
}
const vehicle_self_contact::VehicleSelfContactSetup*
VehiclePhysicalDynamics::self_contact_setup() const noexcept {
    return storage_->self_contact ? &storage_->self_contact->setup() : nullptr;
}
const vehicle_self_contact::RuntimeForecast*
VehiclePhysicalDynamics::self_contact_forecast() const noexcept {
    return storage_->self_contact ? &storage_->self_contact->forecast() : nullptr;
}
bool VehiclePhysicalDynamics::has_prepared_step() const noexcept { return storage_->pending; }
const StepObservation& VehiclePhysicalDynamics::PrepareStep() {
    auto& s=*storage_;
    s.timer.Step([&] {
        output::Require(!s.pending,"Discard or commit the existing prepared step first");
        try {s.Prepare();s.Evaluate();s.Capture();s.pending=true;}
        catch(...) {s.Discard();throw;}
        return true;
    });
    return s.candidate();
}
void VehiclePhysicalDynamics::CommitStep() {
    auto& s=*storage_;
    output::Require(s.pending,"Physical step has not completed preparation");
    auto& state=s.state();
    const auto& view=s.prepared;
    tl::fea::ShellPublicationReport report;
    if(s.wall || s.self_contact) {
        tl::fea::ShellPhysicalScratchReceiptRoster receipts;
        if(s.wall) {
            const auto wall=s.wall->scratch_receipts();
            receipts.mapped_wall=wall.mapped_wall;
        }
        if(s.self_contact) {
            const auto self=s.self_contact->scratch_receipts();
            receipts.self_contact=self.self_contact;
        }
        report=state.publication.SealPhysicalScratchParticipation(
            state.owner,s.token,receipts);
        if(static_cast<int>(report.status)!=0) {
            s.Discard();
            detail::Require(report,"Physical scratch participation");
        }
    }
    s.timer.Measure<StepStage::Commit>([&] {
        report=state.publication.CommitPhysical(state.owner,s.token,s.candidate().mechanics,
            {view.owner_id,view.kinematics.base_epoch,view.attempt,s.config.startup.qualification_id,true});
        return report.status==tl::fea::ShellPublicationStatus::Success;
    });
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
    timer.Measure<StepStage::Discard>([&] {
        if(wall) wall->Discard();
        if(self_contact) self_contact->Discard();
        state().owner.Discard();
        state().publication.DiscardTrial();
        pending=false;
        prepared={};
        return true;
    });
}
} // namespace crash::cases::vehicle_dynamics
