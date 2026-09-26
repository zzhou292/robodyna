#include "Storage.h"
#include "ScratchReceiptRoster.h"
#include "Reports.h"
#include "lib_utils/BoundedArena.h"
#include <cmath>

namespace crash::cases::vehicle_dynamics {
Forecast VehiclePhysicalDynamics::Preflight(const vehicle_runtime::Source& source,Config config) {
    output::Require(tl::fea::ValidCinStructuralStep(config.structural),"Invalid physical structural timestep policy");
    output::Require(std::isfinite(config.maximum_rotation_increment) &&
        config.maximum_rotation_increment>0 && config.maximum_rotation_increment<=.2,
        "Physical free-flight rotation bound must be positive and at most 0.2 rad");
    Forecast result;
    result.startup=vehicle_runtime::VehiclePhysicalStartup::Preflight(source,config.startup);
    const auto activity=vehicle_startup::TiedCinWitnessActivity::Forecast(source.witnesses());
    const auto n=source.coefficients().nodes().size();
    result.motion=tl::fea::NodalUniformMotionObserver::Preflight(n,config.motion_limits);
    detail::Require(result.motion.report,"Physical motion observation capacity");
    output::Require(result.motion.device_bytes<=config.startup.limits.device_bytes &&
        result.startup.device_bytes<=config.startup.limits.device_bytes-result.motion.device_bytes,
        "Complete physical step and observation exceed device cap");
    tl::util::BoundedArenaLayout workspace(config.workspace_bytes);
    tl::util::ArenaRegion region;
    output::Require(workspace.Append<unsigned char>(sizeof(Storage),region) &&
        workspace.Append<unsigned char>(result.motion.host_bytes,region) &&
        workspace.Append<unsigned char>(activity.workspace_bytes,region),
        "Complete physical-step workspace exceeds its host cap");
    result.workspace_bytes=workspace.bytes();
    output::Require(result.workspace_bytes<=config.startup.limits.host_bytes &&
        result.startup.peak_host_upper_bound<=config.startup.limits.host_bytes-result.workspace_bytes,
        "Combined physical startup and step workspace exceeds host cap");
    result.peak_host_upper_bound=result.startup.peak_host_upper_bound+result.workspace_bytes;
    return result;
}
Forecast VehiclePhysicalDynamics::Preflight(const vehicle_runtime::Execution& execution,
    const vehicle_runtime::Attachments& attachments,Config config,const vehicle_runtime::JointModel* joints) {
    return Preflight(vehicle_runtime::Source::Original(execution,attachments,joints),config);
}
VehiclePhysicalDynamics VehiclePhysicalDynamics::Prepare(const vehicle_runtime::Execution& execution,
    const vehicle_runtime::Attachments& attachments,Config config,const vehicle_runtime::JointModel* joints) {
    return Prepare(vehicle_runtime::Source::Original(execution,attachments,joints),config);
}
VehiclePhysicalDynamics VehiclePhysicalDynamics::Prepare(const vehicle_runtime::Source& source,Config config) {
    const auto forecast=Preflight(source,config);
    auto startup=vehicle_runtime::VehiclePhysicalStartup::Prepare(source,config.startup);
    return VehiclePhysicalDynamics(std::make_unique<Storage>(std::move(startup),config,forecast));
}
VehiclePhysicalDynamics::Storage::Storage(vehicle_runtime::VehiclePhysicalStartup&& value,Config c,Forecast f)
    :startup(std::move(value)),config(c),forecast(f),timer(c.timing),
     activity(vehicle_startup::TiedCinWitnessActivity::Create(startup.source().witnesses())),
     stamp(startup.accepted()) {
    // Startup already authenticates original owner coordinates against the
    // complete physical domain. The observer captures that same initial X once.
    detail::Require(motion.Initialize(state().owner,config.motion_limits),"Initial physical motion reference");
    output::Require(motion.allocations().device_bytes==forecast.motion.device_bytes &&
        motion.forecast().host_bytes==forecast.motion.host_bytes,
        "Physical motion observation allocation differs from preflight");
}
VehiclePhysicalDynamics::VehiclePhysicalDynamics(std::unique_ptr<Storage> s):storage_(std::move(s)) {}
VehiclePhysicalDynamics::~VehiclePhysicalDynamics()=default;
VehiclePhysicalDynamics::VehiclePhysicalDynamics(VehiclePhysicalDynamics&&) noexcept=default;
VehiclePhysicalDynamics& VehiclePhysicalDynamics::operator=(VehiclePhysicalDynamics&&) noexcept=default;
const Forecast& VehiclePhysicalDynamics::forecast() const noexcept { return storage_->forecast; }
StepTimingSnapshot VehiclePhysicalDynamics::timing() const noexcept { return storage_->timer.snapshot(); }
tl::fea::NodalStamp VehiclePhysicalDynamics::accepted() const noexcept { return storage_->stamp; }
AllocationInfo VehiclePhysicalDynamics::allocations() const noexcept {
    const auto physical=storage_->startup.allocations();
    AllocationInfo result{physical.device_bytes,physical.device_allocations,true};
    const auto observation=storage_->motion.allocations();
    result.device_bytes+=observation.device_bytes;
    result.device_allocations+=observation.device_allocations;
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
    if(storage_->native_contact) {
        result.device_bytes+=storage_->native_contact->device_bytes();
        result.device_allocation_count_complete=false;
    }
    return result;
}
const native_contact::Group* VehiclePhysicalDynamics::native_contact_group() const noexcept {
    return storage_->native_contact.get();
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
tlfea::contact::SelfContactTransactionAllocationInfo
VehiclePhysicalDynamics::self_contact_allocations() const noexcept {
    return storage_->self_contact
        ? storage_->self_contact->allocations()
        : tlfea::contact::SelfContactTransactionAllocationInfo{};
}
tlfea::contact::SelfContactTransactionDiagnostics
VehiclePhysicalDynamics::self_contact_diagnostics() const noexcept {
    return storage_->self_contact ? storage_->self_contact->diagnostics()
                                  : tlfea::contact::SelfContactTransactionDiagnostics{};
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
    if(s.native_contact) {
        report=state.publication.SealPhysicalScratchParticipation(state.owner,s.token,s.native_contact->scratch_receipts());
        if(report.status!=tl::fea::ShellPublicationStatus::Success) {
            s.Discard();
            detail::Require(report,"Native group scratch participation");
        }
    } else if(s.wall || s.self_contact) {
        const auto wall=s.wall?s.wall->scratch_receipts():
            tl::fea::ShellPhysicalScratchReceiptRoster{};
        const auto self_contact=s.self_contact?
            s.self_contact->scratch_receipts():
            tl::fea::ShellPhysicalScratchReceiptRoster{};
        const auto receipts=detail::ComposeScratchReceipts(
            wall,self_contact);
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
    if(s.native_contact)s.native_contact->Committed();
    s.stamp=state.owner.accepted();s.accepted_slot=1-s.accepted_slot;s.pending=false;
}
void VehiclePhysicalDynamics::DiscardStep() noexcept { storage_->Discard(); }
const StepObservation& VehiclePhysicalDynamics::last_accepted_step() const {
    output::Require(storage_->stamp.epoch!=0,"No physical interval has been accepted");
    return storage_->observations[storage_->accepted_slot];
}
void VehiclePhysicalDynamics::Storage::Discard() noexcept {
    timer.Measure<StepStage::Discard>([&] {
        if(native_contact) native_contact->Discard();
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
