#include "Storage.h"
#include "Reports.h"
#include "lib_utils/BoundedArena.h"
namespace crash::cases::vehicle_runtime {
VehiclePhysicalStartup::VehiclePhysicalStartup(std::unique_ptr<Storage> value) : storage_(std::move(value)) {}
VehiclePhysicalStartup::~VehiclePhysicalStartup() = default;
VehiclePhysicalStartup::VehiclePhysicalStartup(VehiclePhysicalStartup&&) noexcept = default;
VehiclePhysicalStartup& VehiclePhysicalStartup::operator=(VehiclePhysicalStartup&&) noexcept = default;
Forecast VehiclePhysicalStartup::Preflight(const Execution& execution,const Attachments& attachments,Config config,
    const JointModel* joints) {
    return detail::ForecastStartup(config,execution,attachments,sizeof(Storage)+sizeof(VehiclePhysicalStartup),joints);
}
VehiclePhysicalStartup VehiclePhysicalStartup::Prepare(const Execution& execution,
    const Attachments& attachments,Config config,const JointModel* joints) {
    const auto forecast = Preflight(execution,attachments,config,joints);
    auto staged = std::make_unique<Storage>(execution,attachments,config,forecast,joints);
    staged->roles = ResolveSourceRoles(attachments);
    output::Require(staged->roles.capacity_bytes() <= execution.physical().domain()->node_count(),
                    "Source role capacity exceeds forecast");
    staged->InitializeOwner();
    staged->InitializeParticipants();
    staged->InitializeJoints();
    staged->BindInitialCaches();
    staged->InitializePublication();
    output::Require(staged->Allocations().device_bytes == forecast.device_bytes,
                    "Actual native allocations disagree with complete startup forecast");
    return VehiclePhysicalStartup(std::move(staged));
}
const Forecast& VehiclePhysicalStartup::forecast() const noexcept { return storage_->forecast; }
const Execution& VehiclePhysicalStartup::execution() const noexcept { return storage_->execution; }
const Attachments& VehiclePhysicalStartup::attachments() const noexcept { return storage_->attachments; }
tl::fea::NodalStamp VehiclePhysicalStartup::accepted() const noexcept { return storage_->owner.accepted(); }
tl::fea::NodalAllocationInfo VehiclePhysicalStartup::allocations() const noexcept { return storage_->Allocations(); }
tl::fea::NodalAllocationInfo VehiclePhysicalStartup::Storage::Allocations() const noexcept {
    tl::fea::NodalAllocationInfo out;
    for (auto value : {owner.allocations(),qeph.allocations(),t3.allocations(),qbat.allocations(),
                      type25.allocations(),type13.allocations(),solids.allocations(),publication.allocations()}) {
        out.device_bytes += value.device_bytes;
        out.device_allocations += value.device_allocations;
    }
    if(type45) {
        out.device_bytes += type45->allocations().device_bytes;
        out.device_allocations += type45->allocations().device_allocations;
    }
    return out;
}
InitialInspection VehiclePhysicalStartup::InspectInitial() {
    InitialInspection out;
    out.stamp = storage_->owner.accepted();
    output::Require(out.stamp.epoch==0 && out.stamp.time==0 && !out.stamp.reactions_valid,
                    "Initial startup inspection requires unchanged epoch zero");
    out.allocations = storage_->Allocations();
    storage_->InspectOwner(out);
    storage_->InspectShells(out);
    storage_->InspectConnections(out);
    storage_->InspectSolids(out);
    storage_->InspectJoints(out);
    tl::fea::ShellPhysicalDiagnostics diagnostics;
    detail::RequireSuccess(storage_->publication.CopyAcceptedPhysicalDiagnostics(out.stamp,&diagnostics));
    output::Require(diagnostics.valid && !diagnostics.kinetic_available,
                    "Initial common publication diagnostics are invalid");
    output::Require(diagnostics.has_type45==bool(storage_->type45),
                    "Initial publication joint availability differs from its participant scope");
    output::Require(storage_->owner.accepted().epoch==0 && storage_->owner.accepted().time==0 &&
                    storage_->Allocations().device_bytes==out.allocations.device_bytes,
                    "Initial readback changed clock or explicit allocations");
    return out;
}
} // namespace crash::cases::vehicle_runtime
