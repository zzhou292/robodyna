#include "Operations.h"
#include "../WallStageError.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_wall::loaded {
namespace {
void Check(const tlfea::contact::NodalWallDeviceReport& report,const char* stage) {
    if(report.status!=tlfea::contact::NodalWallDeviceStatus::Ok) {
        throw WallStageError(report,stage);
    }
}
}
void Assemble(tlfea::contact::NodalWallMappedContact& contact,tl::fea::FENodalState& owner,
    const tl::fea::NodalTrialToken& token,const tl::fea::NodalAssemblyView& assembly,
    vehicle_dynamics::WallObservation& output) {
    vehicle_dynamics::WallObservation next;
    Check(contact.AssembleAccepted(owner,token,assembly,&next.accepted),
        "Accepted finite mesh wall force and CIN stiffness");
    next.enabled=true;
    output=next;
}
void Evaluate(tlfea::contact::NodalWallMappedContact& contact,tl::fea::FENodalState& owner,
    const tl::fea::NodalTrialToken& token,const tl::fea::NodalPreparedView& prepared,
    const tl::fea::ShellPhysicalDiagnostics& materials,vehicle_dynamics::WallObservation& output) {
    const auto& base=output.accepted.contact;
    output::Require(output.enabled && output.accepted.valid && base.valid &&
        base.phase==tlfea::contact::NodalWallDevicePhase::AcceptedBase &&
        base.owner_id==prepared.owner_id && base.base_epoch==prepared.kinematics.base_epoch &&
        base.attempt==prepared.attempt,
        "Prepared wall stage requires the same attempt's accepted contact observation");
    auto next=output;
    Check(contact.EvaluateCandidate(owner,token,prepared,materials,&next.prepared),
        "Prepared finite mesh wall work and activity");
    output=next;
}
} // namespace crash::cases::vehicle_wall::loaded
