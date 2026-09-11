#include "Stages.h"
#include "Operations.h"
#include "../RuntimeData.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_wall {
void LoadedWall::Stages::Assemble(tl::fea::FENodalState& owner,const tl::fea::NodalTrialToken& token,
    const tl::fea::NodalAssemblyView& assembly,vehicle_dynamics::WallObservation& output) {
    const auto stamp=owner.accepted();
    output::Require(stamp.epoch<planned_intervals,"Loaded run has reached its declared fixed-step horizon");
    loaded::Assemble(contact.data_->contact,owner,token,assembly,output);
}
void LoadedWall::Stages::Evaluate(tl::fea::FENodalState& owner,const tl::fea::NodalTrialToken& token,
    const tl::fea::NodalPreparedView& prepared,const tl::fea::ShellPhysicalDiagnostics& materials,
    vehicle_dynamics::WallObservation& output) {
    loaded::Evaluate(contact.data_->contact,owner,token,prepared,materials,output);
}
void LoadedWall::Stages::Discard() noexcept { contact.data_->contact.DiscardTrial(); }
} // namespace crash::cases::vehicle_wall
