#pragma once
#include "case/vehicle_dynamics/WallObservation.h"
namespace crash::cases::vehicle_wall::loaded {
// Shared concrete TL operations for the actual app owner and small owning
// mixed-family CUDA trajectory fixture. These never commit an accepted state.
void Assemble(tlfea::contact::NodalWallMappedContact&,tl::fea::FENodalState&,
    const tl::fea::NodalTrialToken&,const tl::fea::NodalAssemblyView&,vehicle_dynamics::WallObservation&);
void Evaluate(tlfea::contact::NodalWallMappedContact&,tl::fea::FENodalState&,
    const tl::fea::NodalTrialToken&,const tl::fea::NodalPreparedView&,
    const tl::fea::ShellPhysicalDiagnostics&,vehicle_dynamics::WallObservation&);
} // namespace crash::cases::vehicle_wall::loaded
