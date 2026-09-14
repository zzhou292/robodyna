#pragma once

#include "case/vehicle_dynamics/SelfContactObservation.h"
#include "lib_src/collision/SelfContactTransaction.h"

namespace crash::cases::vehicle_self_contact::runtime {

void Assemble(
    tlfea::contact::SelfContactTransaction&,
    tl::fea::FENodalState&, const tl::fea::NodalTrialToken&,
    const tl::fea::NodalAssemblyView&,
    std::size_t event_capacity,
    vehicle_dynamics::SelfContactObservation&,
    tlfea::contact::SelfContactAcceptedAssemblyReceipt&);

void SealCandidate(
    tlfea::contact::SelfContactTransaction&,
    tl::fea::FENodalState&, const tl::fea::NodalTrialToken&,
    const tl::fea::ShellPhysicalDiagnostics&,
    const tl::fea::NodalPreparedView&,
    std::size_t event_capacity,
    vehicle_dynamics::SelfContactObservation&,
    tlfea::contact::SelfContactAcceptedAssemblyReceipt&,
    tlfea::contact::SelfContactTransactionReceipt&);

}  // namespace crash::cases::vehicle_self_contact::runtime
