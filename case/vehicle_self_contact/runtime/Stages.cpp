#include "Stages.h"

#include "../RuntimeData.h"
#include "Operations.h"

namespace crash::cases::vehicle_self_contact::detail {

void SelfContactStages::Assemble(
    tl::fea::FENodalState& owner,
    const tl::fea::NodalTrialToken& token,
    const tl::fea::NodalAssemblyView& assembly,
    vehicle_dynamics::SelfContactObservation& observation) {
    accepted_ = {};
    receipt_ = {};
    runtime::Assemble(
        contact.data_->transaction, owner, token, assembly,
        budget_.transaction.accepted_event_capacity,
        observation, accepted_);
}

void SelfContactStages::SealCandidate(
    tl::fea::FENodalState& owner,
    const tl::fea::NodalTrialToken& token,
    const tl::fea::ShellPhysicalDiagnostics& common,
    const tl::fea::NodalPreparedView& prepared,
    vehicle_dynamics::SelfContactObservation& observation) {
    receipt_ = {};
    runtime::SealCandidate(
        contact.data_->transaction, owner, token, common, prepared,
        budget_.transaction.accepted_event_capacity,
        observation, accepted_, receipt_);
}

tl::fea::ShellPhysicalScratchReceiptRoster
SelfContactStages::scratch_receipts() const noexcept {
    return receipt_.scratch_receipts();
}

tlfea::contact::SelfContactTransactionDiagnostics
SelfContactStages::diagnostics() const noexcept {return contact.data_->transaction.diagnostics();}

void SelfContactStages::Discard() noexcept {
    accepted_ = {};
    receipt_ = {};
    contact.data_->transaction.DiscardTrial();
}

}  // namespace crash::cases::vehicle_self_contact::detail
