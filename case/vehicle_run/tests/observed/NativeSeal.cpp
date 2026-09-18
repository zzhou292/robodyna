#include "Contribution.h"
#include "case/vehicle_startup/shell_execution/tests/self_contact/CandidateCapture.h"
#include "case/vehicle_dynamics/Storage.h"
#include "case/vehicle_self_contact/RuntimeData.h"
#include "case/vehicle_self_contact/SelfContactStageError.h"
#include "case/vehicle_self_contact/runtime/Operations.h"
#include "case/vehicle_self_contact/runtime/Stages.h"
#include "output/ArtifactIO.h"

namespace crash::cases::vehicle_self_contact {

void CandidateRigidCouponAccess::InstallFailureObserver(
    vehicle_dynamics::VehiclePhysicalDynamics& dynamics,
    const tlfea::contact::self_contact_transaction::CandidateFailureObserver& observer) {
    auto& storage = *dynamics.storage_;
    output::Require(!storage.pending && storage.stamp.epoch == 0 &&
        dynamic_cast<detail::SelfContactStages*>(storage.self_contact.get()),
        "Controller failure observer requires fresh native self-contact stages");
    storage.self_contact = vehicle_run::observed::ObservedContribution::Wrap(
        storage.self_contact, observer, &SealWithFailureObserver);
}

void CandidateRigidCouponAccess::SealWithFailureObserver(
    vehicle_dynamics::detail::SelfContactContribution& contribution,
    tl::fea::FENodalState& owner, const tl::fea::NodalTrialToken& token,
    const tl::fea::ShellPhysicalDiagnostics& common,
    const tl::fea::NodalPreparedView& prepared,
    vehicle_dynamics::SelfContactObservation& observation,
    const tlfea::contact::self_contact_transaction::CandidateFailureObserver& observer) {
    auto* stages = dynamic_cast<detail::SelfContactStages*>(&contribution);
    output::Require(stages, "Observed seal requires the original native self-contact stages");
    // Same observation/authority protocol as runtime::SealCandidate. Only its
    // TL entry point gains an observer; normal PrepareStep owns error rollback.
    stages->receipt_ = {};
    runtime::CheckCandidateObservation(observation, prepared);
    tlfea::contact::SelfContactTransactionReceipt completed;
    const auto report =
        tlfea::contact::self_contact_transaction::QualificationAccess::
            SealCandidateWithFailureObserver(
                stages->contact.data_->transaction, owner, token, common,
                prepared, stages->accepted_, &completed, observer);
    if (report.status != tlfea::contact::SelfContactTransactionStatus::Ok)
        throw SelfContactStageError(report, SelfContactRuntimeStage::CandidateSeal,
                                   stages->budget_.transaction.accepted_event_capacity);
    runtime::ObserveCandidate(observation, prepared, completed);
    stages->receipt_ = completed;
    stages->accepted_ = {};
}

} // namespace crash::cases::vehicle_self_contact
