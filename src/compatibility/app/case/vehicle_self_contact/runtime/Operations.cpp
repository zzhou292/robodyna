#include "Operations.h"

#include "../SelfContactStageError.h"
#include "output/ArtifactIO.h"

namespace crash::cases::vehicle_self_contact::runtime {
namespace {

void Check(
    const tlfea::contact::SelfContactTransactionReport& report,
    SelfContactRuntimeStage stage,
    std::size_t event_capacity) {
    if (report.status !=
        tlfea::contact::SelfContactTransactionStatus::Ok)
        throw SelfContactStageError(
            report, stage, event_capacity);
}

}  // namespace

void Assemble(
    tlfea::contact::SelfContactTransaction& transaction,
    tl::fea::FENodalState& owner,
    const tl::fea::NodalTrialToken& token,
    const tl::fea::NodalAssemblyView& assembly,
    std::size_t event_capacity,
    vehicle_dynamics::SelfContactObservation& observation,
    tlfea::contact::SelfContactAcceptedAssemblyReceipt& authority) {
    authority = {};
    vehicle_dynamics::SelfContactObservation next;
    tlfea::contact::SelfContactAcceptedAssemblyReceipt accepted;
    Check(transaction.AssembleAccepted(
              owner, token, assembly, &accepted),
          SelfContactRuntimeStage::AcceptedAssembly,
          event_capacity);
    next.enabled = true;
    next.accepted_force = accepted.diagnostics();
    next.accepted_broadphase_pairs =
        accepted.broadphase_pairs();
    next.accepted_facet_pairs = accepted.facet_pairs();
    next.accepted_discovered_features =
        accepted.discovered_features();
    observation = next;
    authority = accepted;
}

void CheckCandidateObservation(
    const vehicle_dynamics::SelfContactObservation& observation,
    const tl::fea::NodalPreparedView& prepared) {
    output::Require(
        observation.enabled && observation.accepted_force.valid &&
            observation.accepted_force.owner_id == prepared.owner_id &&
            observation.accepted_force.base_epoch ==
                prepared.kinematics.base_epoch &&
            observation.accepted_force.attempt == prepared.attempt,
        "Prepared self-contact stage requires the same accepted attempt");
}

void ObserveCandidate(
    vehicle_dynamics::SelfContactObservation& observation,
    const tl::fea::NodalPreparedView& prepared,
    const tlfea::contact::SelfContactTransactionReceipt& completed,
    const tlfea::contact::SelfContactTransactionDiagnostics& diagnostics) {
    CheckCandidateObservation(observation, prepared);
    auto next = observation;
    next.regularity_generation =
        completed.regularity_generation();
    next.candidate_broadphase_pairs =
        completed.broadphase_pairs();
    next.candidate_facet_pairs = completed.facet_pairs();
    next.policy_outcomes = completed.policy_outcomes();
    next.policy_summary = completed.policy_summary();
    next.active_parents = completed.active_parents();
    next.removing_parents = completed.removing_parents();
    next.skipped_parents = completed.skipped_parents();
    // Native seal has returned and closed its timing scopes. This copy is
    // descriptive; the controller matches its phase only when publishing totals.
    next.diagnostics = diagnostics;
    observation = next;
}

void SealCandidate(
    tlfea::contact::SelfContactTransaction& transaction,
    tl::fea::FENodalState& owner,
    const tl::fea::NodalTrialToken& token,
    const tl::fea::ShellPhysicalDiagnostics& common,
    const tl::fea::NodalPreparedView& prepared,
    std::size_t event_capacity,
    vehicle_dynamics::SelfContactObservation& observation,
    tlfea::contact::SelfContactAcceptedAssemblyReceipt& accepted,
    tlfea::contact::SelfContactTransactionReceipt& authority) {
    authority = {};
    CheckCandidateObservation(observation, prepared);
    tlfea::contact::SelfContactTransactionReceipt completed;
    Check(transaction.SealCandidate(
              owner, token, common, prepared, accepted, &completed),
          SelfContactRuntimeStage::CandidateSeal,
          event_capacity);
    ObserveCandidate(observation, prepared, completed, transaction.diagnostics());
    authority = completed;
    accepted = {};
}

}  // namespace crash::cases::vehicle_self_contact::runtime
