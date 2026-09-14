#include "Operations.h"

#include "output/ArtifactIO.h"

namespace crash::cases::vehicle_self_contact::runtime {
namespace {

void Check(
    const tlfea::contact::SelfContactTransactionReport& report) {
    output::Require(
        report.status ==
            tlfea::contact::SelfContactTransactionStatus::Ok,
        report.message);
}

}  // namespace

void Assemble(
    tlfea::contact::SelfContactTransaction& transaction,
    tl::fea::FENodalState& owner,
    const tl::fea::NodalTrialToken& token,
    const tl::fea::NodalAssemblyView& assembly,
    vehicle_dynamics::SelfContactObservation& observation,
    tlfea::contact::SelfContactAcceptedAssemblyReceipt& authority) {
    authority = {};
    vehicle_dynamics::SelfContactObservation next;
    tlfea::contact::SelfContactAcceptedAssemblyReceipt accepted;
    Check(transaction.AssembleAccepted(
        owner, token, assembly, &accepted));
    next.enabled = true;
    next.accepted_force = accepted.diagnostics();
    next.accepted_broadphase_pairs =
        accepted.broadphase_pairs();
    next.accepted_facet_pairs = accepted.facet_pairs();
    observation = next;
    authority = accepted;
}

void SealCandidate(
    tlfea::contact::SelfContactTransaction& transaction,
    tl::fea::FENodalState& owner,
    const tl::fea::NodalTrialToken& token,
    const tl::fea::ShellPhysicalDiagnostics& common,
    const tl::fea::NodalPreparedView& prepared,
    vehicle_dynamics::SelfContactObservation& observation,
    tlfea::contact::SelfContactAcceptedAssemblyReceipt& accepted,
    tlfea::contact::SelfContactTransactionReceipt& authority) {
    authority = {};
    output::Require(
        observation.enabled && observation.accepted_force.valid &&
            observation.accepted_force.owner_id == prepared.owner_id &&
            observation.accepted_force.base_epoch ==
                prepared.kinematics.base_epoch &&
            observation.accepted_force.attempt == prepared.attempt,
        "Prepared self-contact stage requires the same accepted attempt");
    auto next = observation;
    tlfea::contact::SelfContactTransactionReceipt completed;
    Check(transaction.SealCandidate(
        owner, token, common, prepared, accepted, &completed));
    next.regularity_generation =
        completed.regularity_generation();
    next.candidate_broadphase_pairs =
        completed.broadphase_pairs();
    next.candidate_facet_pairs = completed.facet_pairs();
    next.policy_outcomes = completed.policy_outcomes();
    next.active_parents = completed.active_parents();
    next.removing_parents = completed.removing_parents();
    next.skipped_parents = completed.skipped_parents();
    observation = next;
    authority = completed;
    accepted = {};
}

}  // namespace crash::cases::vehicle_self_contact::runtime
