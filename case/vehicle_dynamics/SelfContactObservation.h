#pragma once

#include "lib_src/collision/SelfContactTransactionTypes.h"

namespace crash::cases::vehicle_dynamics {

// Diagnostic copy only. The accepted/final transaction receipts remain in the
// private self-contact stage and are never exposed through step observations.
struct SelfContactObservation {
    bool enabled = false;
    tlfea::contact::SelfContactForceDiagnostics accepted_force;
    std::size_t accepted_broadphase_pairs = 0;
    std::size_t accepted_facet_pairs = 0;
    std::size_t accepted_discovered_features = 0;
    std::uint64_t regularity_generation = 0;
    std::size_t candidate_broadphase_pairs = 0;
    std::size_t candidate_facet_pairs = 0;
    std::size_t policy_outcomes = 0;
    tlfea::contact::SelfContactCandidatePolicySummary policy_summary;
    std::size_t active_parents = 0;
    std::size_t removing_parents = 0;
    std::size_t skipped_parents = 0;
};

}  // namespace crash::cases::vehicle_dynamics
