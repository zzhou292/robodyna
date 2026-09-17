#include "../SelfContactSummary.h"
#include "../SelfContactDocument.h"
#include "case/vehicle_self_contact/SelfContactStageError.h"
#include "MechanicsFixture.h"

#include <gtest/gtest.h>

#include <functional>
#include <limits>
#include <type_traits>
#include <vector>

namespace crash::cases::vehicle_run::test {
namespace {

tl::fea::NodalStamp SelfStamp(std::uint64_t epoch) {
    auto stamp = MechanicsStamp(epoch);
    stamp.temporal_scheme = tl::fea::NodalTemporalScheme::StaggeredHalfKickStart;
    stamp.velocity_phase = epoch ? tl::fea::NodalVelocityPhase::PreviousMidpoint
                                 : tl::fea::NodalVelocityPhase::Collocated;
    return stamp;
}

vehicle_dynamics::StepObservation SelfStep(std::uint64_t epoch) {
    auto step = MechanicsStep(epoch);
    step.base = SelfStamp(epoch - 1);
    step.mechanics.base_stamp = step.base;
    auto& contact = step.self_contact;
    contact.enabled = true;
    auto& force = contact.accepted_force;
    force.valid = true;
    force.owner_id = step.base.owner_id;
    force.base_epoch = step.base.epoch;
    force.attempt = step.mechanics.solids.attempt;
    force.configuration_id = 31;
    force.qualification_id = 37;
    force.temporal_scheme = step.base.temporal_scheme;
    force.velocity_phase = step.base.velocity_phase;
    force.position_time = step.base.time;
    force.velocity_time = step.base.velocity_time;
    force.event_count = 7;
    force.vertex_face_event_count = 4;
    force.boundary_vertex_edge_event_count = 2; // Included in the four VF events.
    force.edge_edge_event_count = 3;
    force.active_count = 5;
    force.potential_j = 2;
    force.maximum_force_norm_n = 10;
    force.maximum_sti_diagonal_n_m = 100;
    force.maximum_represented_stiffness_n_m = 150;
    force.equal_opposite_residual_n = {.125, -.25, .5};
    force.global_moment_n_m = {.5, -1, 2};
    contact.accepted_broadphase_pairs = 8;
    contact.accepted_facet_pairs = 16;
    contact.accepted_discovered_features = 200;
    contact.regularity_generation = epoch;
    contact.candidate_broadphase_pairs = 9;
    contact.candidate_facet_pairs = 20;
    contact.policy_outcomes = 20;
    contact.active_parents = 10;
    contact.removing_parents = 1;
    contact.skipped_parents = 2;
    auto& policy = contact.policy_summary;
    policy.complete = true;
    policy.outcomes = 20;
    policy.certified_separated = 4;
    policy.excluded_same_rigid_group = 3;
    policy.excluded_local_intersection = 5;
    policy.represented_by_accepted_vf = 6;
    policy.represented_by_accepted_ee = 2;
    policy.digest = 5411954021770061371ull;
    return step;
}

std::string Json(const SelfContactTotals& totals) {
    const auto document = detail::SelfContactDocument(totals);
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    document.Accept(writer);
    return {buffer.GetString(), buffer.GetSize()};
}

}  // namespace

static_assert(std::is_trivially_copyable_v<SelfContactTotals>);

TEST(VehicleRunSelfContact, TwoCommittedIntervalsKeepBaseForceAndCandidatePolicySeparate) {
    SelfContactTotals totals;
    ObserveAcceptedSelfContact(totals, SelfStep(1), SelfStamp(1));
    EXPECT_TRUE(totals.available);
    EXPECT_EQ(totals.intervals, 1u);
    EXPECT_EQ(totals.last_base_time_s, 0);
    EXPECT_EQ(totals.last_time_s, .125);
    EXPECT_EQ(totals.last_base_velocity_time_s, 0);
    EXPECT_EQ(totals.last_event_count, 7u);
    EXPECT_EQ(totals.last_vertex_face_events + totals.last_edge_edge_events, 7u);
    EXPECT_EQ(totals.last_boundary_vertex_edge_events, 2u);
    EXPECT_EQ(totals.last_policy_outcomes, 20u);
    EXPECT_EQ(totals.last_policy_digest, 5411954021770061371ull);
    EXPECT_EQ(totals.last_accepted_base_potential_j, 2);
    EXPECT_EQ(totals.last_global_moment_n_m[1], -1);

    auto next = SelfStep(2);
    next.self_contact.accepted_force.potential_j = 1;
    next.self_contact.accepted_force.maximum_force_norm_n = 12;
    next.self_contact.accepted_force.maximum_sti_diagonal_n_m = 75;
    ObserveAcceptedSelfContact(totals, next, SelfStamp(2));
    EXPECT_EQ(totals.intervals, 2u);
    EXPECT_EQ(totals.last_attempt, 5u);
    EXPECT_EQ(totals.last_base_time_s, .125);
    EXPECT_EQ(totals.last_time_s, .25);
    EXPECT_EQ(totals.last_base_velocity_time_s, .0625);
    EXPECT_EQ(totals.last_accepted_base_potential_j, 1);
    EXPECT_EQ(totals.peak_accepted_base_potential_j, 2);
    EXPECT_EQ(totals.last_max_force_n, 12);
    EXPECT_EQ(totals.peak_max_force_n, 12);
    EXPECT_EQ(totals.last_max_sti_n_m, 75);
    EXPECT_EQ(totals.peak_max_sti_n_m, 100);
    const auto document = detail::SelfContactDocument(totals);
    EXPECT_EQ(document["last_policy_digest"].GetUint64(), 5411954021770061371ull);
    EXPECT_FALSE(document.HasMember("endpoint_potential_j"));
    EXPECT_FALSE(document.HasMember("total_energy_j"));
    EXPECT_FALSE(document.HasMember("contact_work_j"));
    RecordProperty("self_contact_totals_bytes", sizeof(SelfContactTotals));
}

TEST(VehicleRunSelfContact, StaleOwnerPhaseAttemptAndIncompletePolicyPreserveThenRetry) {
    SelfContactTotals totals;
    ObserveAcceptedSelfContact(totals, SelfStep(1), SelfStamp(1));
    const auto before = Json(totals);
    using Mutate = std::function<void(vehicle_dynamics::StepObservation&)>;
    const std::vector<Mutate> mutations{
        [](auto& s) { s.self_contact.enabled = false; },
        [](auto& s) { s.self_contact.accepted_force.valid = false; },
        [](auto& s) { ++s.self_contact.accepted_force.owner_id; },
        [](auto& s) { ++s.self_contact.accepted_force.base_epoch; },
        [](auto& s) { ++s.self_contact.accepted_force.configuration_id; },
        [](auto& s) { ++s.self_contact.accepted_force.qualification_id; },
        [](auto& s) { --s.self_contact.accepted_force.attempt; },
        [](auto& s) { s.self_contact.accepted_force.position_time = s.proposed_time; },
        [](auto& s) { s.self_contact.accepted_force.velocity_time = s.proposed_time; },
        [](auto& s) { s.self_contact.accepted_force.velocity_phase = tl::fea::NodalVelocityPhase::Collocated; },
        [](auto& s) { s.self_contact.accepted_force.temporal_scheme = tl::fea::NodalTemporalScheme::VelocityFirst; },
        [](auto& s) { s.mechanics.valid = false; },
        [](auto& s) { ++s.mechanics.solids.attempt; },
        [](auto& s) { s.mechanics.solids.phase = static_cast<decltype(s.mechanics.solids.phase)>(255); },
        [](auto& s) { s.self_contact.regularity_generation = 0; },
        [](auto& s) { s.self_contact.policy_summary.complete = false; },
        [](auto& s) { ++s.self_contact.candidate_facet_pairs; },
        [](auto& s) { ++s.self_contact.policy_outcomes; },
        [](auto& s) { ++s.self_contact.policy_summary.excluded_local_intersection; },
        [](auto& s) { s.self_contact.policy_summary.certified_separated = SIZE_MAX; },
        [](auto& s) { ++s.self_contact.accepted_force.edge_edge_event_count; },
        [](auto& s) { s.self_contact.accepted_force.boundary_vertex_edge_event_count = 5; },
        [](auto& s) { s.self_contact.accepted_force.active_count = 8; },
        [](auto& s) {
            auto& force = s.self_contact.accepted_force;
            force.vertex_face_event_count = force.boundary_vertex_edge_event_count = 0;
            force.edge_edge_event_count = force.event_count;
        },
        [](auto& s) {
            auto& force = s.self_contact.accepted_force;
            force.edge_edge_event_count = 0;
            force.vertex_face_event_count = force.event_count;
        },
    };
    for (std::size_t i = 0; i < mutations.size(); ++i) {
        SCOPED_TRACE(i);
        auto next = SelfStep(2);
        mutations[i](next);
        EXPECT_THROW(ObserveAcceptedSelfContact(totals, next, SelfStamp(2)), std::invalid_argument);
        EXPECT_EQ(Json(totals), before);
    }
    EXPECT_THROW(ObserveAcceptedSelfContact(totals, SelfStep(1), SelfStamp(1)), std::invalid_argument);
    EXPECT_THROW(ObserveAcceptedSelfContact(totals, SelfStep(3), SelfStamp(3)), std::invalid_argument);
    EXPECT_EQ(Json(totals), before);
    ObserveAcceptedSelfContact(totals, SelfStep(2), SelfStamp(2));
    EXPECT_EQ(totals.intervals, 2u);
}

TEST(VehicleRunSelfContact, NonfiniteAndNegativeLateDiagnosticsPreserveAllTotals) {
    SelfContactTotals totals;
    ObserveAcceptedSelfContact(totals, SelfStep(1), SelfStamp(1));
    const auto before = Json(totals);
    for (unsigned field = 0; field < 8; ++field) {
        SCOPED_TRACE(field);
        auto next = SelfStep(2);
        auto& force = next.self_contact.accepted_force;
        const auto bad = std::numeric_limits<double>::infinity();
        if (field == 0) force.potential_j = -1;
        if (field == 1) force.maximum_force_norm_n = bad;
        if (field == 2) force.maximum_sti_diagonal_n_m = -1;
        if (field == 3) force.maximum_represented_stiffness_n_m = bad;
        if (field == 4) force.endpoint_a_resultant_n.x = bad;
        if (field == 5) force.endpoint_b_resultant_n.y = bad;
        if (field == 6) force.equal_opposite_residual_n.z = bad;
        if (field == 7) force.global_moment_n_m.x = std::numeric_limits<double>::quiet_NaN();
        EXPECT_THROW(ObserveAcceptedSelfContact(totals, next, SelfStamp(2)), std::invalid_argument);
        EXPECT_EQ(Json(totals), before);
    }
}

TEST(VehicleRunSelfContact, ZeroEventsRemainAvailableAndInitialSummaryDoesNotInventData) {
    const auto initial = detail::SelfContactDocument({});
    EXPECT_FALSE(initial["available"].GetBool());
    EXPECT_FALSE(initial.HasMember("last_event_count"));
    EXPECT_FALSE(initial.HasMember("last_accepted_base_potential_j"));
    auto step = SelfStep(1);
    auto& force = step.self_contact.accepted_force;
    force.event_count = force.vertex_face_event_count = force.edge_edge_event_count = 0;
    force.boundary_vertex_edge_event_count = force.active_count = 0;
    force.potential_j = force.maximum_force_norm_n = force.maximum_sti_diagonal_n_m = 0;
    force.maximum_represented_stiffness_n_m = 0;
    force.equal_opposite_residual_n = force.global_moment_n_m = {};
    auto& policy = step.self_contact.policy_summary;
    policy.certified_separated = 20;
    policy.excluded_same_rigid_group = policy.excluded_local_intersection = 0;
    policy.represented_by_accepted_vf = policy.represented_by_accepted_ee = 0;
    SelfContactTotals totals;
    ObserveAcceptedSelfContact(totals, step, SelfStamp(1));
    EXPECT_TRUE(totals.available);
    EXPECT_EQ(totals.last_event_count, 0u);
    EXPECT_EQ(totals.last_policy_outcomes, 20u);
    EXPECT_EQ(totals.last_accepted_base_potential_j, 0);
}

TEST(VehicleRunSelfContact, ErrorDocumentPreservesExactLowerBoundAndOrdinalMeanings) {
    using namespace vehicle_self_contact;
    using Count = tlfea::contact::SelfContactTransactionCountKind;
    tlfea::contact::SelfContactTransactionReport report;
    report.status = tlfea::contact::SelfContactTransactionStatus::ResourceLimit;
    report.candidate = 65;
    report.count_kind = Count::ExactAcceptedEvents;
    const auto exact = detail::SelfContactErrorDocument(
        SelfContactStageError(report, SelfContactRuntimeStage::AcceptedAssembly, 64));
    EXPECT_STREQ(exact["count_kind"].GetString(), "exact_accepted_events");
    EXPECT_EQ(exact["required_force_events"].GetUint64(), 65u);
    EXPECT_FALSE(exact.HasMember("candidate_ordinal"));
    report.count_kind = Count::AcceptedEventsLowerBound;
    const auto lower = detail::SelfContactErrorDocument(
        SelfContactStageError(report, SelfContactRuntimeStage::AcceptedAssembly, 64));
    EXPECT_EQ(lower["required_events_lower_bound"].GetUint64(), 65u);
    EXPECT_FALSE(lower.HasMember("required_force_events"));
    EXPECT_FALSE(lower.HasMember("reported_exact_accepted_events"));
    report.count_kind = Count::None;
    report.nonlinear_subdivision_work_exhausted = true;
    const auto ordinal = detail::SelfContactErrorDocument(
        SelfContactStageError(report, SelfContactRuntimeStage::CandidateSeal, 64));
    EXPECT_EQ(ordinal["candidate_ordinal"].GetUint64(), 65u);
    EXPECT_TRUE(ordinal["nonlinear_subdivision_work_exhausted"].GetBool());
    EXPECT_FALSE(ordinal.HasMember("required_force_events"));
    EXPECT_FALSE(ordinal.HasMember("required_events_lower_bound"));
}

}  // namespace crash::cases::vehicle_run::test
