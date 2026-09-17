#include "../Run.h"
#include "OriginalFixture.h"
#include "../ContactComposition.h"
#include "case/vehicle_startup/shell_execution/tests/self_contact/CensusFixtureExport.h"
#include "case/vehicle_startup/shell_execution/tests/self_contact/NonlinearCoverageFixture.h"

#include <iostream>

namespace crash::cases::vehicle_run::test {
namespace {
namespace capture = vehicle_startup::shell_execution::self_contact_test;
namespace fixture = capture::nonlinear_fixture;

struct DiscardCandidate {
    vehicle_dynamics::VehiclePhysicalDynamics& dynamics;
    ~DiscardCandidate() { dynamics.DiscardStep(); }
};

output::Document Profile(const OriginalCase& source,
                         const ContactComposition& composition,
                         const vehicle_dynamics::Config& dynamics) {
    output::Document result;
    result.SetObject();
    output::String(result, "physical_profile", "vehicle-supports-v5");
    output::String(result, "contact_profile", "wall-self-contact-v1");
    output::String(result, "scope", "same Source and ContactComposition as TwoCommittedV5IntervalsPreserveBothContactProfilesAndReplay");
    output::Number(result, "leading_gap_m", source.setup.settings().leading_gap_m);
    output::Number(result, "initial_speed_mps", source.setup.settings().initial_speed_mps);
    output::Number(result, "declared_duration_s", source.setup.settings().requested_duration_s);
    output::Number(result, "physical_step_s", dynamics.startup.reserved_step_s);
    output::Integer(result, "source_instance_id", source.setup.execution().physical().domain()->source_instance_id());
    output::Integer(result, "configuration_id", dynamics.startup.configuration_id);
    output::Integer(result, "qualification_id", dynamics.startup.qualification_id);
    output::Integer(result, "self_contact_source_id", composition.runtime_config().source_id);
    output::Integer(result, "event_capacity", composition.runtime_config().event_capacity);
    const auto& limits = composition.runtime_limits().self_contact.transaction;
    output::Integer(result, "parent_pair_capacity", limits.broadphase.max_pairs);
    output::Integer(result, "facet_pair_capacity", limits.max_candidate_pairs);
    output::Integer(result, "facet_chunk", limits.max_facet_pair_chunk);
    output::Integer(result, "crossing_work_per_pair", limits.crossing.max_work_per_pair);
    output::Integer(result, "crossing_depth", limits.crossing.max_depth);
    output::Integer(result, "discovery_workers", limits.accepted_discovery.worker_count);
    output::Integer(result, "crossing_workers", limits.crossing.worker_count);
    return result;
}

std::uint64_t HashProfile(const output::Document& profile) {
    std::uint64_t hash = 1469598103934665603ull;
    for (auto member = profile.MemberBegin(); member != profile.MemberEnd(); ++member) {
        fixture::HashBytes(member->name.GetString(), member->name.GetStringLength(), &hash);
        const auto& value = member->value;
        if (value.IsString()) fixture::HashBytes(value.GetString(), value.GetStringLength(), &hash);
        else if (value.IsUint64()) fixture::HashUnsigned(value.GetUint64(), &hash);
        else fixture::HashUnsigned(output::Bits(value.GetDouble()), &hash);
    }
    return hash;
}
}  // namespace

TEST(VehicleRunWallSelfContactCensus,
     CommittedFirstIntervalThenCompleteActualSecondCandidateCensus) {
    const auto* output_path = std::getenv("ROBO_SECOND_CENSUS_OUTPUT");
    ASSERT_TRUE(output_path && *output_path)
        << "Set an absent ROBO_SECOND_CENSUS_OUTPUT directory to retain every frozen case";
    ASSERT_FALSE(std::filesystem::exists(output_path));
    std::cout << std::unitbuf;
    // Exact same 1 um gap, source setup, event capacity and factory as the
    // failed production-controller two-interval gate. No synthetic restart.
    const auto source = Source(PhysicalProfile::VehicleSupportsV5, nullptr,
                               ContactProfile::WallSelfContactV1);
    ASSERT_TRUE(source.self_contact);
    ASSERT_EQ(source.setup.settings().leading_gap_m, 1e-6);
    auto dynamics_config = vehicle_wall::LoadedWallConfig();
    dynamics_config.startup.reserved_step_s = 2e-7;
    dynamics_config.timing.enabled = true;
    const auto composition = ContactComposition::Prepare(
        ContactProfile::WallSelfContactV1, source.self_contact);
    const auto forecast = composition.Preflight(source.setup, dynamics_config, &source.joints);
    ASSERT_LE(forecast.peak_host_upper_bound, 20ull * 1000 * 1000 * 1000);
    ASSERT_EQ(composition.runtime_config().event_capacity, 65536u);
    auto dynamics = composition.CreateDynamics(source.setup, dynamics_config, &source.joints);
    DiscardCandidate cleanup{dynamics};
    const auto& first = dynamics.PrepareStep();
    ASSERT_TRUE(first.self_contact.enabled);
    ASSERT_EQ(first.self_contact.accepted_force.event_count, 32491u);
    ASSERT_EQ(first.self_contact.policy_summary.digest, 5411954021770061371ull);
    dynamics.CommitStep();
    const auto accepted = dynamics.accepted();
    ASSERT_EQ(accepted.epoch, 1u);
    ASSERT_EQ(output::Bits(accepted.time), output::Bits(2e-7));
    std::cout << "V5_SECOND_CENSUS first_interval_committed=1 accepted_epoch=1" << std::endl;

    const auto snapshot = vehicle_self_contact::CandidateRigidCouponAccess::
        PrepareCandidateCensus(dynamics);
    ASSERT_TRUE(snapshot.prepared_census_receipt.valid());
    ASSERT_EQ(snapshot.accepted_stamp.epoch, 1u);
    ASSERT_EQ(snapshot.prepared_view.kinematics.base_epoch, 1u);
    ASSERT_EQ(output::Bits(snapshot.prepared_view.proposed_time), output::Bits(4e-7));
    ASSERT_EQ(output::Bits(snapshot.prepared_view.kick_dt), output::Bits(2e-7));
    const auto activity = snapshot.prepared_census_receipt.activity_summary();
    ASSERT_TRUE(activity.complete);
    EXPECT_EQ(activity.selected, source.self_contact->active_uses().parents().size());
    EXPECT_EQ(activity.accepted_active, activity.prepared_active + activity.removing);
    EXPECT_EQ(activity.selected, activity.prepared_active + activity.removing + activity.inactive);
    const auto profile = Profile(source, composition, dynamics_config);
    std::uint64_t dt_hash = 1469598103934665603ull;
    fixture::HashUnsigned(output::Bits(snapshot.prepared_view.proposed_time -
                                     snapshot.prepared_view.base_time), &dt_hash);
    fixture::HashUnsigned(output::Bits(snapshot.prepared_view.kick_dt), &dt_hash);
    fixture::HashUnsigned(static_cast<unsigned>(snapshot.prepared_view.rigid_member_trajectory), &dt_hash);
    const auto result = capture::ExportPreparedCensus(snapshot, output_path,
                                                    profile, HashProfile(profile), dt_hash);
    EXPECT_EQ(result.linear_pairs, snapshot.linear_summary.represented_work_exhausted);
    EXPECT_GT(result.files, 0u);
    dynamics.DiscardStep();
    EXPECT_FALSE(snapshot.prepared_census_receipt.valid());
    EXPECT_FALSE(dynamics.has_prepared_step());
    EXPECT_EQ(dynamics.accepted().epoch, accepted.epoch);
    EXPECT_EQ(output::Bits(dynamics.accepted().time), output::Bits(accepted.time));
    RecordProperty("scope", "first interval committed; actual prepared second census frozen and discarded; no second commit claim");
    RecordProperty("fixture_directory", output_path);
    RecordProperty("linear_fixture_pairs", std::to_string(result.linear_pairs));
    RecordProperty("nonlinear_fixture_pairs", std::to_string(result.nonlinear_pairs));
}

}  // namespace crash::cases::vehicle_run::test
