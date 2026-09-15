#include "Source.h"

#include "case/CanonicalWallArtifacts.h"
#include "case/vehicle_self_contact/SelfContactFactories.h"
#include "case/vehicle_self_contact/SelfContactStageError.h"
#include "case/vehicle_startup/physical_model/tests/Support.h"
#include "case/vehicle_wall/LoadedWall.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

#include <cstdlib>
#include <chrono>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace crash::cases::vehicle_startup::shell_execution::
    self_contact_test {
namespace {

namespace app = crash::cases::vehicle_self_contact;
namespace wall = crash::cases::vehicle_wall;
namespace c = tlfea::contact;
namespace fe = tl::fea;

constexpr std::size_t Nodes = 376930;
constexpr std::size_t Parents = 337092;
constexpr std::size_t MaximumFamilyParents = 315963;
constexpr std::size_t Facets = 653055;
constexpr std::size_t ParentPairs = 1584464;
constexpr std::size_t FacetPairs = 5989248;
constexpr std::size_t Chunk = 4096;
constexpr std::size_t WorkPerPair = 4095;
constexpr std::size_t WorkPerChunk = std::size_t{1} << 20;
constexpr std::size_t CompleteCrossingWork =
    FacetPairs * WorkPerPair;
constexpr std::size_t RuntimeHostCap =
    std::size_t{20} * 1000 * 1000 * 1000;
constexpr std::size_t RuntimeDeviceCap = std::size_t{8} << 30;
constexpr std::size_t InitialTransactionArenaBytes = 1542092504;
constexpr std::size_t AcceptedEventCensusCapacity = 1000000;
constexpr unsigned DiscoveryWorkers = 4;
constexpr unsigned CrossingWorkers = 4;
constexpr std::uint64_t SelfSourceId = 0x563553454c464354ull;

double Seconds(
    std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
}

static_assert(CompleteCrossingWork / WorkPerPair == FacetPairs);
static_assert(SelfSourceId != wall::Settings{}.wall_binding_id);

std::size_t EventHashSlots(std::size_t events) {
    if (!events ||
        events > std::numeric_limits<std::size_t>::max() / 2)
        throw std::runtime_error(
            "Exact accepted-event hash capacity is unrepresentable");
    return 2 * events;
}

app::RuntimeLimits RuntimeLimits(std::size_t event_ledger_capacity) {
    const c::SelfContactTransactionLimits::ExactCensus census{
        Nodes, Parents, Parents, MaximumFamilyParents, Facets,
        ParentPairs, FacetPairs, 0};
    app::RuntimeLimits result;
    result.host_bytes = RuntimeHostCap;
    result.device_bytes = RuntimeDeviceCap;
    result.transaction =
        c::SelfContactTransactionLimits::Vehicle(
            census, Chunk, event_ledger_capacity,
            EventHashSlots(event_ledger_capacity), 0,
            WorkPerPair, WorkPerChunk, CompleteCrossingWork, 20,
            RuntimeHostCap, RuntimeDeviceCap, RuntimeHostCap,
            DiscoveryWorkers, CrossingWorkers);
    // Vehicle() builds the generic count/work shape.  Keep the top-level
    // 20 GB/8 GiB transaction admission while respecting each existing
    // production component's narrower declared profile.
    result.transaction.activity.max_selected_parents = 1000000;
    result.transaction.activity.max_family_parents = 1000000;
    result.transaction.activity.max_host_bytes = std::size_t{512} << 20;
    result.transaction.activity.max_startup_host_bytes =
        std::size_t{512} << 20;
    result.transaction.broadphase.max_host_bytes =
        std::size_t{2} << 30;
    result.transaction.broadphase.max_device_bytes =
        std::size_t{2} << 30;
    result.transaction.regularity.max_host_bytes =
        c::SelfContactCurrentRegularityLimits::Vehicle()
            .max_host_bytes;
    return result;
}

app::RuntimeConfig RuntimeConfig(std::size_t events) {
    return {SelfSourceId, events, 0};
}

vehicle_dynamics::Config DynamicsConfig() {
    auto result = wall::LoadedWallConfig();
    result.timing.enabled = true;
    return result;
}

void CheckExactForecast(
    const app::RuntimeForecast& forecast,
    std::size_t event_ledger_capacity,
    std::size_t force_event_capacity) {
    const auto& transaction = forecast.transaction;
    EXPECT_EQ(forecast.identity.source_id, SelfSourceId);
    EXPECT_NE(forecast.identity.owner_id, 0u);
    EXPECT_NE(forecast.identity.configuration_id, 0u);
    EXPECT_NE(forecast.identity.qualification_id, 0u);
    EXPECT_EQ(
        forecast.identity.physical_source_instance_id,
        LevelZeroSetup().physical().domain()->source_instance_id());
    EXPECT_EQ(
        forecast.identity.active_use_identity,
        LevelZeroSetup().active_uses().identity());
    EXPECT_EQ(forecast.identity.roster_entries, 1u);
    EXPECT_EQ(transaction.surface_parent_map_capacity, Parents);
    EXPECT_EQ(transaction.parent_facet_offset_count, Parents + 1);
    EXPECT_EQ(transaction.facet_descriptor_capacity, Facets);
    EXPECT_EQ(transaction.broadphase_pair_capacity, ParentPairs);
    EXPECT_EQ(transaction.complete_facet_pair_capacity, FacetPairs);
    EXPECT_EQ(transaction.facet_pair_chunk_capacity, Chunk);
    EXPECT_EQ(
        transaction.accepted_event_ledger_capacity,
        event_ledger_capacity);
    EXPECT_EQ(
        transaction.event_hash_capacity,
        EventHashSlots(event_ledger_capacity));
    EXPECT_EQ(
        transaction.accepted_event_capacity,
        force_event_capacity);
    EXPECT_EQ(transaction.policy_outcome_capacity, 0u);
    EXPECT_EQ(
        transaction.accepted_discovery.worker_count,
        DiscoveryWorkers);
    EXPECT_EQ(
        transaction.candidate_discovery.worker_count,
        DiscoveryWorkers);
    EXPECT_EQ(transaction.crossing.worker_count, CrossingWorkers);
    EXPECT_EQ(
        transaction.complete_crossing_work_capacity,
        CompleteCrossingWork);
    EXPECT_LE(forecast.peak_host_upper_bound, RuntimeHostCap);
    EXPECT_LE(forecast.device_bytes, RuntimeDeviceCap);
    EXPECT_GT(transaction.participation.publication_host_bytes, 0u);
    if (event_ledger_capacity == 1 && force_event_capacity == 1)
        EXPECT_EQ(
            transaction.candidate_arena_bytes,
            InitialTransactionArenaBytes);
}

void CheckInstalledStartup(
    const app::RuntimeForecast& preview,
    const vehicle_dynamics::VehiclePhysicalDynamics& dynamics) {
    ASSERT_NE(dynamics.self_contact_setup(), nullptr);
    EXPECT_TRUE(
        dynamics.self_contact_setup()->SharesStorage(
            LevelZeroSetup()));
    const auto* actual = dynamics.self_contact_forecast();
    ASSERT_NE(actual, nullptr);
    EXPECT_EQ(actual->identity.source_id, preview.identity.source_id);
    EXPECT_EQ(actual->identity.owner_id, dynamics.accepted().owner_id);
    EXPECT_EQ(
        actual->identity.active_use_identity,
        preview.identity.active_use_identity);
    EXPECT_EQ(
        actual->transaction.candidate_arena_bytes,
        preview.transaction.candidate_arena_bytes);
    EXPECT_EQ(
        actual->transaction.device_bytes,
        preview.transaction.device_bytes);
    EXPECT_EQ(
        dynamics.allocations().device_bytes,
        preview.device_bytes);
    const auto allocation = dynamics.self_contact_allocations();
    EXPECT_EQ(
        allocation.activity.host_bytes,
        preview.transaction.activity.arena_bytes);
    EXPECT_EQ(
        allocation.device.device_bytes,
        preview.transaction.device_bytes);
    EXPECT_EQ(dynamics.accepted().epoch, 0u);
    EXPECT_EQ(dynamics.accepted().time, 0);
}

void PrintFailure(const app::SelfContactStageError& error) {
    const auto& report = error.report();
    std::cout
        << "V5_SELF_CONTACT_RUNTIME status=gate_failure"
        << " stage="
        << static_cast<unsigned>(error.stage())
        << " transaction_status="
        << static_cast<unsigned>(report.status)
        << " candidate=" << report.candidate
        << " pair=" << report.pair
        << " discovery_status="
        << static_cast<unsigned>(report.discovery_status)
        << " discovery_task=" << report.discovery_task
        << " discovery_reason="
        << static_cast<unsigned>(report.discovery_reason)
        << " crossing_status="
        << static_cast<unsigned>(report.crossing_status)
        << " crossing_reason="
        << static_cast<unsigned>(report.crossing_reason)
        << " facet0_parent="
        << report.offending_motion[0].facet.parent_eid
        << " facet0_local="
        << report.offending_motion[0].facet.local_facet
        << " facet0_motion="
        << static_cast<unsigned>(
               report.offending_motion[0].motion)
        << " facet1_parent="
        << report.offending_motion[1].facet.parent_eid
        << " facet1_local="
        << report.offending_motion[1].facet.local_facet
        << " facet1_motion="
        << static_cast<unsigned>(
               report.offending_motion[1].motion)
        << " feature_distance_m="
        << report.offending_feature_distance_m
        << " edge_parameter0="
        << report.offending_edge_parameters[0]
        << " edge_parameter1="
        << report.offending_edge_parameters[1]
        << " reason=" << report.message << '\n';
}

void CheckCandidate(
    const vehicle_dynamics::StepObservation& candidate) {
    const auto& self = candidate.self_contact;
    ASSERT_TRUE(self.enabled);
    EXPECT_TRUE(self.accepted_force.valid);
    EXPECT_EQ(self.accepted_broadphase_pairs, ParentPairs);
    EXPECT_EQ(self.accepted_facet_pairs, FacetPairs);
    EXPECT_TRUE(self.policy_summary.complete);
    EXPECT_FALSE(self.policy_summary.detailed_publication);
    EXPECT_EQ(
        self.policy_summary.outcomes,
        self.candidate_facet_pairs);
    EXPECT_EQ(
        self.policy_summary.certified_separated +
            self.policy_summary.excluded_same_rigid_group +
            self.policy_summary.excluded_local_intersection +
            self.policy_summary.represented_by_accepted_vf,
        self.policy_summary.outcomes);
    EXPECT_EQ(self.policy_outcomes, self.policy_summary.outcomes);
    EXPECT_EQ(
        self.active_parents + self.removing_parents +
            self.skipped_parents,
        Parents);
    const auto crossing_outcomes =
        self.policy_summary.outcomes -
        self.policy_summary.excluded_same_rigid_group;
    ::testing::Test::RecordProperty(
        "accepted_events",
        std::to_string(self.accepted_force.event_count));
    ::testing::Test::RecordProperty(
        "accepted_active",
        std::to_string(self.accepted_force.active_count));
    ::testing::Test::RecordProperty(
        "accepted_parent_pairs",
        std::to_string(self.accepted_broadphase_pairs));
    ::testing::Test::RecordProperty(
        "accepted_facet_pairs",
        std::to_string(self.accepted_facet_pairs));
    ::testing::Test::RecordProperty(
        "same_body_exclusions",
        std::to_string(
            self.policy_summary.excluded_same_rigid_group));
    ::testing::Test::RecordProperty(
        "crossing_outcomes",
        std::to_string(crossing_outcomes));
    ::testing::Test::RecordProperty(
        "policy_digest",
        std::to_string(self.policy_summary.digest));
    std::cout
        << "V5_SELF_CONTACT_RUNTIME status=complete"
        << " accepted_events="
        << self.accepted_force.event_count
        << " accepted_active="
        << self.accepted_force.active_count
        << " accepted_features="
        << self.accepted_discovered_features
        << " parent_pairs="
        << self.accepted_broadphase_pairs
        << " facet_pairs=" << self.accepted_facet_pairs
        << " candidate_parent_pairs="
        << self.candidate_broadphase_pairs
        << " candidate_facet_pairs="
        << self.candidate_facet_pairs
        << " same_body_exclusions="
        << self.policy_summary.excluded_same_rigid_group
        << " certified_separated="
        << self.policy_summary.certified_separated
        << " local_intersections="
        << self.policy_summary.excluded_local_intersection
        << " nonlocal_intersections=0"
        << " represented_vf="
        << self.policy_summary.represented_by_accepted_vf
        << " ee_only=0"
        << " unresolved=0"
        << " unsupported_motion=0"
        << " crossing_outcomes=" << crossing_outcomes
        << " crossing_work_cap=" << CompleteCrossingWork
        << " policy_outcomes="
        << self.policy_summary.outcomes
        << " policy_digest=" << self.policy_summary.digest
        << " rigid_rejected_pair=none"
        << '\n';
}

void CheckStableAttempt(
    vehicle_dynamics::VehiclePhysicalDynamics& dynamics) {
    const auto initial = dynamics.accepted();
    const auto allocations = dynamics.allocations();
    const auto self_allocations =
        dynamics.self_contact_allocations();
    const auto first_start = std::chrono::steady_clock::now();
    const auto first = dynamics.PrepareStep();
    std::cout << "V5_SELF_CONTACT_PHASE first_attempt_s="
              << Seconds(first_start) << '\n';
    ASSERT_NO_FATAL_FAILURE(CheckCandidate(first));
    EXPECT_TRUE(fe::trial_identity::SameStamp(
        dynamics.accepted(), initial));
    dynamics.DiscardStep();
    EXPECT_TRUE(fe::trial_identity::SameStamp(
        dynamics.accepted(), initial));
    EXPECT_EQ(
        dynamics.allocations().device_bytes,
        allocations.device_bytes);
    EXPECT_EQ(
        dynamics.self_contact_allocations().device.device_bytes,
        self_allocations.device.device_bytes);
    const auto retry_start = std::chrono::steady_clock::now();
    const auto retry = dynamics.PrepareStep();
    std::cout << "V5_SELF_CONTACT_PHASE retry_attempt_s="
              << Seconds(retry_start) << '\n';
    ASSERT_NO_FATAL_FAILURE(CheckCandidate(retry));
    EXPECT_EQ(
        retry.self_contact.accepted_force.event_count,
        first.self_contact.accepted_force.event_count);
    EXPECT_EQ(
        retry.self_contact.policy_summary.digest,
        first.self_contact.policy_summary.digest);
    EXPECT_EQ(
        retry.self_contact.policy_summary.outcomes,
        first.self_contact.policy_summary.outcomes);
    dynamics.DiscardStep();
    EXPECT_TRUE(fe::trial_identity::SameStamp(
        dynamics.accepted(), initial));
}

const wall::VehicleWallSetup& ActualWallSetup() {
    static const auto value = [] {
        const char* path = std::getenv("ROBO_VEHICLE_WALL");
        if (!path || !*path)
            throw std::runtime_error(
                "Missing authenticated original wall manifest");
        const auto bytes =
            case_data::ReadPinnedWallManifest(path);
        case_data::CanonicalWall original;
        std::istringstream input(bytes);
        if (original.Load(input).status !=
            case_data::WallStatus::Ok)
            throw std::runtime_error(
                "Authenticated original wall cannot be parsed");
        auto settings = wall::LoadedWallSettings();
        settings.leading_gap_m = 1e-6;
        settings.requested_duration_s = .005;
        return wall::VehicleWallSetup::Prepare(
            Execution(), PhysicalAttachments(),
            original, bytes, settings);
    }();
    return value;
}

app::WallSelfContactLimits CombinedLimits(
    std::size_t events) {
    app::WallSelfContactLimits result;
    result.host_bytes = RuntimeHostCap;
    result.device_bytes = RuntimeDeviceCap;
    result.self_contact = RuntimeLimits(events);
    return result;
}

void CheckWallForecastUnchanged(
    const app::WallSelfContactForecast& combined,
    const wall::RuntimeForecast& standalone) {
    EXPECT_EQ(
        combined.wall.retained_host_upper_bound,
        standalone.retained_host_upper_bound);
    EXPECT_EQ(
        combined.wall.peak_host_upper_bound,
        standalone.peak_host_upper_bound);
    EXPECT_EQ(
        combined.wall.device_bytes,
        standalone.device_bytes);
    EXPECT_EQ(
        combined.wall.contact.device_bytes,
        standalone.contact.device_bytes);
    EXPECT_EQ(
        combined.participation.publication_host_bytes,
        combined.wall.participation.publication_host_bytes);
    EXPECT_GT(
        combined.participation.configured_issuer_host_bytes,
        combined.wall.participation.configured_issuer_host_bytes);
}

TEST(VehicleSelfContactRuntime,
     FullV5ForecastStartupOwnsExactIdentityAndMemory) {
    const auto setup_start = std::chrono::steady_clock::now();
    const auto& setup = LevelZeroSetup();
    std::cout << "V5_SELF_CONTACT_PHASE setup_s="
              << Seconds(setup_start) << '\n';
    ASSERT_EQ(setup.physical().domain()->node_count(), Nodes);
    ASSERT_EQ(setup.active_uses().parents().size(), Parents);
    ASSERT_EQ(setup.counts().q4_parents, MaximumFamilyParents);
    ASSERT_EQ(setup.active_uses().facet_uses().size(), Facets);
    const auto limits = RuntimeLimits(1);
    const auto config = RuntimeConfig(1);
    const auto dynamics_config = DynamicsConfig();
    const auto& joints = physical_model::supports_test::Joints();
    const auto preflight_start = std::chrono::steady_clock::now();
    const auto forecast = app::SelfContactOnly::Preflight(
        setup, dynamics_config, config, limits, &joints);
    std::cout << "V5_SELF_CONTACT_PHASE preflight_s="
              << Seconds(preflight_start) << '\n';
    ASSERT_NO_FATAL_FAILURE(CheckExactForecast(forecast, 1, 1));
    const auto prepare_start = std::chrono::steady_clock::now();
    auto dynamics = app::SelfContactOnly::Prepare(
        setup, dynamics_config, config, limits, &joints);
    std::cout << "V5_SELF_CONTACT_PHASE prepare_s="
              << Seconds(prepare_start) << '\n';
    ASSERT_NO_FATAL_FAILURE(
        CheckInstalledStartup(forecast, dynamics));
    RecordProperty(
        "retained_host_upper_bound",
        std::to_string(forecast.retained_host_upper_bound));
    RecordProperty(
        "peak_host_upper_bound",
        std::to_string(forecast.peak_host_upper_bound));
    RecordProperty(
        "transaction_arena_bytes",
        std::to_string(
            forecast.transaction.candidate_arena_bytes));
    RecordProperty(
        "device_bytes", std::to_string(forecast.device_bytes));
}

TEST(VehicleSelfContactRuntime,
     FullV5OneAttemptIsTypedFailClosedAndRetryStable) {
    const auto setup_start = std::chrono::steady_clock::now();
    const auto& setup = LevelZeroSetup();
    std::cout << "V5_SELF_CONTACT_PHASE setup_s="
              << Seconds(setup_start) << '\n';
    const auto dynamics_config = DynamicsConfig();
    const auto& joints = physical_model::supports_test::Joints();
    std::size_t required_events = 0;
    {
        const auto limits =
            RuntimeLimits(AcceptedEventCensusCapacity);
        const auto prepare_start =
            std::chrono::steady_clock::now();
        auto dynamics = app::SelfContactOnly::Prepare(
            setup, dynamics_config, RuntimeConfig(1),
            limits, &joints);
        std::cout << "V5_SELF_CONTACT_PHASE prepare_s="
                  << Seconds(prepare_start) << '\n';
        const auto initial = dynamics.accepted();
        const auto allocations = dynamics.allocations();
        try {
            ASSERT_NO_FATAL_FAILURE(CheckStableAttempt(dynamics));
        } catch (const app::SelfContactStageError& error) {
            PrintFailure(error);
            required_events = error.required_events();
            if (!required_events)
                throw;
            std::cout
                << "V5_SELF_CONTACT_RUNTIME"
                << " status=event_capacity"
                << " required_events=" << required_events
                << " parent_pairs=" << ParentPairs
                << " facet_pairs=" << FacetPairs << '\n';
            EXPECT_TRUE(fe::trial_identity::SameStamp(
                dynamics.accepted(), initial));
            EXPECT_FALSE(dynamics.has_prepared_step());
            EXPECT_EQ(
                dynamics.allocations().device_bytes,
                allocations.device_bytes);
        }
    }
    if (required_events) {
        const auto limits = RuntimeLimits(required_events);
        const auto config = RuntimeConfig(required_events);
        const auto forecast = app::SelfContactOnly::Preflight(
            setup, dynamics_config, config, limits, &joints);
        ASSERT_NO_FATAL_FAILURE(
            CheckExactForecast(
                forecast, required_events, required_events));
        auto dynamics = app::SelfContactOnly::Prepare(
            setup, dynamics_config, config, limits, &joints);
        ASSERT_NO_FATAL_FAILURE(
            CheckInstalledStartup(forecast, dynamics));
        try {
            ASSERT_NO_FATAL_FAILURE(CheckStableAttempt(dynamics));
        } catch (const app::SelfContactStageError& error) {
            PrintFailure(error);
            throw;
        }
    }
}

TEST(VehicleWallSelfContactRuntime,
     FullV5CombinedForecastStartupOwnsBothFixedSlotsOnce) {
    const auto& wall_setup = ActualWallSetup();
    const auto& self_setup = LevelZeroSetup();
    const auto config = RuntimeConfig(1);
    const auto limits = CombinedLimits(1);
    const auto dynamics_config = DynamicsConfig();
    const auto& joints = physical_model::supports_test::Joints();
    const auto standalone = wall::LoadedWall::Preflight(
        wall_setup, dynamics_config, limits.wall, &joints);
    const auto combined = app::LoadedWallSelfContact::Preflight(
        wall_setup, self_setup, config, limits,
        dynamics_config, &joints);
    ASSERT_NO_FATAL_FAILURE(
        CheckWallForecastUnchanged(combined, standalone));
    auto dynamics = app::LoadedWallSelfContact::Prepare(
        wall_setup, self_setup, config, limits,
        dynamics_config, &joints);
    ASSERT_NE(dynamics.wall_setup(), nullptr);
    ASSERT_NE(dynamics.self_contact_setup(), nullptr);
    EXPECT_TRUE(
        dynamics.wall_setup()->SharesStorage(wall_setup));
    EXPECT_TRUE(
        dynamics.self_contact_setup()->SharesStorage(self_setup));
    EXPECT_EQ(dynamics.accepted().epoch, 0u);
    EXPECT_EQ(
        dynamics.allocations().device_bytes,
        combined.device_bytes);
    RecordProperty(
        "combined_peak_host_upper_bound",
        std::to_string(combined.peak_host_upper_bound));
    RecordProperty(
        "combined_device_bytes",
        std::to_string(combined.device_bytes));
    RecordProperty(
        "combined_publication_host_bytes",
        std::to_string(
            combined.participation.publication_host_bytes));
}

TEST(VehicleWallSelfContactRuntime,
     FullV5CombinedAttemptSealsBothReceiptsAndRetries) {
    const auto& wall_setup = ActualWallSetup();
    const auto& self_setup = LevelZeroSetup();
    const auto dynamics_config = DynamicsConfig();
    const auto& joints = physical_model::supports_test::Joints();
    const auto limits = CombinedLimits(1);
    const auto combined = app::LoadedWallSelfContact::Preflight(
        wall_setup, self_setup, RuntimeConfig(1), limits,
        dynamics_config, &joints);
    const auto standalone = wall::LoadedWall::Preflight(
        wall_setup, dynamics_config, limits.wall, &joints);
    ASSERT_NO_FATAL_FAILURE(
        CheckWallForecastUnchanged(combined, standalone));
    {
        auto dynamics = app::LoadedWallSelfContact::Prepare(
            wall_setup, self_setup, RuntimeConfig(1), limits,
            dynamics_config, &joints);
        const auto initial = dynamics.accepted();
        const auto allocations = dynamics.allocations();
        try {
            const auto first = dynamics.PrepareStep();
            ASSERT_TRUE(first.wall.enabled);
            ASSERT_TRUE(first.wall.accepted.valid);
            ASSERT_TRUE(first.wall.prepared.valid);
            ASSERT_NO_FATAL_FAILURE(CheckCandidate(first));
            EXPECT_TRUE(fe::trial_identity::SameStamp(
                dynamics.accepted(), initial));
            dynamics.DiscardStep();
            EXPECT_TRUE(fe::trial_identity::SameStamp(
                dynamics.accepted(), initial));
            const auto retry = dynamics.PrepareStep();
            ASSERT_TRUE(retry.wall.enabled);
            ASSERT_TRUE(retry.wall.accepted.valid);
            ASSERT_TRUE(retry.wall.prepared.valid);
            ASSERT_NO_FATAL_FAILURE(CheckCandidate(retry));
            EXPECT_EQ(
                retry.self_contact.policy_summary.digest,
                first.self_contact.policy_summary.digest);
            EXPECT_EQ(
                retry.wall.prepared.contact.resultant.value,
                first.wall.prepared.contact.resultant.value);
            dynamics.CommitStep();
            EXPECT_EQ(dynamics.accepted().epoch, 1u);
            EXPECT_EQ(
                dynamics.allocations().device_bytes,
                allocations.device_bytes);
        } catch (const app::SelfContactStageError& error) {
            PrintFailure(error);
            throw;
        }
    }
    // The combined owner has retired.  Reusing the immutable wall setup in
    // the original wall-only factory proves that its source/runtime contract
    // was not consumed or rewritten by the two-slot composition.
    auto wall_only = wall::LoadedWall::Prepare(
        wall_setup, dynamics_config, limits.wall, &joints);
    const auto wall_initial = wall_only.accepted();
    const auto wall_candidate = wall_only.PrepareStep();
    EXPECT_TRUE(wall_candidate.wall.enabled);
    EXPECT_TRUE(wall_candidate.wall.accepted.valid);
    EXPECT_TRUE(wall_candidate.wall.prepared.valid);
    wall_only.DiscardStep();
    EXPECT_TRUE(fe::trial_identity::SameStamp(
        wall_only.accepted(), wall_initial));
}

}  // namespace
}  // namespace crash::cases::vehicle_startup::shell_execution::
   // self_contact_test
