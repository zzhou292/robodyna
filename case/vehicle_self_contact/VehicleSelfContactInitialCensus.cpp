#include "VehicleSelfContactInitialCensus.h"

#include "case/vehicle_dynamics/Storage.h"
#include "case/vehicle_runtime/ParticipantConfigs.h"
#include "case/vehicle_runtime/Reports.h"
#include "lib_src/collision/FixedTriangleFeatureDiscovery.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include "lib_utils/BoundedArena.h"
#include "output/ArtifactIO.h"
#include <algorithm>
#include <chrono>
#include <limits>
#include <string>

namespace crash::cases::vehicle_self_contact {
namespace {

namespace contact = tlfea::contact;
namespace fe = tl::fea;
using Broadphase = contact::SelfContactBroadphase;
using BroadphaseForecast = contact::SelfContactBroadphaseForecast;
using BroadphaseStatus = contact::SelfContactBroadphaseStatus;
using Key = contact::SelfContactPairKey;

constexpr std::size_t V5SelectedParents = 337092;
constexpr std::size_t V5Q4Parents = 315963;
constexpr std::size_t V5T3Parents = 21129;
constexpr std::size_t V5Level0Facets = 653055;

struct WorkspaceRegions {
    tl::util::ArenaRegion surface_to_active;
    tl::util::ArenaRegion active_parents;
    tl::util::ArenaRegion pair_keys;
    tl::util::ArenaRegion accepted_positions;
    tl::util::ArenaRegion represented_triangles;
    tl::util::ArenaRegion exact_sample_pairs;
    tl::util::ArenaRegion feature_task_masks;
    std::size_t bytes = 0;
};

class TrialGuard {
  public:
    explicit TrialGuard(fe::FENodalState& owner) : owner_(&owner) {}
    ~TrialGuard() {
        if (owner_) owner_->Discard();
    }
    void Release() noexcept { owner_ = nullptr; }

  private:
    fe::FENodalState* owner_ = nullptr;
};

[[noreturn]] void CapacityFailure(std::uint64_t required,
                                  const std::string& message) {
    throw InitialCensusCapacityError(required,
        message + " (required_pairs=" + std::to_string(required) + ")");
}

bool Add(std::size_t value, std::size_t& total) noexcept {
    if (value > std::numeric_limits<std::size_t>::max() - total)
        return false;
    total += value;
    return true;
}

contact::SelfContactBroadphaseLimits BroadphaseLimits(
    const VehicleSelfContactSetup& setup, std::size_t pair_capacity,
    InitialCensusLimits limits) {
    return {setup.surface().parents().size(),
            setup.physical().domain()->node_count(),
            pair_capacity, limits.max_broadphase_device_bytes,
            limits.max_host_bytes};
}

BroadphaseForecast PreflightBroadphase(
    const VehicleSelfContactSetup& setup, std::size_t pair_capacity,
    InitialCensusLimits limits, std::uint64_t required,
    const char* phase) {
    const auto preflight = Broadphase::Preflight(
        setup.surface(),
        BroadphaseLimits(setup, pair_capacity, limits));
    if (preflight.report.status != BroadphaseStatus::Ok)
        CapacityFailure(required, std::string(phase) + ": " +
            preflight.report.message);
    return preflight.forecast;
}

bool Same(const BroadphaseForecast& first,
          const BroadphaseForecast& second) noexcept {
    return first.parents == second.parents &&
        first.nodes == second.nodes &&
        first.pair_capacity == second.pair_capacity &&
        first.sort_temp_bytes == second.sort_temp_bytes &&
        first.scan_temp_bytes == second.scan_temp_bytes &&
        first.pair_sort_temp_bytes == second.pair_sort_temp_bytes &&
        first.device_bytes == second.device_bytes &&
        first.retained_source_bytes == second.retained_source_bytes &&
        first.owned_host_bytes == second.owned_host_bytes &&
        first.startup_scratch_bytes == second.startup_scratch_bytes &&
        first.startup_host_bytes == second.startup_host_bytes;
}

bool Same(const InitialFacetCapacityCensus& first,
          const InitialFacetCapacityCensus& second) noexcept {
    return first.current_inflated_aabb_overlap_parent_pairs ==
            second.current_inflated_aabb_overlap_parent_pairs &&
        first.q4_q4_parent_pairs == second.q4_q4_parent_pairs &&
        first.q4_t3_parent_pairs == second.q4_t3_parent_pairs &&
        first.t3_t3_parent_pairs == second.t3_t3_parent_pairs &&
        first.level0_facet_pairs == second.level0_facet_pairs &&
        first.same_complete_rigid_group_parent_pairs ==
            second.same_complete_rigid_group_parent_pairs &&
        first.shared_canonical_vertex_parent_pairs ==
            second.shared_canonical_vertex_parent_pairs &&
        first.shared_canonical_edge_parent_pairs ==
            second.shared_canonical_edge_parent_pairs &&
        first.pair_key_hash == second.pair_key_hash &&
        first.surface_active_source_hash ==
            second.surface_active_source_hash &&
        first.sorted_unique_canonical_keys ==
            second.sorted_unique_canonical_keys &&
        first.no_self_or_reversed_pair_keys ==
            second.no_self_or_reversed_pair_keys &&
        first.descriptive_static_policy_only ==
            second.descriptive_static_policy_only &&
        first.feature_discovery_performed ==
            second.feature_discovery_performed &&
        first.intersection_processing_performed ==
            second.intersection_processing_performed &&
        first.force_admission_performed ==
            second.force_admission_performed;
}

bool Same(const InitialFacetFilterCensus& first,
          const InitialFacetFilterCensus& second) noexcept {
    return first.represented_facet_pairs ==
            second.represented_facet_pairs &&
        first.excluded_same_rigid_group ==
            second.excluded_same_rigid_group &&
        first.coordinate_aabb_separated ==
            second.coordinate_aabb_separated &&
        first.face_axis_separated == second.face_axis_separated &&
        first.edge_cross_axis_separated ==
            second.edge_cross_axis_separated &&
        first.vertex_edge_axis_separated ==
            second.vertex_edge_axis_separated &&
        first.vertex_vertex_axis_separated ==
            second.vertex_vertex_axis_separated &&
        first.exact_remaining == second.exact_remaining &&
        first.exact_sample_count == second.exact_sample_count &&
        first.exact_sample_hash == second.exact_sample_hash &&
        first.category_hash == second.category_hash &&
        first.source_identity_hash == second.source_identity_hash &&
        first.complete_disjoint_accounting ==
            second.complete_disjoint_accounting &&
        first.production_certificates_used ==
            second.production_certificates_used &&
        first.feature_discovery_performed ==
            second.feature_discovery_performed &&
        first.interval_crossing_performed ==
            second.interval_crossing_performed;
}

bool SameParentSource(const fe::ShellPlasticityParentInput& first,
                      const fe::ShellPlasticityParentInput& second) noexcept {
    return first.family == second.family &&
        first.family_index == second.family_index &&
        first.source_parent_id == second.source_parent_id &&
        first.source_part_id == second.source_part_id &&
        first.material_id == second.material_id &&
        first.section_id == second.section_id;
}

std::uint32_t RigidGroupForMember(
    const fe::NodalRigidAssemblyBinding& rigid,
    const fe::RigidBindingMember* member) {
    if (!member) return UINT32_MAX;
    const auto members = rigid.members();
    output::Require(member >= members.data() &&
            member < members.data() + members.size(),
        "Rigid member lookup escaped the actual retained binding");
    const auto member_index =
        static_cast<std::size_t>(member - members.data());
    const auto groups = rigid.groups();
    std::size_t lower = 0, upper = groups.size();
    while (lower < upper) {
        const auto middle = lower + (upper - lower) / 2;
        const auto& group = groups[middle];
        output::Require(group.member_offset <= members.size() &&
                group.member_count <=
                    members.size() - group.member_offset,
            "Actual rigid group member range is invalid");
        if (member_index < group.member_offset)
            upper = middle;
        else if (member_index >=
                 group.member_offset + group.member_count)
            lower = middle + 1;
        else {
            output::Require(middle < UINT32_MAX,
                "Actual rigid group ordinal is unrepresentable");
            return static_cast<std::uint32_t>(middle);
        }
    }
    output::Require(false,
        "Actual rigid member has no containing retained group");
    return UINT32_MAX;
}

std::uint32_t CompleteRigidGroup(
    const fe::NodalRigidAssemblyBinding& rigid,
    const contact::SelfContactParentUse& parent) {
    std::uint32_t common = UINT32_MAX;
    for (unsigned local = 0; local < parent.arity; ++local) {
        const auto group = RigidGroupForMember(
            rigid, rigid.FindMember(parent.nodes[local]));
        if (group == UINT32_MAX) return UINT32_MAX;
        if (common == UINT32_MAX)
            common = group;
        else if (group != common)
            return UINT32_MAX;
    }
    return common;
}

WorkspaceRegions ForecastWorkspace(std::size_t nodes,
                                   std::size_t parents,
                                   std::size_t facets,
                                   std::size_t pairs,
                                   std::size_t host_cap) {
    if (nodes > std::numeric_limits<std::size_t>::max() / 3)
        CapacityFailure(pairs,
            "Initial V5 accepted-position extent overflows");
    WorkspaceRegions result;
    tl::util::BoundedArenaLayout layout(host_cap);
    if (!layout.Append<std::uint32_t>(
            parents, result.surface_to_active) ||
        !layout.Append<InitialCensusParentRow>(
            parents, result.active_parents) ||
        !layout.Append<Key>(pairs, result.pair_keys) ||
        !layout.Append<double>(3 * nodes, result.accepted_positions) ||
        !layout.Append<contact::CurrentFixedTriangle>(
            facets, result.represented_triangles) ||
        !layout.Append<contact::FixedTrianglePair>(
            InitialExactFeatureSampleCapacity,
            result.exact_sample_pairs) ||
        !layout.Append<contact::FixedTriangleFeatureTaskMask>(
            InitialExactFeatureChunkCapacity,
            result.feature_task_masks))
        CapacityFailure(pairs,
            "Initial V5 census fixed host workspace exceeds its hard cap");
    result.bytes = layout.bytes();
    return result;
}

void ComposePeakForecast(InitialCensusResult& result,
                         InitialCensusLimits limits) {
    auto& forecast = result.forecast;
    forecast.app_fixed_host_bytes = sizeof(InitialCensusResult);
    std::size_t retained_with_workspace =
        forecast.exact.owned_host_bytes;
    if (!Add(forecast.fixed_workspace_host_bytes,
             retained_with_workspace) ||
        !Add(forecast.exact_feature_discovery.owned_host_bytes,
             retained_with_workspace))
        CapacityFailure(result.probe_required_pairs,
            "Initial V5 census retained host forecast overflows");
    const auto broadphase_phase = std::max(
        {forecast.count_probe.startup_host_bytes,
         forecast.cap_minus_one.startup_host_bytes,
         forecast.exact.startup_host_bytes,
         retained_with_workspace});
    forecast.peak_census_host_reservation_bytes =
        forecast.app_fixed_host_bytes;
    if (!Add(broadphase_phase,
             forecast.peak_census_host_reservation_bytes) ||
        forecast.peak_census_host_reservation_bytes >
            limits.max_host_bytes)
        CapacityFailure(result.probe_required_pairs,
            "Initial V5 census complete host forecast exceeds its hard cap");

    forecast.peak_census_device_bytes = std::max(
        {forecast.count_probe.device_bytes,
         forecast.cap_minus_one.device_bytes,
         forecast.exact.device_bytes});
    forecast.peak_total_explicit_device_bytes =
        forecast.physical_owner_device_bytes;
    if (!Add(forecast.peak_census_device_bytes,
             forecast.peak_total_explicit_device_bytes))
        CapacityFailure(result.probe_required_pairs,
            "Initial V5 owner plus census device forecast overflows");
}

void CheckLimits(InitialCensusLimits limits) {
    const InitialCensusLimits hard;
    output::Require(limits.max_pairs > 1 &&
            limits.max_pairs <= hard.max_pairs &&
            limits.max_host_bytes &&
            limits.max_host_bytes <= hard.max_host_bytes &&
            limits.max_broadphase_device_bytes &&
            limits.max_broadphase_device_bytes <=
                hard.max_broadphase_device_bytes &&
            limits.broadphase_axis <= 2,
        "Initial V5 census limits exceed INT_MAX/2GiB hard scope");
}

[[noreturn]] void FeatureSampleFailure(
    const InitialFeatureSampleReport& report) {
    throw std::runtime_error(
        "Initial exact feature sample failed (pair=" +
        std::to_string(report.pair) +
        ", task=" + std::to_string(report.task) +
        ", arithmetic_reason=" +
        std::to_string(static_cast<unsigned>(
            report.arithmetic_reason)) +
        "): " + report.message);
}

}  // namespace

InitialCensusResult VehicleSelfContactInitialCensus::Measure(
    const VehicleSelfContactSetup& setup,
    vehicle_dynamics::VehiclePhysicalDynamics& dynamics,
    InitialCensusLimits limits) {
    CheckLimits(limits);
    output::Require(bool(dynamics.storage_) &&
            !dynamics.storage_->pending &&
            !dynamics.wall_setup() &&
            setup.config().facet_level == 0,
        "Initial self-contact census requires one virgin free-flight owner");
    auto& state = dynamics.storage_->state();
    const auto initial_stamp = state.owner.accepted();
    const auto& active = setup.active_uses();
    const auto* rigid = active.rigid();
    const auto cin = active.cin();
    output::Require(initial_stamp.epoch == 0 &&
            initial_stamp.time == 0 &&
            setup.MatchesSource(state.execution, state.attachments,
                                setup.original()) &&
            &setup.physical() == &state.execution.physical() &&
            setup.surface().MatchesPhysical(state.execution.physical()) &&
            setup.facets().surface() &&
            setup.facets().surface()->SharesStorage(setup.surface()) &&
            active.facets() &&
            active.facets()->SharesStorage(setup.facets()) &&
            rigid && cin.model,
        "Initial census setup does not share the actual V5 source graph");
    output::Require(setup.counts().selected_shell_parents ==
                V5SelectedParents &&
            setup.counts().q4_parents == V5Q4Parents &&
            setup.counts().t3_parents == V5T3Parents &&
            setup.census().topology.facets == V5Level0Facets &&
            setup.surface().parents().size() == V5SelectedParents &&
            active.parents().size() == V5SelectedParents &&
            active.facet_uses().size() == V5Level0Facets,
        "Initial census source is not the authenticated full V5 level0 set");

    const auto participants = fe::ShellPhysicalParticipants{
        &state.qeph, &state.t3, &state.qbat, &state.type25,
        &state.type13, &state.solids, state.type45.get(),
        state.beam18.get()};
    const auto participant_configs =
        vehicle_runtime::detail::ConfigureParticipants(
            state.config, state.execution, state.attachments,
            initial_stamp);
    auto publication = state.publication.ValidatePhysicalSources(
        state.owner, state.execution.physical(), participants,
        participant_configs.publication);
    output::Require(publication.status ==
            fe::ShellPublicationStatus::Success,
        publication.message);
    auto owner_report =
        state.owner.ValidateRigidAssemblyBinding(*rigid);
    output::Require(owner_report.status == fe::NodalStatus::Ok,
        owner_report.message);
    owner_report = state.owner.ValidateCinWitnessSource(
        vehicle_runtime::detail::Witnesses(state.attachments));
    output::Require(owner_report.status == fe::NodalStatus::Ok,
        owner_report.message);

    cudaStream_t owner_stream = nullptr;
    owner_report = state.owner.BorrowOwnerStream(&owner_stream);
    output::Require(owner_report.status == fe::NodalStatus::Ok,
        owner_report.message);
    owner_report = state.owner.ValidateOwnerStream(owner_stream);
    output::Require(owner_report.status == fe::NodalStatus::Ok,
        owner_report.message);

    fe::NodalTrialToken token;
    fe::NodalAssemblyView assembly;
    owner_report = state.owner.BeginTrial(&token, &assembly);
    output::Require(owner_report.status == fe::NodalStatus::Ok,
        owner_report.message);
    TrialGuard trial(state.owner);
    owner_report =
        state.owner.AuthenticateAssemblyView(token, assembly);
    output::Require(owner_report.status == fe::NodalStatus::Ok,
        owner_report.message);
    output::Require(assembly.stream == owner_stream &&
            assembly.owner_id == initial_stamp.owner_id &&
            assembly.accepted.position_xyz &&
            assembly.accepted.node_count == initial_stamp.node_count &&
            assembly.accepted.node_count ==
                setup.physical().domain()->node_count() &&
            assembly.accepted.base_epoch == initial_stamp.epoch,
        "Initial broadphase input is not the exact accepted owner view");
    const contact::SelfContactBroadphaseInput input{
        {assembly.accepted.position_xyz,
         static_cast<std::uint32_t>(assembly.accepted.node_count), 3, 1},
        {}, contact::SelfContactBoundsMotion::Current,
        limits.broadphase_axis};

    InitialCensusResult result;
    result.source.physical_domain_source_instance_id =
        setup.physical().domain()->source_instance_id();
    result.source.owner_id = initial_stamp.owner_id;
    result.source.configuration_id =
        participant_configs.publication.configuration_id;
    result.source.qualification_id =
        participant_configs.publication.qualification_id;
    result.source.nodes = initial_stamp.node_count;
    result.source.surface_parents = setup.surface().parents().size();
    result.source.active_parents = active.parents().size();
    result.source.physical_participants =
        6 + bool(state.type45) + bool(state.beam18);
    result.source.rigid_groups = rigid->groups().size();
    result.source.rigid_members = rigid->members().size();
    result.source.cin_rows = cin.range_count;
    result.source.cin_witnesses = cin.witness_count;
    result.source.exact_setup_physical_identity = true;
    result.source.exact_owner_stream_identity = true;
    result.source.exact_rigid_identity = true;
    result.source.exact_cin_identity = true;
    result.source.exact_participant_source_identity = true;
    result.forecast.retained_setup_host_reservation_bytes =
        setup.forecast().retained_setup_reservation_bytes;
    result.forecast.physical_dynamics_peak_host_upper_bound =
        dynamics.forecast().peak_host_upper_bound;
    result.forecast.physical_owner_device_bytes =
        dynamics.allocations().device_bytes;
    output::Require(result.forecast.physical_owner_device_bytes ==
            (dynamics.forecast().startup.device_bytes+dynamics.forecast().motion.device_bytes),
        "Actual physical owner/participant allocation differs from forecast");

    result.forecast.count_probe =
        PreflightBroadphase(setup, 1, limits, 0,
                            "Count probe cannot fit current hard caps");
    {
        Broadphase probe;
        auto report = probe.Initialize(
            setup.surface(), BroadphaseLimits(setup, 1, limits),
            owner_stream);
        output::Require(report.status == BroadphaseStatus::Ok,
            report.message);
        output::Require(Same(probe.forecast(),
                             result.forecast.count_probe),
            "Count-probe broadphase differs from exact preflight");
        report = probe.Evaluate(input, owner_stream);
        output::Require(report.status ==
                BroadphaseStatus::PairCapacity &&
                report.required_pairs > 1 &&
                !probe.pairs().complete &&
                !probe.pairs().device_keys &&
                probe.pairs().count == 0,
            "Initial broadphase did not publish exact count-before-cap failure");
        result.probe_required_pairs = report.required_pairs;
        result.count_before_cap_observed = true;
    }
    const auto required = result.probe_required_pairs;
    if (required > INT_MAX || required > limits.max_pairs)
        CapacityFailure(required,
            "Initial V5 pair count exceeds the permitted INT_MAX app cap");
    const auto exact_capacity = static_cast<std::size_t>(required);
    const auto short_capacity = exact_capacity - 1;
    result.forecast.cap_minus_one = PreflightBroadphase(
        setup, short_capacity, limits, required,
        "Cap-minus-one broadphase cannot fit current hard caps");
    result.forecast.exact = PreflightBroadphase(
        setup, exact_capacity, limits, required,
        "Exact broadphase cannot fit the TL 2GiB device hard cap");
    const auto feature_limits = InitialFeatureSampleDiscoveryLimits(
        InitialExactFeatureChunkCapacity, 4);
    const auto feature_preflight =
        contact::FixedTriangleFeatureDiscovery::Preflight(
            feature_limits);
    output::Require(feature_preflight.report.status ==
            contact::FixedTriangleDiscoveryStatus::Ok,
        feature_preflight.report.message);
    result.forecast.exact_feature_discovery =
        feature_preflight.forecast;
    const auto workspace = ForecastWorkspace(
        initial_stamp.node_count, V5SelectedParents,
        V5Level0Facets, exact_capacity, limits.max_host_bytes);
    result.forecast.surface_to_active_host_bytes =
        workspace.surface_to_active.bytes;
    result.forecast.active_parent_host_bytes =
        workspace.active_parents.bytes;
    result.forecast.pair_key_host_bytes = workspace.pair_keys.bytes;
    result.forecast.accepted_position_host_bytes =
        workspace.accepted_positions.bytes;
    result.forecast.represented_triangle_host_bytes =
        workspace.represented_triangles.bytes;
    result.forecast.exact_sample_pair_host_bytes =
        workspace.exact_sample_pairs.bytes;
    result.forecast.feature_task_mask_host_bytes =
        workspace.feature_task_masks.bytes;
    result.forecast.fixed_workspace_host_bytes = workspace.bytes;
    ComposePeakForecast(result, limits);

    {
        Broadphase short_broadphase;
        auto report = short_broadphase.Initialize(
            setup.surface(),
            BroadphaseLimits(setup, short_capacity, limits),
            owner_stream);
        output::Require(report.status == BroadphaseStatus::Ok,
            report.message);
        output::Require(Same(short_broadphase.forecast(),
                             result.forecast.cap_minus_one),
            "Cap-minus-one broadphase differs from exact preflight");
        report = short_broadphase.Evaluate(input, owner_stream);
        output::Require(report.status ==
                BroadphaseStatus::PairCapacity &&
                report.required_pairs == required &&
                !short_broadphase.pairs().complete,
            "Cap-minus-one did not reproduce the measured required count");
        result.cap_minus_one_required_pairs = report.required_pairs;
        result.cap_minus_one_reproduced_required_count = true;
    }

    {
        Broadphase exact;
        auto report = exact.Initialize(
            setup.surface(),
            BroadphaseLimits(setup, exact_capacity, limits),
            owner_stream);
        output::Require(report.status == BroadphaseStatus::Ok,
            report.message);
        output::Require(Same(exact.forecast(),
                             result.forecast.exact),
            "Exact broadphase differs from its pre-allocation forecast");
        report = exact.Evaluate(input, owner_stream);
        output::Require(report.status == BroadphaseStatus::Ok,
            report.message);
        auto pairs = exact.pairs();
        output::Require(pairs.complete && pairs.count == required &&
                pairs.device_keys,
            "Exact broadphase did not publish the complete measured pair set");
        const auto* stable_device_keys = pairs.device_keys;

        tl::util::HostArena host;
        output::Require(host.Initialize(workspace.bytes),
            "Initial V5 census fixed host allocation failed");
        auto* surface_to_active =
            host.Construct<std::uint32_t>(
                workspace.surface_to_active);
        auto* active_parents =
            host.Construct<InitialCensusParentRow>(
                workspace.active_parents);
        auto* host_keys = host.Construct<Key>(workspace.pair_keys);
        auto* accepted_positions =
            host.Construct<double>(workspace.accepted_positions);
        auto* represented_triangles =
            host.Construct<contact::CurrentFixedTriangle>(
                workspace.represented_triangles);
        auto* exact_sample_pairs =
            host.Construct<contact::FixedTrianglePair>(
                workspace.exact_sample_pairs);
        auto* feature_task_masks =
            host.Construct<contact::FixedTriangleFeatureTaskMask>(
                workspace.feature_task_masks);
        output::Require(surface_to_active && active_parents &&
                host_keys && accepted_positions &&
                represented_triangles && exact_sample_pairs &&
                feature_task_masks &&
                host.bytes() == workspace.bytes,
            "Initial V5 census fixed host layout differs from forecast");
        std::fill_n(surface_to_active, V5SelectedParents,
                    UINT32_MAX);
        const auto surface_parents = setup.surface().parents();
        const auto active_uses = active.parents();
        for (std::size_t active_parent = 0;
             active_parent < active_uses.size(); ++active_parent) {
            const auto& parent = active_uses[active_parent];
            output::Require(parent.surface_parent <
                    surface_parents.size() &&
                    active_parent < UINT32_MAX &&
                    surface_to_active[parent.surface_parent] ==
                        UINT32_MAX,
                "Active-use to S0 parent map is not one-to-one");
            const auto& surface =
                surface_parents[parent.surface_parent];
            output::Require(SameParentSource(parent.source,
                    surface.source) &&
                    parent.arity == surface.arity &&
                    parent.facet_count ==
                        setup.facets().facet_count(
                            parent.surface_parent) &&
                    parent.facet_offset <= UINT32_MAX,
                "Active-use parent source/facets differ from S0");
            surface_to_active[parent.surface_parent] =
                static_cast<std::uint32_t>(active_parent);
            auto& row = active_parents[active_parent];
            row.source_parent_id = parent.source.source_parent_id;
            row.surface_parent =
                static_cast<std::uint32_t>(parent.surface_parent);
            row.facet_count = parent.facet_count;
            row.facet_offset = parent.facet_offset;
            row.reference_half_thickness_m =
                parent.reference_half_thickness_m;
            row.complete_rigid_group =
                CompleteRigidGroup(*rigid, parent);
            row.arity = static_cast<std::uint8_t>(parent.arity);
            for (unsigned local = 0; local < parent.arity; ++local)
                row.vertices[local] = surface.vertices[local];
        }
        output::Require(cudaMemcpyAsync(
                host_keys, pairs.device_keys,
                workspace.pair_keys.bytes,
                cudaMemcpyDeviceToHost, owner_stream) == cudaSuccess &&
                cudaMemcpyAsync(
                    accepted_positions,
                    assembly.accepted.position_xyz,
                    workspace.accepted_positions.bytes,
                    cudaMemcpyDeviceToHost,
                    owner_stream) == cudaSuccess &&
                cudaStreamSynchronize(owner_stream) == cudaSuccess,
            "Complete initial pair/accepted-geometry readback failed");
        auto counted = CountInitialFacetCapacity(
            host_keys, exact_capacity, surface_to_active,
            V5SelectedParents, active_parents, V5SelectedParents,
            &result.capacity);
        output::Require(counted.status ==
                InitialCensusValueStatus::Ok,
            counted.message);
        result.source.surface_active_source_hash =
            result.capacity.surface_active_source_hash;
        const auto geometry_started =
            std::chrono::steady_clock::now();
        contact::FixedContactFacetReadCursor facet_reader;
        auto facet_report = facet_reader.Initialize(setup.facets());
        output::Require(facet_report.status ==
                contact::FixedContactFacetStatus::Ok,
            facet_report.message);
        const contact::VectorView accepted_view{
            accepted_positions,
            static_cast<std::uint32_t>(initial_stamp.node_count), 3, 1};
        for (std::size_t active_parent = 0;
             active_parent < active_uses.size(); ++active_parent) {
            const auto& parent = active_uses[active_parent];
            const auto& row = active_parents[active_parent];
            output::Require(row.facet_offset <= V5Level0Facets &&
                    row.facet_count <=
                        V5Level0Facets - row.facet_offset,
                "Accepted represented facet range is invalid");
            for (std::uint32_t local = 0;
                 local < row.facet_count; ++local) {
                const auto facet = row.facet_offset + local;
                const auto described =
                    facet_reader.Describe(parent.surface_parent, local);
                output::Require(described.report.status ==
                        contact::FixedContactFacetStatus::Ok &&
                        described.facet && facet < active.facet_uses().size() &&
                        active.facet_uses()[facet].parent == active_parent &&
                        active.facet_uses()[facet].local_facet == local,
                    "Accepted represented facet identity is incomplete");
                output::Require(contact::EvaluateCurrentFixedTriangle(
                        *described.facet, accepted_view,
                        represented_triangles + facet) ==
                        contact::Status::kOk,
                    "Accepted owner facet geometry cannot be represented");
            }
        }
        result.geometry_evaluation_us =
            static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() -
                    geometry_started).count());
        const auto filter_started =
            std::chrono::steady_clock::now();
        counted = CountInitialFacetFilterCensus(
            host_keys, exact_capacity, surface_to_active,
            V5SelectedParents, active_parents, V5SelectedParents,
            represented_triangles, V5Level0Facets,
            result.capacity.surface_active_source_hash,
            exact_sample_pairs,
            InitialExactFeatureSampleCapacity,
            &result.filters);
        result.filter_census_us =
            static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() -
                    filter_started).count());
        output::Require(counted.status ==
                InitialCensusValueStatus::Ok &&
                result.filters.represented_facet_pairs ==
                    result.capacity.level0_facet_pairs,
            counted.message);
        result.exact_pair_count = pairs.count;
        result.exact_capacity_succeeded = true;
        result.complete_device_pair_keys = true;

        report = exact.Evaluate(input, owner_stream);
        output::Require(report.status == BroadphaseStatus::Ok,
            report.message);
        pairs = exact.pairs();
        output::Require(pairs.complete && pairs.count == required &&
                pairs.device_keys == stable_device_keys &&
                cudaMemcpyAsync(host_keys, pairs.device_keys,
                    workspace.pair_keys.bytes,
                    cudaMemcpyDeviceToHost,
                    owner_stream) == cudaSuccess &&
                cudaStreamSynchronize(owner_stream) == cudaSuccess,
            "Deterministic exact broadphase rerun/readback failed");
        InitialFacetCapacityCensus rerun;
        counted = CountInitialFacetCapacity(
            host_keys, exact_capacity, surface_to_active,
            V5SelectedParents, active_parents, V5SelectedParents,
            &rerun);
        output::Require(counted.status ==
                InitialCensusValueStatus::Ok &&
                Same(result.capacity, rerun),
            "Initial broadphase rerun hash/census differs");
        InitialFacetFilterCensus rerun_filters;
        const auto rerun_filter_started =
            std::chrono::steady_clock::now();
        counted = CountInitialFacetFilterCensus(
            host_keys, exact_capacity, surface_to_active,
            V5SelectedParents, active_parents, V5SelectedParents,
            represented_triangles, V5Level0Facets,
            rerun.surface_active_source_hash,
            exact_sample_pairs,
            InitialExactFeatureSampleCapacity,
            &rerun_filters);
        result.rerun_filter_census_us =
            static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() -
                    rerun_filter_started).count());
        output::Require(counted.status ==
                InitialCensusValueStatus::Ok &&
                Same(result.filters, rerun_filters),
            "Initial production filter rerun hash/census differs");
        result.rerun_pair_key_hash = rerun.pair_key_hash;
        result.rerun_filter_hash = rerun_filters.category_hash;
        result.deterministic_rerun = true;
        result.deterministic_filter_rerun = true;

        output::Require(result.filters.exact_sample_count ==
                std::min(result.filters.exact_remaining,
                    InitialExactFeatureSampleCapacity) &&
                result.filters.exact_sample_count > 0 &&
                result.filters.exact_sample_hash != 0,
            "Initial exact feature sample retention is incomplete");
        bool qualify_with_worker_one = false;
        {
            contact::FixedTriangleFeatureDiscovery discovery;
            const auto feature_report =
                discovery.Initialize(feature_limits);
            output::Require(feature_report.status ==
                    contact::FixedTriangleDiscoveryStatus::Ok &&
                    discovery.forecast().owned_host_bytes ==
                        result.forecast.exact_feature_discovery.
                            owned_host_bytes &&
                    discovery.forecast().worker_count == 4,
                feature_report.message);
            const auto sampled = DiscoverInitialFeatureSample(
                discovery, represented_triangles, V5Level0Facets,
                exact_sample_pairs,
                result.filters.exact_sample_count,
                feature_task_masks,
                InitialExactFeatureChunkCapacity,
                &result.feature_sample);
            if (sampled.status != InitialFeatureSampleStatus::Ok)
                FeatureSampleFailure(sampled);
            output::Require(
                result.feature_sample.complete.sampled_pairs ==
                    result.filters.exact_sample_count &&
                result.feature_sample.complete.potential_tasks ==
                    result.feature_sample.complete.local_masked_tasks +
                    result.feature_sample.complete.exact_executed_tasks &&
                result.feature_sample.complete.feature_hash != 0 &&
                result.feature_sample.complete.intersection_hash != 0,
                "Initial exact feature sample accounting is incomplete");
            if (result.feature_sample.discovery_us <
                    InitialExactFeatureRerunThresholdUs) {
                InitialFeatureSampleResult rerun_sample;
                const auto rerun_report =
                    DiscoverInitialFeatureSample(
                        discovery, represented_triangles,
                        V5Level0Facets, exact_sample_pairs,
                        result.filters.exact_sample_count,
                        feature_task_masks,
                        InitialExactFeatureChunkCapacity,
                        &rerun_sample);
                if (rerun_report.status !=
                        InitialFeatureSampleStatus::Ok)
                    FeatureSampleFailure(rerun_report);
                output::Require(
                    SameInitialFeatureSampleIdentity(
                        result.feature_sample.complete,
                        rerun_sample.complete),
                    "Initial exact feature sample rerun differs");
                result.feature_sample_rerun_us =
                    rerun_sample.discovery_us;
                result.deterministic_feature_sample = true;
            } else {
                qualify_with_worker_one = true;
            }
        }
        if (qualify_with_worker_one) {
            const auto worker_one_limits =
                InitialFeatureSampleDiscoveryLimits(
                    InitialExactFeatureChunkCapacity, 1);
            contact::FixedTriangleFeatureDiscovery worker_one;
            const auto worker_one_report =
                worker_one.Initialize(worker_one_limits);
            output::Require(worker_one_report.status ==
                    contact::FixedTriangleDiscoveryStatus::Ok,
                worker_one_report.message);
            InitialFeatureSampleResult prefix;
            const auto prefix_report =
                DiscoverInitialFeatureSample(
                    worker_one, represented_triangles,
                    V5Level0Facets, exact_sample_pairs,
                    result.feature_sample.worker_prefix_pairs,
                    feature_task_masks,
                    InitialExactFeatureChunkCapacity, &prefix);
            if (prefix_report.status !=
                    InitialFeatureSampleStatus::Ok)
                FeatureSampleFailure(prefix_report);
            output::Require(
                SameInitialFeatureSampleIdentity(
                    result.feature_sample.worker_prefix,
                    prefix.complete),
                "Initial exact feature worker-1 prefix differs");
            result.worker_one_prefix_us = prefix.discovery_us;
            result.worker_one_prefix_identity = true;
            result.deterministic_feature_sample = true;
        }
    }

    state.owner.Discard();
    trial.Release();
    result.accepted_owner_unchanged =
        fe::trial_identity::SameStamp(
            state.owner.accepted(), initial_stamp) &&
        fe::trial_identity::SameStamp(
            dynamics.accepted(), initial_stamp) &&
        !dynamics.has_prepared_step();
    output::Require(result.accepted_owner_unchanged,
        "Initial census changed the accepted physical owner");
    return result;
}

}  // namespace crash::cases::vehicle_self_contact
