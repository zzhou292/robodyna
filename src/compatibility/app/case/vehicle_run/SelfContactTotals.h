#pragma once

#include "contact_diagnostics/Values.h"
#include <array>
#include <cstdint>

namespace crash::cases::vehicle_run {

// Bounded scalar observations only; no pointers, CUDA types or receipt authority.
// Force/potential are evaluated at the accepted base of the committed interval,
// while policy counts describe the subsequently sealed candidate interval.
struct SelfContactTotals {
    bool available = false;
    std::uint64_t intervals = 0, owner_id = 0;
    std::uint64_t configuration_id = 0, qualification_id = 0, last_attempt = 0;
    double fixed_dt_s = 0, last_base_time_s = 0, last_time_s = 0;
    double last_base_velocity_time_s = 0;
    std::uint64_t last_event_count = 0, peak_event_count = 0, last_active_count = 0;
    std::uint64_t last_vertex_face_events = 0, last_boundary_vertex_edge_events = 0;
    std::uint64_t last_edge_edge_events = 0;
    std::uint64_t last_accepted_parent_pairs = 0, last_accepted_facet_pairs = 0;
    std::uint64_t last_discovered_features = 0;
    std::uint64_t last_candidate_parent_pairs = 0, last_candidate_facet_pairs = 0;
    std::uint64_t last_policy_outcomes = 0, last_policy_digest = 0;
    // Seven existing native work counters (56 bytes), last committed interval
    // only. Operation counts are not elapsed timings or physical energy/work.
    std::uint64_t last_exact_crossing_pairs = 0;
    std::uint64_t last_exact_crossing_work = 0;
    std::uint64_t last_motion_certified_linear_separated = 0;
    std::uint64_t last_linear_policy_coverage_pairs = 0;
    std::uint64_t last_linear_policy_coverage_work = 0;
    std::uint64_t last_nonlinear_subdivision_pairs = 0;
    std::uint64_t last_nonlinear_subdivision_work = 0;
    std::uint64_t last_certified_separated = 0, last_same_rigid_exclusions = 0;
    std::uint64_t last_local_intersections = 0;
    std::uint64_t last_represented_vf = 0, last_represented_ee = 0;
    std::uint64_t last_regularity_generation = 0;
    std::uint64_t last_active_parents = 0, last_removing_parents = 0, last_skipped_parents = 0;
    double last_accepted_base_potential_j = 0, peak_accepted_base_potential_j = 0;
    double last_max_force_n = 0, peak_max_force_n = 0;
    double last_max_sti_n_m = 0, peak_max_sti_n_m = 0;
    double last_max_represented_stiffness_n_m = 0;
    std::array<double, 3> last_equal_opposite_residual_n{};
    std::array<double, 3> last_global_moment_n_m{};
    contact_diagnostics::Snapshot performance;
};

static_assert(sizeof(SelfContactTotals) <= 4096);

}  // namespace crash::cases::vehicle_run
