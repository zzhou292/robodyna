#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace crash::output::physical_run {
// Value-only archive observation. No live receipt, pointer or restart authority.
// Forces/potential belong to the accepted BASE; policy/activity to its candidate.
struct SelfContactValues {
    std::uint64_t source_id=0,selected_parents=0;
    std::uint64_t events=0,vertex_face_events=0,boundary_vertex_edge_events=0,edge_edge_events=0,active_events=0;
    std::uint64_t accepted_parent_pairs=0,accepted_facet_pairs=0,discovered_features=0,regularity_generation=0;
    std::uint64_t candidate_parent_pairs=0,candidate_facet_pairs=0,policy_outcomes=0;
    std::uint64_t certified_separated=0,same_rigid_exclusions=0,local_intersections=0;
    std::uint64_t represented_vf=0,represented_ee=0,policy_digest=0;
    std::uint64_t active_parents=0,removing_parents=0,skipped_parents=0;
    double base_velocity_time=0,potential_j=0,maximum_force_n=0;
    double maximum_sti_n_m=0,maximum_represented_stiffness_n_m=0;
    std::array<double,3> endpoint_a_n{},endpoint_b_n{},equal_opposite_residual_n{},global_moment_n_m{};
};
inline constexpr std::size_t SelfContactIntegerCount=23,SelfContactRealCount=17;
void CheckSelfContactValues(const SelfContactValues&);
std::vector<std::string> SelfContactIntegerFields();
std::vector<std::string> SelfContactRealFields();
void EncodeSelfContact(const SelfContactValues&,std::uint64_t*,double*);
SelfContactValues DecodeSelfContact(const std::uint64_t*,const double*);
} // namespace crash::output::physical_run
