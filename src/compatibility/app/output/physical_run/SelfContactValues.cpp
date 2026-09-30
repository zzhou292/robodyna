#include "SelfContactValues.h"
#include "output/ArtifactIO.h"
#include <cmath>
#include <limits>

namespace crash::output::physical_run {
namespace {
struct Integer {const char* name;std::uint64_t SelfContactValues::*member;};
constexpr Integer Integers[]{
    {"self_source_id",&SelfContactValues::source_id},
    {"self_selected_parents",&SelfContactValues::selected_parents},
    {"self_base_events",&SelfContactValues::events},
    {"self_base_vf_events",&SelfContactValues::vertex_face_events},
    {"self_base_boundary_ve_events",&SelfContactValues::boundary_vertex_edge_events},
    {"self_base_ee_events",&SelfContactValues::edge_edge_events},
    {"self_base_active_events",&SelfContactValues::active_events},
    {"self_base_parent_pairs",&SelfContactValues::accepted_parent_pairs},
    {"self_base_facet_pairs",&SelfContactValues::accepted_facet_pairs},
    {"self_base_discovered_features",&SelfContactValues::discovered_features},
    {"self_regularity_generation",&SelfContactValues::regularity_generation},
    {"self_candidate_parent_pairs",&SelfContactValues::candidate_parent_pairs},
    {"self_candidate_facet_pairs",&SelfContactValues::candidate_facet_pairs},
    {"self_policy_outcomes",&SelfContactValues::policy_outcomes},
    {"self_certified_separated",&SelfContactValues::certified_separated},
    {"self_same_rigid_exclusions",&SelfContactValues::same_rigid_exclusions},
    {"self_local_intersections",&SelfContactValues::local_intersections},
    {"self_represented_vf",&SelfContactValues::represented_vf},
    {"self_represented_ee",&SelfContactValues::represented_ee},
    {"self_policy_digest",&SelfContactValues::policy_digest},
    {"self_candidate_active_parents",&SelfContactValues::active_parents},
    {"self_candidate_removing_parents",&SelfContactValues::removing_parents},
    {"self_candidate_skipped_parents",&SelfContactValues::skipped_parents}};
struct Real {const char* name;double SelfContactValues::*member;};
constexpr Real Reals[]{
    {"self_base_velocity_time_s",&SelfContactValues::base_velocity_time},
    {"self_base_potential_J",&SelfContactValues::potential_j},
    {"self_base_maximum_force_N",&SelfContactValues::maximum_force_n},
    {"self_base_maximum_sti_N_m",&SelfContactValues::maximum_sti_n_m},
    {"self_base_maximum_represented_stiffness_N_m",&SelfContactValues::maximum_represented_stiffness_n_m}};
struct Vector {const char* name;std::array<double,3> SelfContactValues::*member;};
constexpr Vector Vectors[]{
    {"self_base_endpoint_a_N",&SelfContactValues::endpoint_a_n},
    {"self_base_endpoint_b_N",&SelfContactValues::endpoint_b_n},
    {"self_base_equal_opposite_residual_N",&SelfContactValues::equal_opposite_residual_n},
    {"self_base_global_moment_N_m",&SelfContactValues::global_moment_n_m}};
static_assert(std::size(Integers)==SelfContactIntegerCount);
static_assert(std::size(Reals)+3*std::size(Vectors)==SelfContactRealCount);
std::uint64_t Sum(std::initializer_list<std::uint64_t> values) {
    std::uint64_t total=0;
    for(auto value:values) {
        Require(value<=UINT64_MAX-total,"Self-contact count sum overflows");total+=value;
    }
    return total;
}
}
void CheckSelfContactValues(const SelfContactValues& v) {
    Require(v.source_id && v.selected_parents && v.regularity_generation &&
        v.events==Sum({v.vertex_face_events,v.edge_edge_events}) &&
        v.boundary_vertex_edge_events<=v.vertex_face_events && v.active_events<=v.events &&
        (!v.represented_vf || v.vertex_face_events) && (!v.represented_ee || v.edge_edge_events) &&
        v.policy_outcomes==v.candidate_facet_pairs &&
        v.policy_outcomes==Sum({v.certified_separated,v.same_rigid_exclusions,v.local_intersections,
            v.represented_vf,v.represented_ee}) &&
        v.selected_parents==Sum({v.active_parents,v.removing_parents,v.skipped_parents}),
        "Self-contact accepted counts or complete candidate partition differ");
    for(const auto& item:Reals)
        Require(std::isfinite(v.*item.member) && v.*item.member>=0,
            "Self-contact base force observation is nonfinite or negative");
    for(const auto& item:Vectors)for(auto value:v.*item.member)
        Require(std::isfinite(value),"Self-contact base resultant is nonfinite");
}
std::vector<std::string> SelfContactIntegerFields() {
    std::vector<std::string> fields;for(const auto& item:Integers)fields.emplace_back(item.name);return fields;
}
std::vector<std::string> SelfContactRealFields() {
    std::vector<std::string> fields;for(const auto& item:Reals)fields.emplace_back(item.name);
    for(const auto& item:Vectors)for(const auto* axis:{"x","y","z"})fields.emplace_back(std::string(item.name)+"_"+axis);
    return fields;
}
void EncodeSelfContact(const SelfContactValues& v,std::uint64_t* integers,double* reals) {
    for(const auto& item:Integers)*integers++=v.*item.member;
    for(const auto& item:Reals)*reals++=v.*item.member;
    for(const auto& item:Vectors)for(auto value:v.*item.member)*reals++=value;
}
SelfContactValues DecodeSelfContact(const std::uint64_t* integers,const double* reals) {
    SelfContactValues v;
    for(const auto& item:Integers)v.*item.member=*integers++;
    for(const auto& item:Reals)v.*item.member=*reals++;
    for(const auto& item:Vectors)for(auto& value:v.*item.member)value=*reals++;
    return v;
}
} // namespace crash::output::physical_run
