#include "Internal.h"
#include "../TopologyDigestFields.h"
namespace crash::cases::vehicle_self_contact::native::post_gapm::detail {
std::string Digest(const Mixed& source, const GapOperands& gaps, const Values& v, std::size_t cap) {
    native::detail::digest::Fields h("post-gapm-main-source-v1",cap);
    const auto identity=source.provenance().output_digest+":"+gaps.provenance().operand_digest;
    h.Add<std::uint64_t>("input_identity",identity.size(),1,[&](auto i){return std::uint64_t(static_cast<unsigned char>(identity[i]));});
    h.Add<std::uint64_t>("node_source_ids",v.node_ids.size(),1,[&](auto i){return v.node_ids[i];});
    h.Add<double>("native_positions",v.node_ids.size(),3,[&](auto i){return v.positions[i];});
    h.Add<std::uint64_t>("primary_corners",v.corners.size(),4,[&](auto i){return v.corners[i/4].source_corner[i%4];});
    h.Add<std::uint64_t>("before_shell_support",v.before_shell.size(),3,[&](auto i){
        const auto& a=v.before_shell[i/3];const std::uint64_t x[]{a.first_solid_source_id,a.second_solid_source_id,a.unique_match_count};return x[i%3];});
    h.Add<std::uint64_t>("final_support",v.final_support.size(),3,[&](auto i){
        const auto& a=v.final_support[i/3];const std::uint64_t x[]{std::uint64_t(a.first.kind),a.first.source_element_id,a.second_solid_source_id};return x[i%3];});
    h.Add<std::uint64_t>("physical_owners",v.owners.size(),4,[&](auto i){
        const auto& a=v.owners[i/4];const std::uint64_t x[]{std::uint64_t(a.kind),a.source_element,a.source_part,a.physical_row};return x[i%4];});
    h.Add<std::uint64_t>("geometry_defined",v.geometry.size(),1,[&](auto i){return v.geometry[i].defined;});
    h.Add<double>("geometry",v.geometry.size(),5,[&](auto i){
        const auto& a=v.geometry[i/5];const double x[]{a.area,a.first_volume,a.second_volume,a.solid_length,a.exterior_projection};return x[i%5];});
    h.Add<double>("main_coefficients",v.coefficients.size(),1,[&](auto i){return v.coefficients[i];});
    h.Add<std::uint64_t>("secondary_nodes",v.secondary_nodes.size(),1,[&](auto i){return v.secondary_nodes[i];});
    h.Add<std::uint64_t>("main_nodes",v.main_nodes.size(),1,[&](auto i){return v.main_nodes[i];});
    h.Add<double>("secondary_gaps",v.secondary_gaps.size(),1,[&](auto i){return v.secondary_gaps[i];});
    h.Add<double>("main_node_gaps",v.main_node_gaps.size(),1,[&](auto i){return v.main_node_gaps[i];});
    h.Add<double>("main_gap_fields",v.main_gaps.size(),5,[&](auto i){return i%5<4?v.main_gaps[i/5].corner[i%5]:v.main_gaps[i/5].maximum;});
    h.Add<std::uint64_t>("erosion",1,3,[&](auto i){const std::uint64_t x[]{std::uint64_t(v.incoming_erosion),std::uint64_t(v.final_erosion),v.counts.pre_shell_internal};return x[i];});
    const auto& profile=v.gap_profile;
    h.Add<std::uint64_t>("gap_native_controls",1,6,[&](auto i){const std::uint64_t x[]{std::uint64_t(profile.property_type),
        std::uint64_t(profile.input_thickness_mode),std::uint64_t(profile.level),std::uint64_t(profile.gap_mode),
        std::uint64_t(profile.free_edge_gap),std::uint64_t(profile.contact_thickness_update)};return x[i];});
    h.Add<double>("gap_scale_caps",1,3,[&](auto i){const double x[]{profile.scale,profile.maximum_secondary,profile.maximum_main};return x[i];});
    return h.Finish().sha256;
}
}
