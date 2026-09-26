#include "Internal.h"
#include "case/vehicle_self_contact/native/TopologyDigestFields.h"
namespace crash::cases::vehicle_wall::native::detail {
std::string Digest(const source::CanonicalData& source,const Declaration& d,const NamespaceReport& ns,
        const AllocatedIds& ids,const Geometry& g,const tl::fea::NodalNodeDomain& domain,const VehiclePrefix& vehicle,std::size_t cap) {
    namespace hash=vehicle_self_contact::native::detail::digest;
    hash::Fields f("retained-v5-envelope-domain-v1:"+source.inputs.canonical_manifest.sha256+ns.digest,cap);
    const double declared[]{d.material.young_pa,d.material.poisson,d.material.density_kg_m3,d.material.thickness_m,
        d.wall_friction,d.requested_duration_s,d.leading_gap_m,d.transverse_margin_m,d.exposed_clearance_m};
    f.Add<double>("explicit_declaration",1,std::size(declared),[&](auto i){return declared[i];});
    const std::uint64_t policy[]{std::uint64_t(d.profile),d.binding_id,vehicle.begin,vehicle.nodes,domain.node_count(),
        domain.source_instance_id(),std::uint64_t(ns.phase),std::uint64_t(ns.generated_node_after_complete_input)};
    f.Add<std::uint64_t>("source_and_prefix_scope",1,std::size(policy),[&](auto i){return policy[i];});
    const std::uint64_t allocated[]{ids.nodes[0],ids.nodes[1],ids.nodes[2],ids.nodes[3],ids.shell,ids.part,
        ids.material,ids.section,ids.node_set,ids.surface,ids.interface};
    f.Add<std::uint64_t>("allocated_native_ids",1,std::size(allocated),[&](auto i){return allocated[i];});
    f.Add<std::uint64_t>("complete_domain_ids",domain.node_count(),1,[&](auto i){return domain.nodes()[i].source_id;});
    f.Add<double>("complete_domain_positions_si",domain.node_count(),3,[&](auto i){const auto p=domain.nodes()[i/3].position;
        return i%3==0?p.x:i%3==1?p.y:p.z;});
    f.Add<double>("vehicle_only_bounds",2,3,[&](auto i){const auto p=vehicle.reference_bounds[i/3];
        return i%3==0?p.x:i%3==1?p.y:p.z;});
    const double geometry[]{g.placement.translation_x_m,g.placement.represented_wall_x_m,g.native_half_gap,
        g.reference_offset_m,g.reference_plane_m,g.component_primary_stiffness_native,g.wall_mass_kg,g.native_working_length_m};
    f.Add<double>("derived_component_values",1,std::size(geometry),[&](auto i){return geometry[i];});
    f.Add<double>("wall_working_coordinates",4,3,[&](auto i){const auto p=g.reference_native[i/3];
        return i%3==0?p.x:i%3==1?p.y:p.z;});
    f.Add<double>("wall_raw_mass_inertia",4,2,[&](auto i){return i%2?g.reference.isotropic_inertia[i/2]:g.reference.nodal_mass[i/2];});
    return f.Finish().sha256;
}
} // namespace crash::cases::vehicle_wall::native::detail
