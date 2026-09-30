#include "SourceAssemblyWallFields.h"
#include "WallFieldValues.h"
#include <algorithm>

namespace crash::output::assembly::wall_fields {
const source::Parent& ContactSourceParent(const cases::source_assembly::SourceAssemblyBindings& b,
    const cases::ShellCollectionContactGeometry& geometry,std::size_t index) {
    const auto* mapped=geometry.parent_from_weight(index);Require(mapped,"Missing native contact family mapping");
    const auto& parents=b.source().data().parents;
    const auto p=std::find_if(parents.begin(),parents.end(),[&](const auto& p){return p.source_id==mapped->source_id;});
    Require(p!=parents.end()&&p->family_index==mapped->family_index&&
        (p->family==source::ShellFamily::Qeph?tl::fea::ShellBindingFamily::Qeph:tl::fea::ShellBindingFamily::T3)==mapped->family,
        "Native contact mapping differs from original source parent");return *p;
}
Document SetupDocument(const cases::source_assembly::SourceAssemblyWallSetup& setup) {
    Require(setup.initialized(),"Missing immutable assembly wall setup");const auto& s=*setup.settings();const auto& c=*setup.certificate();
    const auto& k=c.initial_kinetic;const auto& p=c.penalty;Document d;d.SetObject();
    String(d,"contact_model",tlfea::contact::NodalWallContactModel);String(d,"boundary_policy","released_external_connections");
    String(d,"certificate_columns","value,lower,upper,error");Integer(d,"configuration_id",s.configuration_id);
    Integer(d,"qualification_id",s.qualification_id);Integer(d,"wall_binding_id",s.wall_binding_id);
    FiniteArray(d,"initial_velocity_xyz_m_per_s",s.initial_velocity.data(),3);Number(d,"declared_leading_gap_m",s.leading_gap);
    Put(d,"actual_leading_gap_interval_m",Values(d,{c.leading_gap.lower,c.leading_gap.upper}));
    Number(d,"area_floor_m2",s.penalty.area_floor);Number(d,"design_penetration_m",s.penalty.design_penetration);
    Number(d,"penetration_cap_m",s.penalty.penetration_cap);Number(d,"kinetic_budget_factor",s.penalty.kinetic_budget_factor);
    Number(d,"kinetic_budget_upper_J",p.kinetic_budget_upper);Number(d,"stiffness_per_area_N_m3",p.stiffness_per_area);
    Number(d,"design_potential_lower_J",p.design_potential_lower);Number(d,"minimum_nodal_area_lower_m2",p.minimum_nodal_area_lower);
    Integer(d,"minimum_area_node",p.minimum_area_node);Integer(d,"kappa_upward_steps",p.kappa_upward_steps);
    Put(d,"native_initial_kinetic_J",Certificate(d,k.native_nodes));Put(d,"aggregate_initial_kinetic_J",Certificate(d,k.with_aggregate_groups));
    Put(d,"generated_primary_mass_kg",Certificate(d,k.generated_primary_mass_kg));
    Integer(d,"ordinary_nodes",k.ordinary_nodes);Integer(d,"member_nodes",k.member_nodes);Integer(d,"generated_primaries",k.generated_primaries);
    String(d,"initial_metric","Uniform positive X translation and zero spin; ordinary native nodes plus each group total mass once; primary already included");
    String(d,"group_initial_columns","source_group_id,source_node_set_id,member_count,native_members_J,aggregate_J,generated_primary_mass_kg");
    Value groups(rapidjson::kArrayType);for(const auto& g:k.groups) {
        auto row=Ids(d,{g.source_group_id,g.source_node_set_id,g.member_count});row.PushBack(Certificate(d,g.native_members),d.GetAllocator());
        row.PushBack(Certificate(d,g.aggregate),d.GetAllocator());row.PushBack(g.generated_primary_mass_kg,d.GetAllocator());groups.PushBack(row,d.GetAllocator());
    }
    Put(d,"group_initial_kinetic",std::move(groups));Number(d,"motion_margin_m",s.motion_margin);Number(d,"exposed_clearance_m",s.exposed_clearance);
    Number(d,"parent_force_error_N",s.parent_force_error);Number(d,"parent_energy_error_J",s.parent_energy_error);
    Number(d,"maximum_contact_step_rate",s.maximum_step_rate);Boolean(d,"projected_motion_covered",c.coverage.covered);
    Put(d,"projected_motion_minimum_m",Vector(d,c.coverage.physical.minimum));Put(d,"projected_motion_maximum_m",Vector(d,c.coverage.physical.maximum));
    Put(d,"query_minimum_m",Vector(d,c.coverage.query.minimum));Put(d,"query_maximum_m",Vector(d,c.coverage.query.maximum));
    Put(d,"query_lower_expansion_upper_m",Vector(d,c.coverage.lower_expansion_upper));Put(d,"query_upper_expansion_upper_m",Vector(d,c.coverage.upper_expansion_upper));
    String(d,"wall_placement_file","placed-wall-placement.json");String(d,"wall_mesh_file","placed-wall.mesh.json");
    String(d,"original_wall_manifest_file","original-canonical-wall.manifest.json");
    const auto& geometry=*setup.source_geometry();const auto& w=*geometry.weights();Value nodes(rapidjson::kArrayType),parents(rapidjson::kArrayType);
    for(unsigned n=0;n<w.node_count();++n)nodes.PushBack(Certificate(d,w.node(n).area),d.GetAllocator());
    String(d,"contact_parent_columns","weight_index,source_parent_index,source_element_id,arity,area_m2,share_m2");
    for(unsigned e=0;e<w.parent_count();++e) {
        const auto& p=w.parent(e);const auto& source=ContactSourceParent(*setup.bindings(),geometry,e);
        auto row=Ids(d,{e,source.index,p.parent_element_id,p.arity});
        row.PushBack(Certificate(d,p.area),d.GetAllocator());row.PushBack(Certificate(d,p.share),d.GetAllocator());parents.PushBack(row,d.GetAllocator());
    }
    Put(d,"nodal_area_m2",std::move(nodes));Put(d,"contact_parents",std::move(parents));return d;
}
} // namespace crash::output::assembly::wall_fields
