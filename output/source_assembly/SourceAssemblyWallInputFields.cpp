#include "SourceAssemblyWallFields.h"
#include "WallFieldValues.h"
#include "SourceAssemblyConnectorFields.h"
#include <algorithm>

namespace crash::output::assembly::wall_fields {
namespace {
Document Group(const tl::fea::NodalRigidGroupModel& model,std::size_t index) {
    const auto& g=model.groups()[index];Document d;d.SetObject();
    Integer(d,"source_group_id",g.source_group_id);Integer(d,"source_node_set_id",g.source_node_set_id);Integer(d,"member_count",g.member_count);
    Number(d,"structural_mass_kg",g.structural_mass_kg);Number(d,"total_mass_kg",g.total_mass_kg);
    Number(d,"native_total_inertia_sum_kg_m2",g.native_total_inertia_sum);
    Number(d,"physical_inertia_sum_kg_m2",g.physical_inertia_sum);Number(d,"added_inertia_sum_kg_m2",g.added_inertia_sum);
    Put(d,"generated_primary_position_m",Vector(d,g.generated_primary_position));Put(d,"structural_center_m",Vector(d,g.structural_center));
    Put(d,"center_m",Vector(d,g.center));Put(d,"raw_tensor_kg_m2",Matrix(d,g.raw_tensor));Put(d,"effective_tensor_kg_m2",Matrix(d,g.effective_tensor));
    Put(d,"principal_axes_row_major",Matrix(d,g.principal.axes));Put(d,"principal_inertia_kg_m2",Vector(d,g.principal.inertia));
    Put(d,"raw_principal_inertia_kg_m2",Vector(d,g.raw_principal_inertia));
    Document r;r.SetObject();Number(r,"primary_mass_kg",g.regularization.primary_mass_kg);
    Number(r,"primary_isotropic_inertia_kg_m2",g.regularization.primary_isotropic_inertia_kg_m2);
    Put(r,"principal_inertia_added_kg_m2",Vector(r,g.regularization.principal_inertia_added));Put(r,"tensor_added_kg_m2",Matrix(r,g.regularization.tensor_added));
    Boolean(r,"principal_threshold_reached",g.regularization.principal_threshold_reached);
    Boolean(r,"principal_inertia_changed",g.regularization.principal_inertia_changed);Child(d,"regularization",r);
    String(d,"member_columns","source_node_id,global_node,reference_xyz_m,mass_kg,total_inertia_kg_m2,physical_inertia_kg_m2,added_inertia_kg_m2");
    Value members(rapidjson::kArrayType);for(std::size_t j=0;j<g.member_count;++j) {
        const auto& m=model.members()[g.member_offset+j];auto row=Ids(d,{m.source_node_id,m.global_node});
        row.PushBack(Vector(d,m.position),d.GetAllocator());
        for(double x:{m.mass_kg,m.total_inertia_kg_m2,m.physical_inertia_kg_m2,m.added_inertia_kg_m2})row.PushBack(x,d.GetAllocator());
        members.PushBack(row,d.GetAllocator());
    }
    Put(d,"members",std::move(members));return d;
}
}
Document InputDocument(const cases::source_assembly::SourceAssemblyBindings& b) {
    const auto& s=b.source().data();Document d;d.SetObject();
    String(d,"inventory_file","source-assembly-inventory.json");String(d,"inventory_schema",s.schema);
    String(d,"inventory_sha256",s.identity.sha256);Integer(d,"inventory_bytes",s.identity.bytes);
    Integer(d,"node_count",s.nodes.size());Integer(d,"parent_count",s.parents.size());Integer(d,"qeph_count",s.qeph_count);Integer(d,"t3_count",s.t3_count);
    String(d,"material_rate_policy","openradioss_direct_import_default");
    String(d,"rate_resolution","Source VP=0 and C/P retained; absent direct-import Fcut resolves through ISMOOTH=1 to 10000 per second");
    Put(d,"source_units_to_SI",Values(d,{s.units.mass_to_kg,s.units.length_to_m,s.units.time_to_s}));
    String(d,"part_columns","source_part_id,source_material_id,source_section_id,first_parent,parent_count");
    Value parts(rapidjson::kArrayType);for(const auto& p:s.parts)parts.PushBack(Ids(d,{p.id,p.material_id,p.section_id,p.first_parent,p.parent_count}),d.GetAllocator());
    Put(d,"parts",std::move(parts));
    String(d,"material_columns","source_material_id,source_curve_id,density_kg_m3,young_Pa,poisson_ratio,source_rate_C_per_s,source_rate_P,source_VP,resolved_rate_enabled,resolved_cutoff_Hz");
    Value materials(rapidjson::kArrayType);for(const auto& m:s.materials) {
        auto row=Ids(d,{m.id,m.curve_id});for(double x:{m.density_kg_m3,m.young_pa,m.poisson_ratio,m.rate_c_per_s,m.rate_p})row.PushBack(x,d.GetAllocator());
        const auto p=std::find_if(s.parents.begin(),s.parents.end(),[&](const auto& p){return p.material_id==m.id;});
        tl::fea::sections::PointParameters resolved;
        Require(p!=s.parents.end()&&b.materials().Parameters(p->family==source::ShellFamily::Qeph?
            tl::fea::ShellBindingFamily::Qeph:tl::fea::ShellBindingFamily::T3,p->family_index,&resolved),"Missing prepared source material");
        row.PushBack(m.source_rate_type,d.GetAllocator());row.PushBack(resolved.rate.enabled,d.GetAllocator());
        row.PushBack(resolved.rate.cutoff_hz,d.GetAllocator());materials.PushBack(row,d.GetAllocator());
    }
    Put(d,"materials",std::move(materials));
    String(d,"section_columns","source_section_id,source_elform,through_thickness_points,source_thickness_m");
    Value sections(rapidjson::kArrayType);for(const auto& s:s.sections) {
        auto row=Ids(d,{s.id,s.source_elform,s.through_thickness_points});row.PushBack(FiniteArray(d,s.thickness_m.data(),4),d.GetAllocator());sections.PushBack(row,d.GetAllocator());
    }
    Put(d,"sections",std::move(sections));
    String(d,"curve_columns","source_curve_id,equivalent_plastic_strain,true_stress_Pa");
    Value curves(rapidjson::kArrayType);for(const auto& c:s.curves) {
        auto row=Ids(d,{c.id});row.PushBack(FiniteArray(d,c.plastic_strain.data(),c.plastic_strain.size()),d.GetAllocator());
        row.PushBack(FiniteArray(d,c.stress_pa.data(),c.stress_pa.size()),d.GetAllocator());curves.PushBack(row,d.GetAllocator());
    }
    Put(d,"curves",std::move(curves));
    String(d,"native_node_columns","global_node,source_node_id,mass_kg,total_inertia_kg_m2,physical_inertia_kg_m2,added_inertia_kg_m2");
    Value nodes(rapidjson::kArrayType);for(std::size_t n=0;n<s.nodes.size();++n) {
        const auto& m=b.shells().nodes()[n].native;auto row=Ids(d,{n,s.nodes[n].source_id});
        for(double x:{m.mass,m.isotropic_inertia,m.physical_inertia,m.added_inertia})row.PushBack(x,d.GetAllocator());nodes.PushBack(row,d.GetAllocator());
    }
    Put(d,"native_nodes",std::move(nodes));
    if(b.connectors())Child(d,"connectors",ConnectorInputDocument(b));
    String(d,"part_native_columns","source_part_id,parent_count,qeph_count,t3_count,mass_kg,total_inertia_kg_m2,physical_inertia_kg_m2,added_inertia_kg_m2");
    Value ledger(rapidjson::kArrayType);for(const auto& p:b.part_mass_ledger()) {
        auto row=Ids(d,{p.source_part_id,p.parent_count,p.qeph_count,p.t3_count});
        for(double x:{p.native.mass,p.native.isotropic_inertia,p.native.physical_inertia,p.native.added_inertia})row.PushBack(x,d.GetAllocator());ledger.PushBack(row,d.GetAllocator());
    }
    Put(d,"part_native_ledger",std::move(ledger));Value groups(rapidjson::kArrayType);
    if(const auto* g=b.rigid_groups())for(std::size_t i=0;i<g->group_count();++i) {Value v;v.CopyFrom(Group(*g,i),d.GetAllocator());groups.PushBack(v,d.GetAllocator());}
    Put(d,"internal_rigid_groups",std::move(groups));
    Document boundary;boundary.SetObject();String(boundary,"policy",s.boundary.policy);
    String(boundary,"interpretation",s.boundary.interpretation);String(boundary,"unresolved_tied_scope",s.boundary.unresolved_tied_scope);
    auto ids=[&](const char* key,const auto& source) {Value a(rapidjson::kArrayType);for(auto id:source)a.PushBack(Value().SetUint64(id),boundary.GetAllocator());Put(boundary,key,std::move(a));};
    ids("released_nodal_rigid_ids",s.boundary.nodal_rigid_ids);ids("released_spotweld_ids",s.boundary.spotweld_ids);
    ids("external_node_ids",s.boundary.external_node_ids);ids("external_part_ids",s.boundary.external_part_ids);
    Value ties(rapidjson::kArrayType);for(const auto& scope:s.boundary.tied_scopes) {
        Document item;item.SetObject();String(item,"filename",scope.filename);Integer(item,"source_line",scope.source_line);String(item,"reason",scope.reason);
        Value master(rapidjson::kArrayType),slave(rapidjson::kArrayType);
        for(auto id:scope.selected_master_parts)master.PushBack(Value().SetUint64(id),item.GetAllocator());
        for(auto id:scope.selected_slave_parts)slave.PushBack(Value().SetUint64(id),item.GetAllocator());
        Put(item,"selected_master_parts",std::move(master));Put(item,"selected_slave_parts",std::move(slave));Boolean(item,"pairing_qualified",false);
        Value value;value.CopyFrom(item,boundary.GetAllocator());ties.PushBack(value,boundary.GetAllocator());
    }
    Put(boundary,"released_tied_scopes",std::move(ties));Child(d,"boundary",boundary);
    String(d,"boundary_source_scope","All original memberships, external endpoints and attachment cards remain in the authenticated inventory file");
    return d;
}
} // namespace crash::output::assembly::wall_fields
