#include "SourceAssemblyWallFields.h"
#include "WallFieldValues.h"
#include <algorithm>
#include <cmath>

namespace crash::output::assembly::wall_fields {
Document ContactDocument(const FrameView& v) {
    CheckContactPhase(v);
    const auto& a=v.contact;const auto& c=*a.diagnostics;
    const auto& source=v.bindings->source().data();
    const auto& geometry=*v.setup->source_geometry();const auto& w=*geometry.weights();
    Document d;d.SetObject();String(d,"phase","prepared_candidate_of_accepted_interval");
    String(d,"certificate_columns","value,lower,upper,error");Integer(d,"owner_id",c.owner_id);
    Integer(d,"configuration_id",c.configuration_id);Integer(d,"qualification_id",c.qualification_id);Integer(d,"wall_binding_id",c.wall_binding_id);
    Integer(d,"base_epoch",c.base_epoch);Integer(d,"attempt",c.attempt);Number(d,"time_s",c.time);
    Number(d,"velocity_time_s",c.velocity_time);Number(d,"base_time_s",c.base_time);
    Number(d,"base_velocity_time_s",c.base_velocity_time);Number(d,"kick_dt_s",c.kick_dt);
    Put(d,"resultant_N",Certificate(d,c.resultant));Put(d,"potential_J",Certificate(d,c.potential));
    Put(d,"wall_reaction_xyz_N",Vector(d,c.wall_reaction));Put(d,"wall_moment_xyz_N_m",Vector(d,c.wall_moment));
    Put(d,"wall_kick_moment_xyz_N_m_s",Vector(d,c.wall_kick_moment));Put(d,"wall_kick_moment_error_xyz_N_m_s",Vector(d,c.wall_kick_moment_error));
    Number(d,"surface_power_W",c.surface_power);Number(d,"maximum_penetration_m",c.maximum_penetration);
    Number(d,"native_mass_stiffness_rate_bound_per_s2",c.stiffness_rate_bound);
    String(d,"rate_scope","Native physical diagonal mass local diagnostic; not the constrained system inverse or coupled stability proof");
    Number(d,"base_potential_J",c.base_potential);Number(d,"base_potential_error_J",c.base_potential_error);
    Number(d,"potential_increment_J",c.potential_increment);Number(d,"kick_work_J",c.kick_work);
    Number(d,"kick_work_roundoff_J",c.kick_work_roundoff);Number(d,"drift_work_J",c.drift_work);
    Number(d,"drift_work_roundoff_J",c.drift_work_roundoff);Number(d,"conservative_defect_J",c.conservative_defect);
    Number(d,"work_uncertainty_J",c.work_uncertainty);Number(d,"quadratic_work_upper_J",c.quadratic_work_upper);
    Number(d,"wall_kick_impulse_N_s",c.wall_kick_impulse);Number(d,"wall_kick_impulse_error_N_s",c.wall_kick_impulse_error);
    String(d,"node_columns","global_node,source_node_id,wall_triangle_id,force_N,potential_J,stiffness_N_m,force_world_xyz_N,wall_point_xyz_m,wall_reaction_xyz_N,wall_moment_xyz_N_m,surface_power_W,fixed,touching_or_penetrating");
    String(d,"parent_columns","weight_index,source_parent_index,source_element_id,source_part_id,source_material_id,source_section_id,feature_id,parent_face_id,family,arity,force_N,resultant_N,potential_J");
    Value nodes(rapidjson::kArrayType),parents(rapidjson::kArrayType);const auto wall=v.setup->placed_wall()->view();
    for(std::size_t n=0;n<a.node_count;++n) {
        const auto& p=a.nodes[n];Require(p.valid&&p.node==n&&p.base_epoch==c.base_epoch&&p.attempt==c.attempt&&!p.fixed&&
            p.local_velocity_first_timestep==0&&p.wall_point.x==v.setup->placed_wall()->geometry()->wall_x()&&
            p.force_world.x==-p.force.value&&p.force_world.y==0&&p.force_world.z==0&&
            std::any_of(wall.triangles,wall.triangles+wall.triangle_count,[&](const auto& f){return f.triangle_id==a.wall_face[n];}),
            "Contact node or original mesh-face mapping changed");
        auto row=Ids(d,{n,source.nodes[n].source_id,a.wall_face[n]});
        row.PushBack(Certificate(d,p.force),d.GetAllocator());row.PushBack(Certificate(d,p.potential),d.GetAllocator());
        row.PushBack(Certificate(d,p.stiffness),d.GetAllocator());row.PushBack(Vector(d,p.force_world),d.GetAllocator());
        row.PushBack(Vector(d,p.wall_point),d.GetAllocator());row.PushBack(Vector(d,p.wall_reaction),d.GetAllocator());
        row.PushBack(Vector(d,p.wall_moment),d.GetAllocator());Require(std::isfinite(p.surface_power),"Nonfinite contact power");
        row.PushBack(p.surface_power,d.GetAllocator());row.PushBack(p.fixed,d.GetAllocator());row.PushBack(p.touching_or_penetrating,d.GetAllocator());
        nodes.PushBack(row,d.GetAllocator());
    }
    for(std::size_t e=0;e<a.parent_count;++e) {
        const auto& p=a.parents[e];const auto& expected=w.parent(static_cast<unsigned>(e));
        const auto& source_parent=ContactSourceParent(*v.bindings,geometry,e);const auto index=source_parent.index;
        Require(p.valid&&p.parent_element_id==expected.parent_element_id&&p.parent_element_id==source_parent.source_id&&
            p.parent_face_id==expected.parent_face_id&&p.feature_id==expected.feature_id&&p.arity==expected.arity&&p.family==expected.family,
            "Contact parent/source/material mapping changed");
        auto row=Ids(d,{e,index,p.parent_element_id,source_parent.part_id,source_parent.material_id,source_parent.section_id,p.feature_id,p.parent_face_id});
        row.PushBack(Value(p.arity==4?"Q4_center_area":"T3_native",d.GetAllocator()),d.GetAllocator());row.PushBack(p.arity,d.GetAllocator());
        Value forces(rapidjson::kArrayType);for(unsigned local=0;local<4;++local)forces.PushBack(Certificate(d,p.force[local]),d.GetAllocator());
        row.PushBack(forces,d.GetAllocator());row.PushBack(Certificate(d,p.resultant),d.GetAllocator());row.PushBack(Certificate(d,p.potential),d.GetAllocator());
        parents.PushBack(row,d.GetAllocator());
    }
    Put(d,"nodes",std::move(nodes));Put(d,"parents",std::move(parents));return d;
}
} // namespace crash::output::assembly::wall_fields
