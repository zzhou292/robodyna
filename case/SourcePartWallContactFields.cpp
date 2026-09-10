#include "SourcePartWallFields.h"
#include "SourcePartFieldPrimitives.h"
namespace crash::cases::source_part_wall {
using namespace output;
namespace {
Value Certificate(Document& d,const contact::Q4CertifiedIntegral& c) {
    Require(contact::nodal_wall_detail::Certificate(c),"Invalid accepted contact certificate");
    const double values[]{c.value,c.lower,c.upper,c.error};return FiniteArray(d,values,4);
}
void Vector(Document& d,const char* key,contact::Vec3 x) {const double v[]{x.x,x.y,x.z};FiniteArray(d,key,v,3);}
void Vector(Value& item,Document& d,const char* key,contact::Vec3 x) {
    const double v[]{x.x,x.y,x.z};item.AddMember(Value(key,d.GetAllocator()),FiniteArray(d,v,3),d.GetAllocator());
}
}
Document SourcePartWallContactFields(const contact::NodalWallDeviceResults& result) {
    const auto& c=result.diagnostics;Require(c.valid&&c.phase==contact::NodalWallDevicePhase::PreparedCandidate&&
        c.scheme==tl::fea::NodalTemporalScheme::StaggeredHalfKickStart&&c.velocity_phase==tl::fea::NodalVelocityPhase::PreviousMidpoint&&
        c.node_count==source::NodeCount&&c.parent_count==source::ParentCount,"Invalid copied source contact result");
    Document d;d.SetObject();String(d,"phase","prepared_candidate_of_committed_interval");
    String(d,"certificate_columns","value,lower,upper,error");
    Integer(d,"owner_id",c.owner_id);Integer(d,"configuration_id",c.configuration_id);Integer(d,"qualification_id",c.qualification_id);
    Integer(d,"wall_binding_id",c.wall_binding_id);Integer(d,"base_epoch",c.base_epoch);Integer(d,"attempt",c.attempt);
    String(d,"temporal_scheme","staggered_half_kick_start");String(d,"velocity_phase","previous_midpoint");
    Number(d,"time_s",c.time);Number(d,"velocity_time_s",c.velocity_time);Number(d,"base_time_s",c.base_time);
    Number(d,"base_velocity_time_s",c.base_velocity_time);Number(d,"kick_dt_s",c.kick_dt);
    d.AddMember("resultant_N",Certificate(d,c.resultant),d.GetAllocator());d.AddMember("potential_J",Certificate(d,c.potential),d.GetAllocator());
    Vector(d,"wall_reaction_xyz_N",c.wall_reaction);Vector(d,"wall_moment_xyz_N_m",c.wall_moment);
    Number(d,"surface_power_W",c.surface_power);Number(d,"maximum_penetration_m",c.maximum_penetration);
    Number(d,"stiffness_rate_bound_s2",c.stiffness_rate_bound);Number(d,"base_potential_J",c.base_potential);
    Number(d,"base_potential_error_J",c.base_potential_error);Number(d,"potential_increment_J",c.potential_increment);
    Number(d,"kick_work_J",c.kick_work);Number(d,"kick_work_roundoff_J",c.kick_work_roundoff);
    Number(d,"drift_work_J",c.drift_work);Number(d,"drift_work_roundoff_J",c.drift_work_roundoff);
    Number(d,"conservative_defect_J",c.conservative_defect);Number(d,"work_uncertainty_J",c.work_uncertainty);
    Number(d,"quadratic_work_upper_J",c.quadratic_work_upper);Number(d,"wall_kick_impulse_N_s",c.wall_kick_impulse);
    Number(d,"wall_kick_impulse_error_N_s",c.wall_kick_impulse_error);
    Vector(d,"wall_kick_moment_xyz_N_m_s",c.wall_kick_moment);Vector(d,"wall_kick_moment_error_xyz_N_m_s",c.wall_kick_moment_error);
    Integer(d,"node_count",c.node_count);Integer(d,"parent_count",c.parent_count);
    Value nodes(rapidjson::kArrayType),parents(rapidjson::kArrayType);
    for(unsigned n=0;n<c.node_count;++n) {
        const auto& a=result.nodes[n];Require(a.valid&&a.node==n&&a.base_epoch==c.base_epoch&&a.attempt==c.attempt,
            "Contact node identity differs from the copied interval");Value item(rapidjson::kObjectType);
        elastic::field_detail::UInt(item,d,"node",n);elastic::field_detail::UInt(item,d,"wall_face",result.wall_face[n]);
        item.AddMember("force_N",Certificate(d,a.force),d.GetAllocator());item.AddMember("potential_J",Certificate(d,a.potential),d.GetAllocator());
        item.AddMember("stiffness_N_m",Certificate(d,a.stiffness),d.GetAllocator());
        Vector(item,d,"force_world_xyz_N",a.force_world);Vector(item,d,"wall_point_xyz_m",a.wall_point);
        Vector(item,d,"wall_reaction_xyz_N",a.wall_reaction);Vector(item,d,"wall_moment_xyz_N_m",a.wall_moment);
        elastic::field_detail::Scalar(item,d,"surface_power_W",a.surface_power);
        item.AddMember("fixed",a.fixed,d.GetAllocator());item.AddMember("touching_or_penetrating",a.touching_or_penetrating,d.GetAllocator());
        nodes.PushBack(item,d.GetAllocator());
    }
    for(unsigned p=0;p<c.parent_count;++p) {
        const auto& a=result.parents[p];Require(a.valid,"Contact parent is not valid");Value item(rapidjson::kObjectType),force(rapidjson::kArrayType);
        elastic::field_detail::UInt(item,d,"source_element_id",a.parent_element_id);elastic::field_detail::UInt(item,d,"feature_id",a.feature_id);
        elastic::field_detail::UInt(item,d,"parent_face_id",a.parent_face_id);elastic::field_detail::UInt(item,d,"arity",a.arity);
        item.AddMember("family",Value(a.family==contact::NodalWallParentFamily::Q4CenterArea?"Q4_center_area":"T3_native",d.GetAllocator()),d.GetAllocator());
        for(unsigned j=0;j<4;++j)force.PushBack(Certificate(d,a.force[j]),d.GetAllocator());
        item.AddMember("force_N",force,d.GetAllocator());item.AddMember("resultant_N",Certificate(d,a.resultant),d.GetAllocator());
        item.AddMember("potential_J",Certificate(d,a.potential),d.GetAllocator());parents.PushBack(item,d.GetAllocator());
    }
    d.AddMember("nodes",nodes,d.GetAllocator());d.AddMember("parents",parents,d.GetAllocator());return d;
}
}
