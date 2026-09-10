#include "SourcePartWallFields.h"
#include "SourcePartFieldPrimitives.h"
namespace crash::cases::source_part_wall {
using namespace output;
namespace {
Value Certificate(Document& d,const contact::Q4CertifiedIntegral& c) {
    Require(contact::nodal_wall_detail::Certificate(c),"Invalid owning wall setup certificate");
    const double values[]{c.value,c.lower,c.upper,c.error};return FiniteArray(d,values,4);
}
void Vector(Document& d,const char* key,contact::Vec3 x) {const double v[]{x.x,x.y,x.z};FiniteArray(d,key,v,3);}
}
Document SourcePartWallSetupFields(const SourcePartWallSetup& setup) {
    Require(setup.initialized(),"Wall setup is not prepared");const auto& s=*setup.settings();const auto& c=*setup.certificate();
    Document d;d.SetObject();String(d,"contact_model",contact::NodalWallContactModel);
    String(d,"certificate_columns","value,lower,upper,error");
    Integer(d,"configuration_id",s.configuration_id);Integer(d,"qualification_id",s.qualification_id);Integer(d,"wall_binding_id",s.wall_binding_id);
    Number(d,"declared_leading_gap_m",s.leading_gap);const double gap[]{c.leading_gap.lower,c.leading_gap.upper};FiniteArray(d,"actual_leading_gap_interval_m",gap,2);
    Number(d,"area_floor_m2",s.area_floor);Number(d,"design_penetration_m",s.design_penetration);Number(d,"penetration_cap_m",s.penetration_cap);
    Number(d,"kinetic_budget_factor",s.kinetic_budget_factor);Number(d,"kinetic_budget_upper_J",c.kinetic_budget_upper);
    Number(d,"stiffness_per_area_N_m3",c.stiffness_per_area);Number(d,"design_potential_lower_J",c.design_potential_lower);
    Number(d,"minimum_nodal_area_lower_m2",c.minimum_nodal_area_lower);Integer(d,"minimum_area_node",c.minimum_area_node);
    Integer(d,"kappa_upward_steps",c.kappa_upward_steps);Number(d,"measured_initial_kinetic_J",c.measured_initial_kinetic);
    d.AddMember("native_initial_kinetic_J",Certificate(d,c.native_initial_kinetic),d.GetAllocator());
    FiniteArray(d,"initial_velocity_xyz_m_per_s",c.initial_velocity.data(),3);Number(d,"fixed_dt_s",c.fixed_dt);
    Number(d,"motion_margin_m",s.motion_margin);Number(d,"exposed_clearance_m",s.exposed_clearance);
    Number(d,"parent_force_error_N",s.parent_force_error);Number(d,"parent_energy_error_J",s.parent_energy_error);
    Number(d,"maximum_contact_step_rate",s.maximum_step_rate);Boolean(d,"projected_motion_covered",c.coverage.covered);
    Vector(d,"projected_motion_minimum_m",c.coverage.physical.minimum);Vector(d,"projected_motion_maximum_m",c.coverage.physical.maximum);
    Vector(d,"query_minimum_m",c.coverage.query.minimum);Vector(d,"query_maximum_m",c.coverage.query.maximum);
    Vector(d,"query_lower_expansion_upper_m",c.coverage.lower_expansion_upper);Vector(d,"query_upper_expansion_upper_m",c.coverage.upper_expansion_upper);
    String(d,"wall_placement_file","placed-wall-placement.json");String(d,"wall_mesh_file","placed-wall.mesh.json");
    Value areas(rapidjson::kArrayType),parents(rapidjson::kArrayType);
    const auto& geometry=*setup.source_geometry();const auto& weights=*geometry.weights();
    for(unsigned n=0;n<source::NodeCount;++n)areas.PushBack(Certificate(d,weights.node(n).area),d.GetAllocator());
    for(unsigned p=0;p<source::ParentCount;++p) {
        const auto& w=weights.parent(p);Value item(rapidjson::kObjectType);
        elastic::field_detail::UInt(item,d,"weight_index",p);elastic::field_detail::UInt(item,d,"source_parent_index",geometry.source_parent_index(p));
        elastic::field_detail::UInt(item,d,"source_element_id",w.parent_element_id);elastic::field_detail::UInt(item,d,"arity",w.arity);
        item.AddMember("area_m2",Certificate(d,w.area),d.GetAllocator());item.AddMember("share_m2",Certificate(d,w.share),d.GetAllocator());
        parents.PushBack(item,d.GetAllocator());
    }
    d.AddMember("nodal_area_m2",areas,d.GetAllocator());d.AddMember("contact_parents",parents,d.GetAllocator());return d;
}
}
