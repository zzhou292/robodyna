#include "ElasticCouponFields.h"
#include "output/ArtifactIO.h"
#include <cmath>

namespace crash::case_data {
namespace {
using namespace crash::output;
using Document=rapidjson::Document;
using Value=rapidjson::Value;
Value Array(Document& doc,const double* data,std::size_t count) {
    Value values(rapidjson::kArrayType);
    for(std::size_t i=0;i<count;++i) { Require(std::isfinite(data[i]),"Nonfinite coupon output field"); values.PushBack(data[i],doc.GetAllocator()); }
    return values;
}
void Array(Document& doc,const char* name,const double* data,std::size_t count) {
    Value key(name,doc.GetAllocator()); doc.AddMember(key,Array(doc,data,count),doc.GetAllocator());
}
}  // namespace

Document CouponFrameFields(const ElasticCouponFrame& f) {
    const auto& a=f.element_association;
    using Phase=tl::fea::reissner::ShellBatchPhase;
    Require(f.stamp.owner_id&&a.valid&&a.owner_id==f.stamp.owner_id&&a.configuration_id&&
            a.configuration_id==f.metrics.diagnostics.configuration_id&&a.attempt&&
            ((a.phase==Phase::kAcceptedBase&&a.base_epoch==f.stamp.epoch)||
             (a.phase==Phase::kPreparedCandidate&&f.stamp.epoch>0&&a.base_epoch==f.stamp.epoch-1)),
            "Coupon element fields do not identify the accepted configuration");
    Document doc; doc.SetObject();
    String(doc,"schema","robo_dyna.elastic_coupon_fields.v1");
    Integer(doc,"owner_id",f.stamp.owner_id); Integer(doc,"accepted_epoch",f.stamp.epoch);
    Number(doc,"accepted_time_s",f.stamp.time); Number(doc,"fixed_dt_s",f.stamp.fixed_dt);
    Boolean(doc,"reactions_valid",f.stamp.reactions_valid);
    Integer(doc,"reaction_base_epoch",f.stamp.reaction_base_epoch); Number(doc,"reaction_time_s",f.stamp.reaction_time);
    Array(doc,"position_xyz_m",f.position.data(),f.position.size());
    Array(doc,"orientation_wxyz",f.rotation.data(),f.rotation.size());
    Array(doc,"velocity_xyz_m_per_s",f.velocity.data(),f.velocity.size());
    Array(doc,"omega_world_xyz_rad_per_s",f.omega.data(),f.omega.size());
    Array(doc,"reaction_force_xyz_N_at_base",f.reaction_force.data(),f.reaction_force.size());
    Array(doc,"reaction_couple_world_xyz_Nm_at_base",f.reaction_couple.data(),f.reaction_couple.size());
    const auto& association=f.element_association;
    Integer(doc,"element_evaluation_base_epoch",association.base_epoch);
    Integer(doc,"element_evaluation_attempt",association.attempt);
    Integer(doc,"qualification_id",association.configuration_id);
    String(doc,"element_evaluation_phase",association.phase==tl::fea::reissner::ShellBatchPhase::kAcceptedBase?
           "accepted_base":"prepared_candidate_subsequently_committed");
    String(doc,"element_order","Synthetic parent elements 1,2; four Gauss points in immutable setup order");
    String(doc,"gauss_row_order","eps1.x,eps1.y,eps1.z,eps2.x,eps2.y,eps2.z,k1.x,k1.y,k1.z,k2.x,k2.y,k2.z");
    String(doc,"gauss_basis","Corotated material frame at each Gauss point; transverse rows eps1.z/eps2.z use ANS");
    String(doc,"gauss_strain_units","Rows 0-5 dimensionless; rows 6-11 1/m");
    String(doc,"gauss_resultant_units","Rows 0-5 N/m; rows 6-11 N (bending/twisting couple per width); these are section resultants, not Cauchy stress");
    Value elements(rapidjson::kArrayType);
    for(unsigned e=0;e<f.element.size();++e) {
        const auto& source=f.element[e]; Value element(rapidjson::kObjectType);
        Require(std::isfinite(source.energy)&&std::isfinite(source.bending_energy),"Nonfinite coupon element energy");
        element.AddMember("parent_element",e+1,doc.GetAllocator());
        element.AddMember("elastic_energy_J",source.energy,doc.GetAllocator());
        element.AddMember("bending_energy_J",source.bending_energy,doc.GetAllocator());
        Value forces(rapidjson::kArrayType),couples(rapidjson::kArrayType);
        for(unsigned n=0;n<4;++n) {
            const double force[]{source.force[n].x,source.force[n].y,source.force[n].z};
            const double couple[]{source.couple[n].x,source.couple[n].y,source.couple[n].z};
            forces.PushBack(Array(doc,force,3),doc.GetAllocator()); couples.PushBack(Array(doc,couple,3),doc.GetAllocator());
        }
        element.AddMember("force_world_N",forces,doc.GetAllocator()); element.AddMember("couple_world_Nm",couples,doc.GetAllocator());
        Value strain(rapidjson::kArrayType),resultant(rapidjson::kArrayType);
        for(unsigned p=0;p<4;++p) {
            strain.PushBack(Array(doc,source.strain[p],12),doc.GetAllocator());
            resultant.PushBack(Array(doc,source.resultant[p],12),doc.GetAllocator());
        }
        element.AddMember("gauss_strain",strain,doc.GetAllocator()); element.AddMember("gauss_resultant",resultant,doc.GetAllocator());
        elements.PushBack(element,doc.GetAllocator());
    }
    doc.AddMember("elements",elements,doc.GetAllocator()); return doc;
}

Document CouponConfiguration(const ElasticCouponCase& run,unsigned frame_every) {
    Require(run.metrics()&&run.modal()&&run.output(),"Coupon output requires an initialized run");
    const auto& m=*run.modal(); Document doc; doc.SetObject();
    String(doc,"schema","robo_dyna.elastic_coupon_configuration.v1");
    String(doc,"scope","Synthetic two-Q4 clamped elastic release; no contact, plasticity or vehicle model");
    String(doc,"units","SI; orientations wxyz; couples and angular velocities world-frame");
    String(doc,"inertia_policy","Physical rho*t^3*A/12 tangential inertia; equal numerical drilling gives total J*I");
    using D=reference::ElasticCouponData;
    Number(doc,"length_m",D::length); Number(doc,"width_m",D::width); Number(doc,"thickness_m",D::thickness);
    Number(doc,"young_modulus_Pa",D::young_modulus); Number(doc,"poisson_ratio",D::poisson_ratio); Number(doc,"density_kg_per_m3",D::density);
    Number(doc,"initial_tip_displacement_m",D::initial_tip_displacement);
    Number(doc,"fixed_dt_s",run.metrics()->stamp.fixed_dt); Number(doc,"half_period_horizon_s",m.horizon);
    Integer(doc,"required_steps",run.metrics()->required_steps); Integer(doc,"frame_every",frame_every);
    Number(doc,"modal_angular_frequency_rad_per_s",m.first_mode_angular_frequency);
    Number(doc,"initial_energy_J",run.metrics()->initial_energy);
    Number(doc,"monitored_norm_limit_per_s2",m.monitored_norm_limit);
    Number(doc,"spectral_step_limit_s",m.spectral_step_limit); Number(doc,"wave_step_limit_s",m.wave_step_limit);
    Number(doc,"rotary_step_limit_s",m.rotary_step_limit); Number(doc,"proposed_step_limit_s",m.proposed_step_limit);
    Number(doc,"reference_symmetry_error",m.reference_symmetry_error);
    Number(doc,"derivative_refinement_error",m.mass_scaled_derivative_refinement_error);
    Number(doc,"TL_derivative_error",m.tl_derivative_relative_error); Number(doc,"TL_directional_error",m.tl_directional_relative_error);
    Array(doc,"squared_frequency_per_s2",m.squared_frequency.data(),m.squared_frequency.size());
    Array(doc,"sampled_operator_norm_per_s2",m.sampled_operator_norm.data(),m.sampled_operator_norm.size());
    Array(doc,"initial_mode_increment_m_rad",m.initial_mode_increment.data(),m.initial_mode_increment.size());
    const auto& model=*run.model_data();
    Value nodes(rapidjson::kArrayType),sections(rapidjson::kArrayType);
    for(unsigned n=0;n<reference::kCouponNodes;++n) {
        Value node(rapidjson::kObjectType);
        const auto& mass=model.nodal_mass[n];
        node.AddMember("mass_kg",mass.mass,doc.GetAllocator());
        node.AddMember("physical_tangential_inertia_kg_m2",mass.physical_tangential_inertia,doc.GetAllocator());
        node.AddMember("artificial_drilling_inertia_kg_m2",mass.artificial_drilling_inertia,doc.GetAllocator());
        node.AddMember("clamped",model.fixed[n],doc.GetAllocator());
        const auto& p=model.reference_configuration.position[n]; const double position[]{p.x,p.y,p.z};
        const auto& q=model.reference_configuration.rotation[n]; const double rotation[]{q.w,q.x,q.y,q.z};
        node.AddMember("reference_position_m",Array(doc,position,3),doc.GetAllocator());
        node.AddMember("reference_orientation_wxyz",Array(doc,rotation,4),doc.GetAllocator());
        nodes.PushBack(node,doc.GetAllocator());
    }
    for(unsigned e=0;e<reference::kCouponElements;++e) {
        Value element(rapidjson::kObjectType),connectivity(rapidjson::kArrayType);
        for(auto node:model.connectivity[e])connectivity.PushBack(Value().SetUint64(node),doc.GetAllocator());
        element.AddMember("connectivity_zero_based",connectivity,doc.GetAllocator());
        element.AddMember("section_stiffness_12x12_row_major",Array(doc,model.section[e].stiffness,144),doc.GetAllocator());
        sections.PushBack(element,doc.GetAllocator());
    }
    doc.AddMember("reference_nodes",nodes,doc.GetAllocator()); doc.AddMember("reference_elements",sections,doc.GetAllocator());
    Number(doc,"maximum_displacement_m",ElasticCouponLimits::displacement);
    Number(doc,"maximum_relative_energy_error",ElasticCouponLimits::energy_fraction);
    Number(doc,"maximum_director_departure_rad",ElasticCouponLimits::director_departure);
    Number(doc,"maximum_pair_angle_rad",ElasticCouponLimits::pair_angle);
    Number(doc,"maximum_step_rotation_rad",ElasticCouponLimits::rotation_increment);
    Number(doc,"maximum_membrane_shear_strain",ElasticCouponLimits::strain);
    Number(doc,"maximum_thickness_curvature",ElasticCouponLimits::thickness_curvature);
    Number(doc,"minimum_area_ratio",ElasticCouponLimits::minimum_area_ratio); Number(doc,"maximum_area_ratio",ElasticCouponLimits::maximum_area_ratio);
    String(doc,"admission_scope","Sampled spectral envelope plus candidate geometry, energy and work gates; not a general nonlinear stability proof");
    String(doc,"field_phase","Per-frame element fields at accepted geometry; clamp reactions at preceding base state; intervals give base force work and accepted energy");
    const auto& b=*run.output()->surface().binding();
    Integer(doc,"owner_id",b.identity.owner); Integer(doc,"run_id",b.identity.run); Integer(doc,"topology_id",b.identity.topology);
    String(doc,"source_identity","Synthetic fixture asset/instance 1; not Yaris deck identities");
    Value vertices(rapidjson::kArrayType),triangles(rapidjson::kArrayType);
    for(const auto& v:b.vertices) {
        Value item(rapidjson::kArrayType);
        for(std::uint64_t n:{std::uint64_t(v.tl_node),v.source.asset,v.source.instance,v.source.node})item.PushBack(Value().SetUint64(n),doc.GetAllocator());
        vertices.PushBack(item,doc.GetAllocator());
    }
    for(const auto& t:b.triangles) {
        Value item(rapidjson::kArrayType);
        for(std::uint64_t n:{std::uint64_t(t.vertices[0]),std::uint64_t(t.vertices[1]),std::uint64_t(t.vertices[2]),
                            t.asset,t.instance,t.element,t.part,std::uint64_t(t.local_face),std::uint64_t(t.subtriangle)})
            item.PushBack(Value().SetUint64(n),doc.GetAllocator());
        triangles.PushBack(item,doc.GetAllocator());
    }
    String(doc,"vertex_binding_columns","tl_node,asset,instance,source_node");
    String(doc,"triangle_binding_columns","v0,v1,v2,asset,instance,parent_element,part,local_face,subtriangle");
    doc.AddMember("vertex_binding",vertices,doc.GetAllocator()); doc.AddMember("triangle_binding",triangles,doc.GetAllocator());
    return doc;
}
}  // namespace crash::case_data
