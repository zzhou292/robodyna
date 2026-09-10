#include "SourcePartElasticFields.h"
#include "output/SurfaceBindingFields.h"
#include <cmath>

namespace crash::cases::source_part_elastic {
using namespace output;
namespace {
void CheckPhase(const Snapshot& f) {
    const auto& s=f.stamp;
    Require(s.owner_id&&s.node_count==NodeCount&&s.has_rotations&&std::isfinite(s.fixed_dt)&&s.fixed_dt>0&&
        std::isfinite(s.time)&&s.time>=0&&s.temporal_scheme==tl::fea::NodalTemporalScheme::StaggeredHalfKickStart,
        "Invalid source-part field owner or scheme");
    if(!s.epoch) {
        Require(s.time==0&&s.velocity_phase==tl::fea::NodalVelocityPhase::Collocated&&s.velocity_time==0&&
            !s.reactions_valid&&s.reaction_base_epoch==0&&s.reaction_time==0&&s.reaction_kick_dt==0,
            "Source initial field timing is not collocated startup");
        for(std::size_t n=0;n<NodeCount;++n) {
            for(unsigned j=0;j<4;++j)Require(f.orientation[4*n+j]==(j==0?1:0),"Source initial orientation is not identity");
            for(unsigned j=0;j<3;++j)Require(f.velocity[3*n+j]==0&&f.omega[3*n+j]==0&&
                f.synchronized_velocity[3*n+j]==0&&f.synchronized_omega[3*n+j]==0,"Source initial velocities are not rest");
        }
    } else Require(s.velocity_phase==tl::fea::NodalVelocityPhase::PreviousMidpoint&&s.reactions_valid&&
        s.reaction_base_epoch==s.epoch-1&&std::isfinite(s.reaction_time)&&s.reaction_time>=0&&
        Bits(s.time)==Bits(s.reaction_time+s.fixed_dt)&&Bits(s.velocity_time)==Bits(s.reaction_time+.5*s.fixed_dt)&&
        Bits(s.reaction_kick_dt)==Bits(s.epoch==1?.5*s.fixed_dt:s.fixed_dt),
        "Source field velocity or first-kick phase is invalid");
}
void UInt(Value& object, Document& doc, const char* key, std::uint64_t value) {
    object.AddMember(Value(key,doc.GetAllocator()),Value().SetUint64(value),doc.GetAllocator());
}
void Scalar(Value& object, Document& doc, const char* key, double value) {
    Require(std::isfinite(value),"Nonfinite source-part artifact scalar");
    Value name(key,doc.GetAllocator());
    object.AddMember(name,value,doc.GetAllocator());
}
template<class D> Value Family(Document& doc,const D& d) {
    Value item(rapidjson::kObjectType);
    UInt(item,doc,"owner_id",d.owner_id); UInt(item,doc,"configuration_id",d.configuration_id);
    UInt(item,doc,"qualification_id",d.qualification_id); UInt(item,doc,"epoch",d.epoch);
    UInt(item,doc,"base_epoch",d.base_epoch); UInt(item,doc,"attempt",d.attempt);
    Scalar(item,doc,"time_s",d.time); Scalar(item,doc,"base_time_s",d.base_time);
    Scalar(item,doc,"velocity_time_s",d.velocity_time); Scalar(item,doc,"base_velocity_time_s",d.base_velocity_time);
    Scalar(item,doc,"kick_dt_s",d.kick_dt);
    item.AddMember("valid",d.valid,doc.GetAllocator());
    item.AddMember("has_completed_interval",d.has_completed_interval,doc.GetAllocator());
    item.AddMember("accepted_force_assembled",d.accepted_force_assembled,doc.GetAllocator());
    item.AddMember("kinetic_available",d.kinetic_available,doc.GetAllocator());
    item.AddMember("internal_work_J",FiniteArray(doc,d.internal_work,2),doc.GetAllocator());
    item.AddMember("internal_work_increment_J",FiniteArray(doc,d.internal_work_increment,2),doc.GetAllocator());
    Scalar(item,doc,"internal_kick_work_J",d.internal_kick_work);
    Scalar(item,doc,"internal_drift_work_J",d.internal_drift_work);
    Scalar(item,doc,"minimum_area_ratio",d.minimum_area_ratio);
    Scalar(item,doc,"minimum_thickness_ratio",d.minimum_thickness_ratio);
    Scalar(item,doc,"maximum_displacement_m",d.maximum_displacement);
    Scalar(item,doc,"maximum_absolute_strain",d.maximum_absolute_strain);
    Scalar(item,doc,"maximum_thickness_curvature",d.maximum_thickness_curvature);
    Scalar(item,doc,"minimum_native_dt_s",d.minimum_native_dt);
    return item;
}
void Kinetic(Document& doc,const char* key,const tl::fea::ShellBatchKinetic& k) {
    Value item(rapidjson::kObjectType);
    Scalar(item,doc,"translation_J",k.translation); Scalar(item,doc,"rotation_total_J",k.rotation);
    Scalar(item,doc,"rotation_physical_isotropic_J",k.physical_isotropic);
    Scalar(item,doc,"rotation_added_isotropic_J",k.added_isotropic);
    doc.AddMember(Value(key,doc.GetAllocator()),item,doc.GetAllocator());
}
}
visual::Binding SourcePartSurfaceBinding(const SourcePartElasticCase& run,std::uint64_t run_id,std::uint64_t topology_id) {
    Require(run.initialized()&&run_id&&topology_id,"Source-part surface requires initialized explicit identity");
    visual::Binding result; result.identity={run.diagnostics().shells.qeph.owner_id,run_id,topology_id};
    result.tl_node_count=NodeCount;
    for(std::size_t n=0;n<NodeCount;++n)
        result.vertices.push_back({static_cast<std::uint32_t>(n),{1,1,run.source().nodes()[n].source_id}});
    for(const auto& p:run.source().parents()) {
        const auto& v=p.local_node_indices;
        result.triangles.push_back({{v[0],v[1],v[2]},1,1,p.source_id,source::PartId,0,0});
        if(p.arity==4) result.triangles.push_back({{v[0],v[2],v[3]},1,1,p.source_id,source::PartId,0,1});
    }
    return result;
}
output::Document SourcePartConfiguration(const SourcePartElasticCase& run,const visual::Binding& binding,
                                        std::uint64_t steps,unsigned frame_every) {
    const auto& c=run.config(); Document doc; doc.SetObject();
    String(doc,"schema","robo_dyna.source_part_elastic_configuration.v1");
    String(doc,"scope","Original Yaris part 2000157; experimental elastic pulse and free response");
    String(doc,"units","SI; endpoint positions and wxyz orientations; raw world velocity and omega at declared staggered time");
    String(doc,"source_readiness_sha256",source::ReadinessSha256); Integer(doc,"source_readiness_bytes",source::ReadinessBytes);
    Integer(doc,"source_part_id",source::PartId); Integer(doc,"q4_count",source::Q4Count); Integer(doc,"t3_count",source::T3Count);
    String(doc,"source_identity","Authenticated original EIDs/node IDs; asset 1 and instance 1 identify this single imported source");
    String(doc,"material_policy","Experimental LAW1 E=200 GPa nu=0.3; original density and thickness; source MAT024/ELFORM2 not reproduced");
    String(doc,"attachment_policy","Free part; six source nodal rigid groups and unresolved tied scope retained as unapplied source metadata");
    String(doc,"mass_policy","Native QEPH/T3 structural mass and total isotropic J; physical and added J archived separately without recombination");
    Number(doc,"young_modulus_Pa",200e9); Number(doc,"poisson_ratio",.3);
    Number(doc,"density_kg_m3",run.source().surface_mass().density_kg_m3);
    Number(doc,"thickness_m",run.source().surface_mass().thickness_m);
    Number(doc,"fixed_dt_s",c.dt); Number(doc,"requested_horizon_s",steps*c.dt); Integer(doc,"required_steps",steps);
    Integer(doc,"frame_every",frame_every); Integer(doc,"owner_id",binding.identity.owner);
    Integer(doc,"run_id",binding.identity.run); Integer(doc,"topology_id",binding.identity.topology);
    Integer(doc,"configuration_id",c.configuration_id); Integer(doc,"qualification_id",c.qualification_id);
    Number(doc,"pulse_duration_s",c.pulse_duration); Number(doc,"acceleration_m_s2",c.acceleration);
    Integer(doc,"spatial_axis",c.spatial_axis); FiniteArray(doc,"direction_xyz",c.direction.data(),3);
    String(doc,"pulse_law","sin(pi*t/T)^2 on [0,T]; accepted-base force = native mass * acceleration * pulse * (xi^2 - mass weighted mean), xi in [-1,1]");
    Number(doc,"maximum_displacement_m",c.maximum_displacement); Number(doc,"maximum_rotation_rad",c.maximum_rotation);
    Number(doc,"maximum_strain",c.maximum_strain); Number(doc,"maximum_thickness_curvature",c.maximum_thickness_curvature);
    Number(doc,"minimum_area_ratio",c.minimum_area_ratio); Number(doc,"maximum_area_ratio",c.maximum_area_ratio);
    Number(doc,"minimum_thickness_ratio",c.minimum_thickness_ratio); Number(doc,"maximum_thickness_ratio",c.maximum_thickness_ratio);
    Number(doc,"maximum_energy_residual_J",c.maximum_energy_residual);
    Number(doc,"relative_energy_residual",c.relative_energy_residual);
    Number(doc,"maximum_native_dt_fraction",c.maximum_native_dt_fraction);
    AppendSurfaceBinding(doc,binding);
    Value nodes(rapidjson::kArrayType),parents(rapidjson::kArrayType);
    for(std::size_t n=0;n<NodeCount;++n) {
        Value item(rapidjson::kObjectType); const auto& s=run.source().nodes()[n]; const auto& b=run.binding().nodes()[n];
        UInt(item,doc,"local_node",n); UInt(item,doc,"source_node_id",s.source_id); UInt(item,doc,"canonical_index",s.canonical_index);
        UInt(item,doc,"source_line",s.source_line); const double x[]{b.position.x,b.position.y,b.position.z};
        item.AddMember("reference_xyz_m",FiniteArray(doc,x,3),doc.GetAllocator());
        Scalar(item,doc,"mass_kg",b.native.mass); Scalar(item,doc,"isotropic_inertia_kg_m2",b.native.isotropic_inertia);
        Scalar(item,doc,"physical_inertia_kg_m2",b.native.physical_inertia); Scalar(item,doc,"added_inertia_kg_m2",b.native.added_inertia);
        nodes.PushBack(item,doc.GetAllocator());
    }
    std::size_t qi=0,ti=0;
    for(std::size_t p=0;p<source::ParentCount;++p) {
        const auto& s=run.source().parents()[p]; Value item(rapidjson::kObjectType),connectivity(rapidjson::kArrayType);
        UInt(item,doc,"source_parent_index",p); UInt(item,doc,"source_element_id",s.source_id); UInt(item,doc,"source_line",s.source_line);
        UInt(item,doc,"canonical_index",s.canonical_index); UInt(item,doc,"arity",s.arity);
        UInt(item,doc,"family_index",s.arity==4?qi++:ti++);
        item.AddMember("family",Value(s.arity==4?"QEPH":"T3",doc.GetAllocator()),doc.GetAllocator());
        for(unsigned j=0;j<s.arity;++j) connectivity.PushBack(s.local_node_indices[j],doc.GetAllocator());
        item.AddMember("local_connectivity",connectivity,doc.GetAllocator()); parents.PushBack(item,doc.GetAllocator());
    }
    doc.AddMember("reference_nodes",nodes,doc.GetAllocator()); doc.AddMember("source_parents",parents,doc.GetAllocator());
    return doc;
}
void AppendSourcePartDiagnostics(Document& doc,const Diagnostics& d) {
    Number(doc,"external_kick_work_J",d.external_kick_work); Number(doc,"external_drift_work_J",d.external_drift_work);
    Number(doc,"absolute_external_drift_work_J",d.absolute_external_drift_work);
    Number(doc,"maximum_chord_change_m",d.maximum_chord_change);
    Number(doc,"kinetic_work_residual_J",d.kinetic_work_residual); Number(doc,"kinetic_work_allowance_J",d.kinetic_work_allowance);
    Number(doc,"synchronized_kinetic_J",d.synchronized_kinetic); Number(doc,"total_internal_work_J",d.total_internal_work);
    Number(doc,"energy_residual_J",d.energy_residual); Number(doc,"maximum_relative_displacement_m",d.max_relative_displacement);
    Number(doc,"maximum_rotation_rad",d.maximum_rotation); Number(doc,"maximum_area_ratio",d.maximum_area_ratio);
    Number(doc,"maximum_thickness_ratio",d.maximum_thickness_ratio);
    Boolean(doc,"shell_diagnostics_valid",d.shells.valid);
    Kinetic(doc,"base_kinetic",d.shells.base_kinetic); Kinetic(doc,"carried_kinetic",d.shells.kinetic);
    auto q=Family(doc,d.shells.qeph),t=Family(doc,d.shells.t3);
    Scalar(q,doc,"hourglass_viscous_work_J",d.shells.qeph.hourglass_viscous_work);
    Scalar(q,doc,"hourglass_viscous_work_increment_J",d.shells.qeph.hourglass_viscous_work_increment);
    doc.AddMember("qeph",q,doc.GetAllocator()); doc.AddMember("t3",t,doc.GetAllocator());
}
Document SourcePartFrameFields(const Snapshot& f) {
    CheckPhase(f); const auto& s=f.stamp;
    Document doc; doc.SetObject(); String(doc,"schema","robo_dyna.source_part_elastic_fields.v1");
    Integer(doc,"owner_id",s.owner_id); Integer(doc,"accepted_epoch",s.epoch); Number(doc,"accepted_time_s",s.time);
    Number(doc,"fixed_dt_s",s.fixed_dt); String(doc,"temporal_scheme","staggered_half_kick_start");
    String(doc,"velocity_phase",s.velocity_phase==tl::fea::NodalVelocityPhase::Collocated?"collocated":"previous_midpoint");
    Number(doc,"velocity_time_s",s.velocity_time); Boolean(doc,"reactions_valid",s.reactions_valid);
    Integer(doc,"reaction_base_epoch",s.reaction_base_epoch); Number(doc,"reaction_time_s",s.reaction_time);
    Number(doc,"reaction_kick_dt_s",s.reaction_kick_dt);
    FiniteArray(doc,"position_xyz_m",f.position.data(),f.position.size());
    FiniteArray(doc,"orientation_wxyz",f.orientation.data(),f.orientation.size());
    FiniteArray(doc,"velocity_xyz_m_per_s",f.velocity.data(),f.velocity.size());
    FiniteArray(doc,"omega_world_xyz_rad_per_s",f.omega.data(),f.omega.size());
    FiniteArray(doc,"synchronized_velocity_xyz_m_per_s",f.synchronized_velocity.data(),f.synchronized_velocity.size());
    FiniteArray(doc,"synchronized_omega_world_xyz_rad_per_s",f.synchronized_omega.data(),f.synchronized_omega.size());
    String(doc,"synchronized_fields_policy","Derived endpoint velocity from raw previous midpoint plus complete endpoint internal and applied pulse RHS half kick; never the raw owner field");
    AppendSourcePartDiagnostics(doc,f.diagnostics); return doc;
}
} // namespace crash::cases::source_part_elastic
