#include "SourcePartElasticFields.h"
#include "SourcePartCommonFields.h"
#include "SourcePartFieldPrimitives.h"
#include "output/SurfaceBindingFields.h"
#include <cmath>

namespace crash::cases::source_part_elastic {
using namespace output;
namespace {
using field_detail::UInt;
using field_detail::Scalar;
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
    AppendSourcePartInputTables(doc,run);
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
    CheckSourcePartPhase(f,{});
    Document doc; doc.SetObject(); String(doc,"schema","robo_dyna.source_part_elastic_fields.v1");
    AppendSourcePartKinematics(doc,f);
    String(doc,"synchronized_fields_policy","Derived endpoint velocity from raw previous midpoint plus complete endpoint internal and applied pulse RHS half kick; never the raw owner field");
    AppendSourcePartDiagnostics(doc,f.diagnostics); return doc;
}
} // namespace crash::cases::source_part_elastic
