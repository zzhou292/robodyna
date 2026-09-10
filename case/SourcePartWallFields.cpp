#include "SourcePartWallFields.h"
#include "SourcePartCommonFields.h"
#include "output/SurfaceBindingFields.h"
#include <cmath>

namespace crash::cases::source_part_wall {
using namespace output;
namespace {
void Object(Document& doc,const char* key,const Document& child) {
    Value value;value.CopyFrom(child,doc.GetAllocator());doc.AddMember(Value(key,doc.GetAllocator()),value,doc.GetAllocator());
}
}
Document SourcePartWallConfiguration(const elastic::SourcePartElasticCase& run,const visual::Binding& binding,
    std::uint64_t steps,unsigned every) {
    Require(run.initialized()&&run.config().experiment==elastic::Experiment::MeshWallImpact&&run.wall_setup(),
            "Wall fields require the initialized mesh-wall experiment");
    const auto& c=run.config();Document d;d.SetObject();
    String(d,"schema","robo_dyna.source_part_wall_configuration.v1");
    String(d,"scope","Original Yaris part 2000157; experimental elastic QEPH/T3 impact against the placed original finite mesh wall");
    String(d,"units","SI; endpoint x/q, raw world v/omega at declared midpoint, separately derived endpoint v/omega");
    String(d,"source_readiness_sha256",source::ReadinessSha256);Integer(d,"source_readiness_bytes",source::ReadinessBytes);
    Integer(d,"source_part_id",source::PartId);Integer(d,"q4_count",source::Q4Count);Integer(d,"t3_count",source::T3Count);
    String(d,"material_policy","Experimental LAW1 E=200 GPa nu=0.3; original density and thickness; source MAT024/ELFORM2 not reproduced");
    String(d,"attachment_policy","Free part; source rigid groups and tied scope remain unapplied metadata");
    String(d,"mass_policy","Actual native QEPH/T3 structural mass and total isotropic J; no contact-area proxy mass");
    String(d,"loading_policy","Uniform physical initial velocity; zero initial spin; no external pulse or gravity");
    Number(d,"young_modulus_Pa",200e9);Number(d,"poisson_ratio",.3);
    Number(d,"density_kg_m3",run.source().surface_mass().density_kg_m3);Number(d,"thickness_m",run.source().surface_mass().thickness_m);
    Number(d,"fixed_dt_s",c.dt);Number(d,"requested_horizon_s",steps*c.dt);Integer(d,"required_steps",steps);Integer(d,"frame_every",every);
    Integer(d,"owner_id",binding.identity.owner);Integer(d,"run_id",binding.identity.run);Integer(d,"topology_id",binding.identity.topology);
    Integer(d,"configuration_id",c.configuration_id);Integer(d,"qualification_id",c.qualification_id);
    FiniteArray(d,"initial_velocity_xyz_m_per_s",c.initial_velocity.data(),3);Number(d,"initial_kinetic_J",run.initial_kinetic_energy());
    Number(d,"maximum_displacement_m",c.maximum_displacement);Number(d,"maximum_rotation_rad",c.maximum_rotation);
    Number(d,"maximum_strain",c.maximum_strain);Number(d,"maximum_thickness_curvature",c.maximum_thickness_curvature);
    Number(d,"minimum_area_ratio",c.minimum_area_ratio);Number(d,"maximum_area_ratio",c.maximum_area_ratio);
    Number(d,"minimum_thickness_ratio",c.minimum_thickness_ratio);Number(d,"maximum_thickness_ratio",c.maximum_thickness_ratio);
    Number(d,"maximum_energy_residual_J",c.maximum_energy_residual);Number(d,"relative_energy_residual",c.relative_energy_residual);
    Number(d,"maximum_native_dt_fraction",c.maximum_native_dt_fraction);
    String(d,"energy_policy","K synchronized + native internal and viscous work + contact potential - measured initial K0; contact work is not subtracted twice");
    String(d,"contact_result_policy","Epoch zero has certified separation only; later contact results retain their prepared-candidate provenance beside the committed owner stamp");
    AppendSurfaceBinding(d,binding);elastic::AppendSourcePartInputTables(d,run);
    Object(d,"wall_setup",SourcePartWallSetupFields(*run.wall_setup()));return d;
}
void CheckAcceptedWallContact(const tl::fea::NodalStamp& s,const elastic::Diagnostics& d,const contact::NodalWallDeviceResults* r) {
    Require(s.owner_id&&d.shells.valid,"Missing accepted source wall identity");
    if(!s.epoch) {Require(!r,"Initial wall fields cannot fabricate a prepared candidate");return;}
    Require(r&&r->diagnostics.valid,"Accepted wall interval has no copied contact result");
    const auto& c=r->diagnostics;const auto& q=d.shells.qeph;const auto& t=d.shells.t3;
    Require(c.phase==contact::NodalWallDevicePhase::PreparedCandidate&&c.owner_id==s.owner_id&&
        c.scheme==tl::fea::NodalTemporalScheme::StaggeredHalfKickStart&&c.velocity_phase==tl::fea::NodalVelocityPhase::PreviousMidpoint&&
        c.base_epoch+1==s.epoch&&c.attempt&&c.attempt==q.attempt&&c.attempt==t.attempt&&
        c.configuration_id==q.configuration_id&&c.configuration_id==t.configuration_id&&
        c.qualification_id==q.qualification_id&&c.qualification_id==t.qualification_id&&
        q.epoch==s.epoch&&t.epoch==s.epoch&&q.has_completed_interval&&t.has_completed_interval&&
        q.accepted_force_assembled&&t.accepted_force_assembled&&Bits(c.time)==Bits(s.time)&&
        Bits(c.velocity_time)==Bits(s.velocity_time)&&Bits(c.base_time)==Bits(s.reaction_time)&&
        Bits(c.kick_dt)==Bits(s.reaction_kick_dt)&&c.node_count==source::NodeCount&&c.parent_count==source::ParentCount,
        "Contact copy is not associated with this jointly committed source interval");
}
void AppendSourcePartWallMetrics(Document& d,const elastic::WallMetrics& m) {
    Number(d,"cumulative_wall_kick_impulse_N_s",m.wall_kick_impulse);Number(d,"cumulative_wall_kick_impulse_error_N_s",m.wall_kick_impulse_error);
    FiniteArray(d,"carried_momentum_residual_xyz_kg_m_per_s",m.carried_momentum_residual.data(),3);
    FiniteArray(d,"carried_momentum_allowance_xyz_kg_m_per_s",m.carried_momentum_allowance.data(),3);
    FiniteArray(d,"carried_angular_momentum_xyz_kg_m2_per_s",m.carried_angular_momentum.data(),3);
    FiniteArray(d,"cumulative_wall_kick_moment_xyz_N_m_s",m.wall_kick_moment.data(),3);
    FiniteArray(d,"cumulative_wall_kick_moment_error_xyz_N_m_s",m.wall_kick_moment_error.data(),3);
    String(d,"angular_momentum_timing","Accepted endpoint x with raw carried v/omega at velocity_time_s; reporting only, no angular admission gate");
    Number(d,"synchronized_kinetic_uncertainty_J",m.synchronized_kinetic_uncertainty);
    Number(d,"physical_energy_uncertainty_J",m.physical_energy_uncertainty);Number(d,"energy_allowance_J",m.energy_allowance);
    Integer(d,"first_contact_epoch",m.first_contact_epoch);Integer(d,"last_contact_epoch",m.last_contact_epoch);
    Integer(d,"contact_intervals",m.contact_intervals);Integer(d,"active_nodes",m.active_nodes);
}
unsigned StrictlySeparatedNodes(const contact::NodalWallDeviceResults* result) {
    if(!result)return source::NodeCount; // The caller has checked certified startup separation.
    Require(result->diagnostics.node_count==source::NodeCount,"Strict separation requires the complete source node union");
    unsigned separated=0;for(unsigned n=0;n<result->diagnostics.node_count;++n) {
        const auto& node=result->nodes[n];separated+=!node.touching_or_penetrating&&node.force.upper==0&&node.potential.upper==0;
    }
    return separated;
}
Document SourcePartWallFrameFields(const elastic::Snapshot& f,const SourcePartWallSetup& setup,
    const contact::NodalWallDeviceResults* contact_result,const elastic::WallMetrics& metrics) {
    Require(setup.initialized()&&setup.owner_stamp()->owner_id==f.stamp.owner_id&&
        Bits(setup.owner_stamp()->fixed_dt)==Bits(f.stamp.fixed_dt),"Wall frame requires its original immutable owner setup");
    elastic::CheckSourcePartPhase(f,setup.settings()->initial_velocity);CheckAcceptedWallContact(f.stamp,f.diagnostics,contact_result);
    Document d;d.SetObject();String(d,"schema","robo_dyna.source_part_wall_fields.v1");elastic::AppendSourcePartKinematics(d,f);
    String(d,"synchronized_fields_policy","Derived endpoint velocity from raw previous midpoint plus complete native internal and contact endpoint RHS half kick; raw owner fields are unchanged");
    elastic::AppendSourcePartDiagnostics(d,f.diagnostics);AppendSourcePartWallMetrics(d,metrics);
    Integer(d,"strictly_separated_nodes",StrictlySeparatedNodes(contact_result));
    String(d,"contact_state",contact_result?"committed_interval_candidate":"certified_separated_startup");
    if(contact_result)Object(d,"contact",SourcePartWallContactFields(*contact_result));
    else d.AddMember("contact",Value(rapidjson::kNullType),d.GetAllocator());
    return d;
}
}
