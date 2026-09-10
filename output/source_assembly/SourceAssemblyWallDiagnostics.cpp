#include "SourceAssemblyWallFields.h"
#include "SourceAssemblyWallKineticChannels.h"
#include "case/source_assembly_dynamics/NativeRotation.h"
#include <cmath>

namespace crash::output::assembly::wall_fields {
namespace {
using cases::source_assembly_observation::KineticSummary;
Document Phase(const tl::fea::rigid::ObservationPhase& p) {
    Document d;d.SetObject();String(d,"kind",p.kind==tl::fea::rigid::ObservationPhaseKind::PhysicalInitialization?
        "physical_initialization":p.kind==tl::fea::rigid::ObservationPhaseKind::StoredMidpointWithLaggedFrame?
        "stored_midpoint_with_lagged_frame":"unspecified");
    Number(d,"position_time_s",p.position_time);Number(d,"velocity_time_s",p.velocity_time);Number(d,"frame_time_s",p.frame_time);return d;
}
Document Kinetic(const KineticSummary& k) {
    Document d;d.SetObject();Child(d,"phase",Phase(k.phase));
    KineticChannels(d,k.ordinary,k.grouped_members,k.groups,k.native_total,k.effective_total);
    Number(d,"publication_residual_J",k.publication_residual);Number(d,"publication_roundoff_budget_J",k.publication_roundoff_budget);return d;
}
Value Work(Document& d,const tl::fea::rigid::KickWorkChannels& w) {return Values(d,{w.translation,w.rotation,w.total});}
Value NativeKinetic(Document& d,const tl::fea::ShellBatchKinetic& k) {
    return Values(d,{k.translation,k.rotation,k.physical_isotropic,k.added_isotropic});
}
template<class D> Document Family(const D& a) {
    Document d;d.SetObject();String(d,"phase","accepted");String(d,"usage","coupled_forces");
    Integer(d,"owner_id",a.owner_id);Integer(d,"configuration_id",a.configuration_id);Integer(d,"qualification_id",a.qualification_id);
    Integer(d,"epoch",a.epoch);Integer(d,"base_epoch",a.base_epoch);Integer(d,"attempt",a.attempt);
    Number(d,"time_s",a.time);Number(d,"base_time_s",a.base_time);Number(d,"velocity_time_s",a.velocity_time);
    Number(d,"base_velocity_time_s",a.base_velocity_time);Number(d,"kick_dt_s",a.kick_dt);
    Boolean(d,"valid",a.valid);Boolean(d,"has_completed_interval",a.has_completed_interval);
    Boolean(d,"accepted_force_assembled",a.accepted_force_assembled);Boolean(d,"kinetic_available",a.kinetic_available);
    FiniteArray(d,"native_internal_work_J",a.internal_work,2);FiniteArray(d,"native_internal_work_increment_J",a.internal_work_increment,2);
    Number(d,"minimum_area_ratio",a.minimum_area_ratio);Number(d,"minimum_thickness_ratio",a.minimum_thickness_ratio);
    Number(d,"maximum_displacement_m",a.maximum_displacement);Number(d,"maximum_absolute_strain",a.maximum_absolute_strain);
    Number(d,"maximum_thickness_curvature",a.maximum_thickness_curvature);Number(d,"minimum_native_dt_s",a.minimum_native_dt);
    Number(d,"internal_kick_work_J",a.internal_kick_work);Number(d,"internal_drift_work_J",a.internal_drift_work);return d;
}
}
Document DiagnosticsDocument(const dynamics::Diagnostics& a) {
    Require(a.rotation_domain==dynamics::RotationDomain::NodalQuaternion||a.rotation_domain==dynamics::RotationDomain::NativeShellGeometryV1,
        "Unknown diagnostic rotation domain");
    Document d;d.SetObject();Boolean(d,"has_interval",a.has_interval);
    if(a.rotation_domain==dynamics::RotationDomain::NativeShellGeometryV1) {
        for(double v:{a.maximum_native_frame_rotation,a.maximum_native_normal_rotation,a.maximum_rigid_member_rotation})
            Require(std::isfinite(v)&&v>=0&&v<=dynamics::NativeRotationQualifiedBound,"Invalid native rotation diagnostic");
        Document measured;measured.SetObject();
        Number(measured,"maximum_frame_rotation_rad",a.maximum_native_frame_rotation);
        Number(measured,"maximum_nodal_normal_rotation_rad",a.maximum_native_normal_rotation);
        Number(measured,"maximum_rigid_member_rotation_rad",a.maximum_rigid_member_rotation);Child(d,"native_rotation_domain",measured);
    }
    Number(d,"native_internal_work_J",a.native_internal_work);Number(d,"maximum_rotation_rad",a.maximum_rotation);
    Number(d,"maximum_area_ratio",a.maximum_area_ratio);Number(d,"maximum_thickness_ratio",a.maximum_thickness_ratio);
    Number(d,"maximum_plastic_strain",a.maximum_plastic_strain);Number(d,"cumulative_plastic_work_J",a.cumulative_plastic_work);
    Integer(d,"yielded_points",a.yielded_points);Integer(d,"yielded_parents",a.yielded_parents);Integer(d,"active_contact_nodes",a.active_contact_nodes);
    Integer(d,"first_contact_epoch",a.first_contact_epoch);Integer(d,"last_contact_epoch",a.last_contact_epoch);Integer(d,"contact_intervals",a.contact_intervals);
    Document shells;shells.SetObject();String(shells,"native_kinetic_columns","translation_J,rotation_J,physical_isotropic_J,added_isotropic_J");
    Put(shells,"base_native_kinetic_J",NativeKinetic(shells,a.shells.base_kinetic));Put(shells,"native_kinetic_J",NativeKinetic(shells,a.shells.kinetic));
    auto q=Family(a.shells.qeph);Number(q,"hourglass_viscous_work_J",a.shells.qeph.hourglass_viscous_work);
    Number(q,"hourglass_viscous_work_increment_J",a.shells.qeph.hourglass_viscous_work_increment);
    Child(shells,"qeph",q);Child(shells,"t3",Family(a.shells.t3));Child(d,"shells",shells);
    Document m;m.SetObject();if(a.has_interval)Child(m,"before",Kinetic(a.motion.before));else Put(m,"before",Value());
    Child(m,"after",Kinetic(a.motion.after));String(m,"kick_work_columns","translation_J,rotation_J,total_J");
    Put(m,"applied_kick_work_J",Work(m,a.motion.applied));Put(m,"reaction_kick_work_J",Work(m,a.motion.reaction));
    Number(m,"native_delta_J",a.motion.native_delta);Number(m,"effective_delta_J",a.motion.effective_delta);
    Number(m,"replacement_delta_J",a.motion.replacement_delta);Number(m,"native_residual_J",a.motion.native_residual);
    Number(m,"effective_residual_J",a.motion.effective_residual);Number(m,"roundoff_budget_J",a.motion.roundoff_budget);
    String(m,"reaction_work_semantics","Native constraint reaction kick work; recurrence consistency, not dissipation");
    String(m,"energy_semantics","Stored midpoint with lagged-frame observation; effective residual is bookkeeping, not collocated or independent physical energy conservation");
    Child(d,"motion",m);return d;
}
} // namespace crash::output::assembly::wall_fields
