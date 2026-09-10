#include "State.h"
#include "case/wall_penalty/WallPenaltyCertification.h"
#include <algorithm>
#include <cmath>

namespace crash::cases::source_assembly_dynamics {
namespace {
bool Enclosed(double value,const contact::Q4CertifiedIntegral& bounds) noexcept {
    return std::isfinite(value)&&contact::nodal_wall_detail::Certificate(bounds)&&
        value>=bounds.lower&&value<=bounds.upper;
}
bool ZeroSections(const std::vector<fe::ShellBatchSectionState>& states) noexcept {
    for(const auto& s:states) {
        if(s.cumulative_plastic_work_J!=0)return false;
        for(const auto& p:s.history.point) {
            if(p.plastic_strain!=0||p.filtered_rate_per_s!=0)return false;
            for(double v:p.stress)if(v!=0)return false;
        }
    }
    return true;
}
}
Report SourceAssemblyWallCase::Impl::Initialize() {
    auto& initial=accepted();const auto& b=bindings.shells();const auto& settings=*setup.settings();
    const auto wall_mesh=setup.placed_wall()->view();
    for(std::size_t i=0;i<wall_faces.size();++i)wall_faces[i]=wall_mesh.triangles[i].triangle_id;
    std::sort(wall_faces.begin(),wall_faces.end());
    const auto& velocity=settings.initial_velocity;
    for(std::size_t n=0;n<nodes();++n) {
        const auto& native=b.nodes()[n];
        const double x[]{native.position.x,native.position.y,native.position.z};
        for(unsigned a=0;a<3;++a) {initial.fields.x[3*n+a]=x[a];initial.fields.v[3*n+a]=velocity[a];}
        initial.fields.orientation[4*n]=1;
        inverse_mass[n]=1/native.native.mass;inverse_inertia[n]=1/native.native.isotropic_inertia;
    }
    fe::NodalStateConfig nc;nc.node_count=nodes();nc.fixed_dt=config.fixed_dt;
    nc.max_device_bytes=config.storage.owner_device_bytes;nc.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
    auto r=Convert(owner.Initialize(nc,initial.fields.view(),inverse_mass.data(),
        {free.data(),free.data(),inverse_inertia.data()},*bindings.rigid_groups()));if(!r)return r;
    q::QephBatchConfig qc;qc.owner=owner.accepted();qc.element_count=quads();
    qc.configuration_id=settings.configuration_id;qc.qualification_id=settings.qualification_id;
    qc.usage=q::BatchUsage::CoupledForces;qc.max_device_bytes=config.storage.qeph_device_bytes;
    qc.storage_limits.max_parents=config.storage.max_parents;qc.storage_limits.max_nodes=config.storage.max_nodes;
    qc.startup={fe::ShellBatchStartupKind::ReferenceUniformTranslation,{velocity[0],velocity[1],velocity[2]}};
    t::T3BatchConfig tc;tc.owner=qc.owner;tc.element_count=triangles();
    tc.configuration_id=qc.configuration_id;tc.qualification_id=qc.qualification_id;tc.usage=t::BatchUsage::CoupledForces;
    tc.max_device_bytes=config.storage.t3_device_bytes;tc.storage_limits=qc.storage_limits;tc.startup=qc.startup;
    r=QReport(qeph.InitializeJoined(qc,b,bindings.materials()));if(!r)return r;
    r=TReport(t3.InitializeJoined(tc,b,bindings.materials()));if(!r)return r;
    fe::NodalTrialToken initial_token;fe::NodalAssemblyView initial_view;
    r=Convert(owner.BeginTrial(&initial_token,&initial_view));if(!r)return r;
    r=QReport(qeph.AssembleAccepted(owner,initial_view));if(!r)return Stop(r);
    r=TReport(t3.AssembleAccepted(owner,initial_view));if(!r)return Stop(r);
    Discard();
    r=Convert(publication.Initialize(owner,qeph,t3,config.storage.publication));if(!r)return r;
    r=Convert(owner.CopyAccepted(initial.fields.buffer(),&initial.diagnostics.stamp));if(!r)return r;
    fe::NodalStamp group_stamp;
    r=Convert(owner.CopyAcceptedRigidGroups({initial.fields.groups.data(),groups()},&group_stamp));if(!r)return r;
    if(!fe::trial_identity::SameStamp(group_stamp,initial.diagnostics.stamp))
        return Failure(Status::ComponentFailure,"Initial source group and nodal stamps disagree");
    r=Convert(publication.CopyAcceptedDiagnostics(initial.diagnostics.stamp,&initial.diagnostics.shells));if(!r)return r;
    r=QReport(qeph.CopyAcceptedResults(initial.diagnostics.stamp,initial.parents.qeph.data(),quads(),&initial.diagnostics.shells.qeph));if(!r)return r;
    r=TReport(t3.CopyAcceptedResults(initial.diagnostics.stamp,initial.parents.t3.data(),triangles(),&initial.diagnostics.shells.t3));if(!r)return r;
    r=QReport(qeph.CopyAcceptedSectionHistory(initial.diagnostics.stamp,initial.parents.qsection.data(),quads(),&initial.diagnostics.shells.qeph));if(!r)return r;
    r=TReport(t3.CopyAcceptedSectionHistory(initial.diagnostics.stamp,initial.parents.tsection.data(),triangles(),&initial.diagnostics.shells.t3));if(!r)return r;
    if(!ZeroSections(initial.parents.qsection)||!ZeroSections(initial.parents.tsection))
        return Failure(Status::ComponentFailure,"Source startup contains nonzero plastic history");
    r=Convert(observation::ObserveInitial(bindings,initial.diagnostics.stamp,initial.fields.view(),
        initial.fields.groups.data(),groups(),initial.diagnostics.shells.kinetic,&initial.diagnostics.motion.after));if(!r)return r;
    const auto& kinetic=setup.certificate()->initial_kinetic;
    if(!Enclosed(initial.diagnostics.motion.after.native_total,kinetic.native_nodes)||
       !Enclosed(initial.diagnostics.motion.after.effective_total,kinetic.with_aggregate_groups))
        return Failure(Status::ObservationFailure,"Measured startup kinetic does not match its corresponding wall-design metric");
    contact::NodalWallDeviceConfig wc;
    const auto made=setup.MakeDeviceConfig(initial.diagnostics.stamp,&wc,config.storage.contact);
    if(!made)return Failure(Status::SourceMismatch,made.message);
    r=Convert(wall.Initialize(wc,setup.placed_wall()->view(),*setup.source_geometry()->weights(),
        {initial.fields.x.data(),nodes(),3,1},inverse_mass.data(),free.data(),setup.certificate()->coverage.physical));if(!r)return r;
    const auto rate=wall_penalty::CheckContactStep(config.fixed_dt,wall.stiffness_rate_bound(),
        settings.maximum_step_rate,&contact_step_rate_upper);
    if(!rate)return Failure(Status::EnvelopeFailure,rate.message,0,rate.node,contact_step_rate_upper,settings.maximum_step_rate);
    if(Allocations().device_bytes>config.storage.max_device_bytes)
        return Failure(Status::ResourceLimit,"Assembly participants exceed the total owned device-byte budget");
    return Success();
}
} // namespace crash::cases::source_assembly_dynamics
