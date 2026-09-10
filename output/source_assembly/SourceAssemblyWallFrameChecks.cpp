#include "SourceAssemblyWallFields.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include "lib_src/math/Quaternion.h"
#include <cmath>

namespace crash::output::assembly::wall_fields {
namespace fe=tl::fea;
namespace {
template<class D> void Family(const D& a,const D& b,const fe::NodalStamp& s,
                             const cases::source_assembly::SourceAssemblyWallSettings& settings) {
    Require(a.valid&&b.valid&&a.phase==decltype(a.phase)::Accepted&&b.phase==a.phase&&
        a.usage==decltype(a.usage)::CoupledForces&&b.usage==a.usage&&a.owner_id==s.owner_id&&b.owner_id==a.owner_id&&
        a.epoch==s.epoch&&b.epoch==a.epoch&&a.time==s.time&&b.time==a.time&&a.velocity_time==s.velocity_time&&
        b.velocity_time==a.velocity_time&&a.configuration_id==settings.configuration_id&&b.configuration_id==a.configuration_id&&
        a.qualification_id==settings.qualification_id&&b.qualification_id==a.qualification_id&&a.attempt==b.attempt&&
        a.base_epoch==b.base_epoch&&a.base_time==b.base_time&&a.base_velocity_time==b.base_velocity_time&&a.kick_dt==b.kick_dt&&
        a.has_completed_interval==bool(s.epoch)&&b.has_completed_interval==a.has_completed_interval&&
        a.accepted_force_assembled==bool(s.epoch)&&b.accepted_force_assembled==a.accepted_force_assembled&&
        !a.kinetic_available&&!b.kinetic_available,"Assembly accepted family/source/phase mismatch");
    if(s.epoch)Require(a.base_epoch==s.reaction_base_epoch&&a.base_time==s.reaction_time&&
        a.kick_dt==s.reaction_kick_dt&&a.attempt,"Assembly family interval does not match accepted reaction phase");
    else Require(!a.base_epoch&&!a.attempt&&a.base_time==0&&a.base_velocity_time==0&&a.kick_dt==0,
        "Initial family fabricates a completed interval");
    const double av[]{a.kinetic_translation,a.kinetic_rotation,a.kinetic_physical_isotropic,a.kinetic_added_isotropic,
        a.internal_work[0],a.internal_work[1],a.internal_work_increment[0],a.internal_work_increment[1],
        a.minimum_area_ratio,a.minimum_thickness_ratio,a.maximum_displacement,a.maximum_absolute_strain,
        a.maximum_thickness_curvature,a.minimum_native_dt,a.internal_kick_work,a.internal_drift_work};
    const double bv[]{b.kinetic_translation,b.kinetic_rotation,b.kinetic_physical_isotropic,b.kinetic_added_isotropic,
        b.internal_work[0],b.internal_work[1],b.internal_work_increment[0],b.internal_work_increment[1],
        b.minimum_area_ratio,b.minimum_thickness_ratio,b.maximum_displacement,b.maximum_absolute_strain,
        b.maximum_thickness_curvature,b.minimum_native_dt,b.internal_kick_work,b.internal_drift_work};
    for(unsigned i=0;i<16;++i)Require(std::isfinite(av[i])&&Bits(av[i])==Bits(bv[i]),"Captured family diagnostic changed");
    for(unsigned i=0;i<4;++i)Require(av[i]==0,"Joined family cannot publish an independent kinetic metric");
}
void Kinetic(const fe::ShellBatchKinetic& a,const fe::ShellBatchKinetic& b) {
    const double av[]{a.translation,a.rotation,a.physical_isotropic,a.added_isotropic};
    const double bv[]{b.translation,b.rotation,b.physical_isotropic,b.added_isotropic};
    for(unsigned i=0;i<4;++i)Require(std::isfinite(av[i])&&av[i]>=0&&Bits(av[i])==Bits(bv[i]),"Captured native kinetic changed");
}
void Phase(const fe::rigid::ObservationPhase& p,bool initial,double x,double v,double frame) {
    Require(p.kind==(initial?fe::rigid::ObservationPhaseKind::PhysicalInitialization:
        fe::rigid::ObservationPhaseKind::StoredMidpointWithLaggedFrame)&&p.position_time==x&&
        p.velocity_time==v&&p.frame_time==frame,"Assembly stored observation phase mismatch");
}
}
void CheckFrame(const FrameView& v) {
    Require(v.surface&&v.bindings&&v.setup&&v.stamp&&v.diagnostics&&v.captured_shells&&v.setup->initialized(),
        "Missing assembly frame association");
    const auto& s=*v.stamp;const auto& b=*v.bindings;const auto& src=b.source().data();
    Require(b.rigid_groups()&&b.rigid_groups()->prepared(),"Missing complete prepared group model");const auto& g=*b.rigid_groups();
    const auto& map=v.surface->binding();const auto& d=*v.diagnostics;const auto& n=v.nodes;
    Require(s.node_count==src.nodes.size()&&n.node_count==s.node_count&&n.position_xyz&&n.velocity_xyz&&
        n.orientation_wxyz&&n.angular_velocity_xyz&&v.qeph.count==src.qeph_count&&v.t3.count==src.t3_count&&
        v.qeph.values&&v.qeph.reported_thickness_m&&v.t3.values&&v.t3.reported_thickness_m&&
        (s.epoch?(v.contact.node_count==src.nodes.size()&&v.contact.parent_count==src.parents.size()&&
          v.contact.diagnostics&&v.contact.nodes&&v.contact.parents&&v.contact.wall_face):
          (!v.contact.diagnostics&&!v.contact.nodes&&!v.contact.parents&&!v.contact.wall_face&&!v.contact.node_count&&!v.contact.parent_count)),
        "Incomplete active assembly output extents");
    Require(s.owner_id&&map.identity.owner==s.owner_id&&map.identity.run&&map.identity.topology&&
        s.rigid_groups.source_instance_id==b.source_instance_id()&&s.rigid_groups.group_count==g.group_count()&&
        s.rigid_groups.member_count==g.member_count()&&s.has_rotations&&std::isfinite(s.time)&&s.time>=0&&
        std::isfinite(s.fixed_dt)&&s.fixed_dt>=1e-12&&s.temporal_scheme==fe::NodalTemporalScheme::StaggeredHalfKickStart&&
        s.velocity_phase==(s.epoch?fe::NodalVelocityPhase::PreviousMidpoint:fe::NodalVelocityPhase::Collocated)&&
        fe::trial_identity::SameStamp(s,d.stamp)&&d.has_interval==bool(s.epoch)&&v.captured_shells->valid&&d.shells.valid&&
        src.identity.sha256==v.surface->source().data().identity.sha256&&src.identity.bytes==v.surface->source().data().identity.bytes&&
        b.source_instance_id()==v.setup->bindings()->source_instance_id()&&
        b.shells().inventory()==v.setup->bindings()->shells().inventory(),"Assembly accepted owner/source scope mismatch");
    if(!s.epoch)Require(s.time==0&&s.velocity_time==0&&!s.reactions_valid&&!s.reaction_base_epoch&&
        s.reaction_time==0&&s.reaction_kick_dt==0,"Invalid physical initialization stamp");
    else Require(s.reactions_valid&&s.reaction_base_epoch+1==s.epoch&&s.time==s.reaction_time+s.fixed_dt&&
        s.velocity_time==s.reaction_time+.5*s.fixed_dt&&s.reaction_kick_dt==(s.epoch==1?.5*s.fixed_dt:s.fixed_dt),
        "Invalid accepted staggered interval stamp");
    CheckForceStageFrame(v);
    const auto& settings=*v.setup->settings();Family(v.captured_shells->qeph,d.shells.qeph,s,settings);
    Family(v.captured_shells->t3,d.shells.t3,s,settings);
    Require(Bits(v.captured_shells->qeph.hourglass_viscous_work)==Bits(d.shells.qeph.hourglass_viscous_work)&&
        Bits(v.captured_shells->qeph.hourglass_viscous_work_increment)==Bits(d.shells.qeph.hourglass_viscous_work_increment),
        "Captured native viscous work changed");
    Kinetic(v.captured_shells->base_kinetic,d.shells.base_kinetic);Kinetic(v.captured_shells->kinetic,d.shells.kinetic);
    Phase(d.motion.after.phase,!s.epoch,s.time,s.velocity_time,s.reaction_time);
    if(s.epoch) {
        const auto& p=d.motion.before.phase;
        Phase(p,s.epoch==1,s.reaction_time,d.shells.qeph.base_velocity_time,p.frame_time);
        Require(s.epoch==1?p.frame_time==0:p.frame_time+s.fixed_dt==s.reaction_time,"Base observation lagged frame mismatch");
    }
    Require(map.vertices.size()==src.nodes.size(),"Incomplete source vertex mapping");
    for(std::size_t i=0;i<map.vertices.size();++i)Require(map.vertices[i].tl_node==i&&
        map.vertices[i].source.instance==b.source_instance_id()&&map.vertices[i].source.node==src.nodes[i].source_id,
        "Assembly output source node/instance mapping changed");
    Require(ValidSections(*v.surface,v.qeph,v.t3),"Invalid complete source section fields");
    for(std::size_t i=0;i<n.node_count;++i) {
        const auto* q=n.orientation_wxyz+4*i;
        Require(tl::math::UnitQuaternion({q[0],q[1],q[2],q[3]}),"Invalid accepted orientation");
        for(unsigned a=0;a<3;++a)Require(std::isfinite(n.position_xyz[3*i+a])&&std::isfinite(n.velocity_xyz[3*i+a])&&
            std::isfinite(n.angular_velocity_xyz[3*i+a]),"Invalid accepted nodal field");
    }
}
} // namespace crash::output::assembly::wall_fields
