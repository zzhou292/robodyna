#pragma once
#include "case/source_assembly_observation/SourceAssemblyObservation.h"
#include "case/source_assembly/tests/SourceAssemblyBindingTestSupport.h"
#include <cstring>
#include <limits>

namespace crash::cases::source_assembly_observation::test {
inline constexpr double H=1./67108864;
template<class T> auto Bytes(const T& value) {
    std::array<unsigned char,sizeof(T)> out;
    std::memcpy(out.data(),&value,sizeof(T)); return out;
}
struct Fields {
    explicit Fields(std::size_t count):v(3*count),w(3*count) {}
    std::vector<double> v,w;
    fe::HostNodalKinematicsView view() const { return {nullptr,v.data(),w.data(),v.size()/3,nullptr}; }
};
inline fe::ShellBatchKinetic Kinetic(const SourceAssemblyBindings& b,const Fields& f) {
    fe::ShellBatchKinetic sum;
    for(std::size_t n=0;n<b.shells().node_count();++n) {
        const auto& m=b.shells().nodes()[n].native;
        for(unsigned a=0;a<3;++a) {
            const double v=f.v[3*n+a],w=f.w[3*n+a];
            sum.translation+=.5*m.mass*v*v; sum.rotation+=.5*m.isotropic_inertia*w*w;
            sum.physical_isotropic+=.5*m.physical_inertia*w*w; sum.added_isotropic+=.5*m.added_inertia*w*w;
        }
    }
    return sum;
}
struct Fixture {
    SourceAssemblyBindings bindings=SourceAssemblyBindings::Prepare(source_assembly::test::Load(),source_assembly::test::Options());
    Fields old{bindings.shells().node_count()},next{bindings.shells().node_count()};
    std::vector<fe::NodalRigidGroupSnapshot> old_groups,next_groups;
    std::vector<double> force=std::vector<double>(old.v.size()),couple=force,reaction=force,reaction_couple=force;
    fe::NodalStamp stamp;
    Fixture() {
        const auto& model=*bindings.rigid_groups();
        for(std::size_t n=0;n<old.v.size()/3;++n) old.v[3*n]=next.v[3*n]=8;
        for(std::size_t g=0;g<model.group_count();++g) {
            const auto& p=model.groups()[g];
            old_groups.push_back({p.source_group_id,p.source_node_set_id,{p.center,{8,0,0},{},p.principal.axes}});
        }
        next_groups=old_groups;
        stamp.owner_id=81; stamp.node_count=old.v.size()/3; stamp.fixed_dt=H; stamp.has_rotations=true;
        stamp.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
        stamp.rigid_groups={bindings.source_instance_id(),model.group_count(),model.member_count()};
    }
    Input input() const {
        Input in; in.bindings=&bindings; in.base=stamp;
        auto& p=in.prepared; p.owner_id=stamp.owner_id; p.attempt=9;
        p.kinematics.node_count=p.base_kinematics.node_count=stamp.node_count;
        p.kinematics.base_epoch=p.base_kinematics.base_epoch=stamp.epoch;
        p.temporal_scheme=stamp.temporal_scheme; p.base_velocity_phase=stamp.velocity_phase;
        p.velocity_phase=fe::NodalVelocityPhase::PreviousMidpoint; p.base_time=stamp.time;
        p.base_velocity_time=stamp.velocity_time; p.proposed_time=stamp.time+H;
        p.velocity_time=stamp.time+.5*H; p.kick_dt=stamp.epoch ? H : .5*H; p.rigid_groups=stamp.rigid_groups;
        in.before=old.view(); in.after=next.view(); in.before_groups=old_groups.data(); in.after_groups=next_groups.data();
        in.group_count=old_groups.size(); in.applied_force_xyz=force.data(); in.applied_couple_xyz=couple.data();
        in.reaction_force_xyz=reaction.data(); in.reaction_couple_xyz=reaction_couple.data();
        in.base_kinetic=Kinetic(bindings,old); in.kinetic=Kinetic(bindings,next); return in;
    }
    std::size_t LastOrdinary() const {
        std::vector<bool> member(stamp.node_count,false);
        const auto& model=*bindings.rigid_groups();
        for(std::size_t i=0;i<model.member_count();++i) member[model.members()[i].global_node]=true;
        for(std::size_t n=member.size();n--;) if(!member[n]) return n;
        return SIZE_MAX;
    }
};
inline void Near(double actual,long double expected) {
    EXPECT_LE(std::abs(static_cast<long double>(actual)-expected),2e-12L*std::max(1e-25L,std::abs(expected)));
}
} // namespace crash::cases::source_assembly_observation::test
