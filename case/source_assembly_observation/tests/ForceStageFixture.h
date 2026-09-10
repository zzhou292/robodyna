#pragma once
#include "ObservationFixture.h"
#include "case/source_assembly_observation/SourceAssemblyForceStageKinetic.h"

namespace crash::cases::source_assembly_observation::test {
struct ForceFixture : Fixture {
    Fields acceleration{stamp.node_count};
    std::vector<fe::NodalRigidGroupAccelerationSnapshot> group_acceleration;
    ForceFixture() {
        for(std::size_t n=0;n<stamp.node_count;++n) {
            acceleration.v[3*n]=1e7*(1+n%7); acceleration.v[3*n+1]=-2e7; acceleration.v[3*n+2]=3e7;
            acceleration.w[3*n]=2e7; acceleration.w[3*n+1]=1e7*(1+n%3); acceleration.w[3*n+2]=-1e7;
        }
        const auto& model=*bindings.rigid_groups();
        for(std::size_t g=0;g<model.group_count();++g) {
            const auto& p=model.groups()[g];
            group_acceleration.push_back({p.source_group_id,p.source_node_set_id,p.member_count,
                                         {1e7*(g+1),-3e7,2e7},{-2e7,1e7*(g+1),3e7}});
        }
    }
    void Later() {
        stamp.epoch=2; stamp.time=2*H; stamp.velocity_time=1.5*H;
        stamp.velocity_phase=fe::NodalVelocityPhase::PreviousMidpoint;
        stamp.reactions_valid=true; stamp.reaction_base_epoch=1;
        stamp.reaction_time=H; stamp.reaction_kick_dt=H;
        for(std::size_t n=0;n<stamp.node_count;++n) {
            old.v[3*n]=.125*(1+n%11); old.w[3*n+1]=-.25;
        }
        const double c=std::cos(.23),s=std::sin(.23);
        for(std::size_t g=0;g<old_groups.size();++g) {
            auto& before=old_groups[g].state; before.velocity={.2*(g+1),-.3,.4}; before.omega={.5,-.7,.2*(g+1)};
            next_groups[g].state=before;
            auto& axes=next_groups[g].state.principal_axes;
            for(unsigned k=0;k<3;++k) {
                const double x=before.principal_axes.v[k],y=before.principal_axes.v[3+k];
                axes.v[k]=c*x-s*y; axes.v[3+k]=s*x+c*y;
            }
        }
    }
    ForceStageInput force_input() const {
        ForceStageInput in; in.bindings=&bindings; in.base=in.before_group_stamp=stamp;
        in.before=old.view(); in.before_groups=old_groups.data(); in.force_groups=next_groups.data();
        in.group_count=old_groups.size(); in.prepared=Fixture::input().prepared;
        // Value-fixture identity addresses only. No pointer is dereferenced;
        // these are not authenticated live owner/CUDA trajectory claims.
        auto fill=[](fe::DeviceNodalKinematicsView& v,std::uintptr_t address) {
            v.position_xyz=reinterpret_cast<const double*>(address);
            v.velocity_xyz=reinterpret_cast<const double*>(address+0x10000);
            v.angular_velocity_xyz=reinterpret_cast<const double*>(address+0x20000);
            v.orientation_wxyz=reinterpret_cast<const double*>(address+0x30000);
        };
        fill(in.prepared.kinematics,0x100000); fill(in.prepared.base_kinematics,0x200000);
        in.frame_prepared=in.capture_prepared=in.prepared;
        in.acceleration_xyz=acceleration.v.data(); in.angular_acceleration_xyz=acceleration.w.data();
        in.group_acceleration=group_acceleration.data(); in.acceleration_nodes=stamp.node_count;
        in.acceleration_groups=group_acceleration.size(); return in;
    }
};
} // namespace crash::cases::source_assembly_observation::test
