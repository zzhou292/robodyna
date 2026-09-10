#pragma once
#include "ForceStageFixture.h"

namespace crash::cases::source_assembly_observation::test {
using Triple=std::array<long double,3>;
inline Triple Values(tl::math::Vec3 v) {return {v.x,v.y,v.z};}
inline Triple Values(const double* data,std::size_t n) {return {data[3*n],data[3*n+1],data[3*n+2]};}
inline Triple Collocated(Triple before,Triple a,long double half) {
    for(unsigned k=0;k<3;++k) before[k]+=a[k]*half;
    return before;
}
inline long double Square(Triple v) {return v[0]*v[0]+v[1]*v[1]+v[2]*v[2];}
struct ForceOracle {long double ordinary=0,members=0,groups=0,rotation=0,primary=0,replacement=0;};
inline ForceOracle Oracle(const ForceStageInput& in) {
    ForceOracle out; const auto& model=*in.bindings->rigid_groups();
    std::vector<bool> member(in.before.node_count,false);
    const long double half=in.base.epoch?static_cast<long double>(in.base.fixed_dt)/2:0;
    const auto nodal=[&](std::size_t n) {
        const auto v=Collocated(Values(in.before.velocity_xyz,n),Values(in.acceleration_xyz,n),half);
        const auto w=Collocated(Values(in.before.angular_velocity_xyz,n),Values(in.angular_acceleration_xyz,n),half);
        const auto& mass=in.bindings->shells().nodes()[n].native;
        return .5L*mass.mass*Square(v)+.5L*mass.isotropic_inertia*Square(w);
    };
    for(std::size_t g=0;g<in.group_count;++g) {
        const auto& p=model.groups()[g]; const auto& before=in.before_groups[g].state;
        const auto& a=in.group_acceleration[g]; const auto& frame=in.force_groups[g].state.principal_axes;
        const auto v=Collocated(Values(before.velocity),Values(a.acceleration),half);
        const auto w=Collocated(Values(before.omega),Values(a.angular_acceleration),half);
        const auto principal=Values(p.principal.inertia);
        long double rotation=0,members=0;
        // Independent world tensor R J R^T, not production local-spin math.
        for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) for(unsigned k=0;k<3;++k)
            rotation+=.5L*w[i]*frame.v[3*i+k]*principal[k]*frame.v[3*j+k]*w[j];
        for(std::size_t i=0;i<p.member_count;++i) {
            const auto n=model.members()[p.member_offset+i].global_node; member[n]=true; members+=nodal(n);
        }
        const auto group=.5L*p.total_mass_kg*Square(v)+rotation;
        out.groups+=group; out.members+=members; out.rotation+=rotation;
        out.primary+=.5L*p.regularization.primary_mass_kg*Square(v); out.replacement+=group-members;
    }
    for(std::size_t n=0;n<member.size();++n) if(!member[n]) out.ordinary+=nodal(n);
    return out;
}
inline void Check(const ForceStageSummary& out,const ForceOracle& expected) {
    Near(out.ordinary.total,expected.ordinary); Near(out.grouped_members.total,expected.members);
    Near(out.groups.total,expected.groups); Near(out.groups.rotation,expected.rotation);
    Near(out.groups.primary_translation,expected.primary);
    Near(out.native_total,expected.ordinary+expected.members); Near(out.effective_total,expected.ordinary+expected.groups);
    EXPECT_LE(std::abs(static_cast<long double>(out.replacement)-expected.replacement),
              2e-12L*std::max(1e-25L,expected.groups+expected.members));
}
} // namespace crash::cases::source_assembly_observation::test
