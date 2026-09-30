#pragma once
#include "SourceAssemblyFlightFixture.h"
namespace crash::qualification::source_assembly {
inline tl::math::Vec3 ParentVector(const std::vector<double>& a,std::size_t n) { return {a[3*n],a[3*n+1],a[3*n+2]}; }
inline q::PrescribedInterval QParentInterval(const Rig& r,const Prepared& p,std::size_t e) {
    q::PrescribedInterval in;in.base_time=p.view.base_time;in.dt=TimeStep;in.sample_index=p.view.kinematics.base_epoch+1;
    for(unsigned n=0;n<4;++n) {
        const auto global=r.bindings.shells().qeph_nodes(e)[n];
        in.position_endpoint[n]=ParentVector(p.endpoint.x,global);in.velocity_midpoint[n]=ParentVector(p.endpoint.v,global);
        in.omega_midpoint[n]=ParentVector(p.endpoint.w,global);
    }
    return in;
}
inline t::PrescribedInterval TParentInterval(const Rig& r,const Prepared& p,std::size_t e) {
    t::PrescribedInterval in;in.base_time=p.view.base_time;in.dt=TimeStep;in.sample_index=p.view.kinematics.base_epoch+1;
    for(unsigned n=0;n<3;++n) {
        const auto global=r.bindings.shells().t3_nodes(e)[n];
        in.position[n]=ParentVector(p.endpoint.x,global);in.velocity[n]=ParentVector(p.endpoint.v,global);
        in.angular_velocity[n]=ParentVector(p.endpoint.w,global);
    }
    return in;
}
}
