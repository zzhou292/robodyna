#include "GroupStepNativeFixture.h"

extern "C" {
void nodal_rigid_native_frame(const double*,const double*,const double*,double*);
void nodal_rigid_native_wrench(const int*,const double*,const double*,const double*,const double*,double*);
void nodal_rigid_native_primary_step(const double*,const double*,const double*,const double*,const double*,const double*,
    const double*,const double*,const double*,double*,double*,double*,double*,double*,double*,double*);
void nodal_rigid_native_member_step(const int*,const double*,const double*,const double*,const double*,const double*,
    const double*,const double*,const double*,const double*,const double*,const double*,const double*,
    double*,double*,double*,double*,double*,double*,double*);
void nodal_rigid_native_two_member_step(const int*,const double*,const double*,const double*,const double*,const double*,
    const double*,const double*,const double*,const double*,const double*,const double*,const double*,
    double*,double*,double*,double*,double*,double*,double*,const double*);
}
namespace rigid_step_test {
namespace {
using Triple=std::array<double,3>;
Triple Values(Vec3 v) { return {v.x,v.y,v.z}; }
Vec3 Vector(const double* x) { return {x[0],x[1],x[2]}; }
std::array<double,9> NativeAxes(const tl::math::Matrix3& m) {
  std::array<double,9> output{};
  for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) output[3*i+j]=m.v[3*j+i];
  return output;
}
tl::math::Matrix3 Axes(const double* native) {
  tl::math::Matrix3 out;
  for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) out.v[3*i+j]=native[3*j+i];
  return out;
}
}
tl::math::Matrix3 NativeFrame(const tl::math::Matrix3& in,Vec3 omega,double dt) {
  const auto input=NativeAxes(in); const auto spin=Values(omega); std::array<double,9> output;
  nodal_rigid_native_frame(input.data(),spin.data(),&dt,output.data()); return Axes(output.data());
}
Trial NativePacketActive(const Input& in,int count,double length) {
   const auto& body=in.body;
  const auto frame=NativeAxes(body.previous_frame.axes);
  const auto j=Values(body.previous_frame.inertia);
  const auto center=Values(body.center),velocity=Values(body.velocity),omega=Values(body.omega);
  const double durations[]{body.durations.previous_drift_dt,body.durations.kick_dt,body.durations.drift_dt};
  double x[3*Count],v[3*Count],w[3*Count],f[3*Count],c[3*Count],mass[Count],inertia[Count];
  for(unsigned n=0;n<unsigned(count);++n) {
    const auto& m=in.member[n]; mass[n]=m.mass; inertia[n]=m.inertia;
    for(unsigned a=0;a<3;++a) { x[3*n+a]=Get(m.position,a); v[3*n+a]=Get(m.velocity,a);
      w[3*n+a]=Get(m.omega,a); f[3*n+a]=Get(m.force,a); c[3*n+a]=Get(m.couple,a); }
  }
  double wrench[6]; nodal_rigid_native_wrench(&count,x,center.data(),f,c,wrench);
  double axes[9],saved[3],a[3],alpha[3],new_x[3],new_v[3],new_w[3];
  nodal_rigid_native_primary_step(frame.data(),j.data(),center.data(),velocity.data(),omega.data(),&body.mass,
      wrench,wrench+3,durations,axes,saved,a,alpha,new_x,new_v,new_w);
  Trial out; auto& p=out.primary;
  p.force_frame={Axes(axes),body.previous_frame.inertia}; p.saved_body_omega=Vector(saved);
  p.acceleration=Vector(a); p.angular_acceleration=Vector(alpha);
  p.center=Vector(new_x); p.velocity=Vector(new_v); p.omega=Vector(new_w);
  double ma[3*Count],mar[3*Count],mx[3*Count],mv[3*Count],mw[3*Count],mr[3*Count],mrc[3*Count];
  if(count==2) {
  nodal_rigid_native_two_member_step(&count,center.data(),velocity.data(),new_w,a,durations,
      x,v,w,mass,inertia,f,c,ma,mar,mx,mv,mw,mr,mrc,&length);
  } else {
  nodal_rigid_native_member_step(&count,center.data(),velocity.data(),new_w,a,durations,
      x,v,w,mass,inertia,f,c,ma,mar,mx,mv,mw,mr,mrc);
  }
  for(unsigned n=0;n<unsigned(count);++n) {
    auto& m=out.member[n]; m.acceleration=Vector(ma+3*n); m.angular_acceleration=Vector(mar+3*n);
    m.position=Vector(mx+3*n); m.velocity=Vector(mv+3*n); m.omega=Vector(mw+3*n);
    m.reaction_force=Vector(mr+3*n); m.reaction_couple=Vector(mrc+3*n);
  }
  return out;
}
Trial NativePacket(const Input& in) { return NativePacketActive(in,Count,1); }
Trial NativeTwoPacket(const Input& in,double length) { return NativePacketActive(in,2,length); }
} // namespace rigid_step_test
