#pragma once

#include "PersistentShell.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace tl::qualification::shell::test {
struct Vec { double x=0,y=0,z=0; };
inline Vec Add(Vec a,Vec b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
inline Vec Sub(Vec a,Vec b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
inline Vec Scale(Vec a,double s){return {a.x*s,a.y*s,a.z*s};}
inline double Dot(Vec a,Vec b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline Vec Cross(Vec a,Vec b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline double Norm(Vec a){return std::hypot(a.x,a.y,a.z);}
inline Vec Get(const std::array<double,3*MaxNodes>& a,int i){return {a[3*i],a[3*i+1],a[3*i+2]};}
inline void Put(std::array<double,3*MaxNodes>& a,int i,Vec v){a[3*i]=v.x;a[3*i+1]=v.y;a[3*i+2]=v.z;}
inline Vec Rotate(Vec p){
  const double a=.41,b=-.37;
  const Vec q{std::cos(a)*p.x-std::sin(a)*p.y,std::sin(a)*p.x+std::cos(a)*p.y,p.z};
  return {q.x,std::cos(b)*q.y-std::sin(b)*q.z,std::sin(b)*q.y+std::cos(b)*q.z};
}
inline void Near(double actual,double expected,double absolute=2e-10,double relative=2e-9){
  EXPECT_NEAR(actual,expected,absolute+relative*std::abs(expected));
}
inline void Near(Vec a,Vec b,double absolute=2e-7){
  Near(a.x,b.x,absolute);Near(a.y,b.y,absolute);Near(a.z,b.z,absolute);
}

struct Motion {
  std::array<double,3*MaxNodes> x{},v{},w{};
  std::size_t count=4;
  fea::HostNodalKinematicsView view() const {return {x.data(),v.data(),w.data(),count};}
};
inline Motion Rectangle(){
  Motion m;
  Put(m.x,0,{-.2,-.15,0});Put(m.x,1,{.2,-.15,0});
  Put(m.x,2,{.2,.15,0});Put(m.x,3,{-.2,.15,0});return m;
}
inline Configuration Config(){Configuration c;c.elements[0].nodes={0,1,2,3};return c;}
inline Motion Rotated(Motion m){
  for(std::size_t i=0;i<m.count;++i){
    Put(m.x,i,Add(Rotate(Get(m.x,i)),{.3,-.2,.5}));
    Put(m.v,i,Rotate(Get(m.v,i)));Put(m.w,i,Rotate(Get(m.w,i)));
  }
  return m;
}

// Engineering components: xx,yy,xy,yz,xz,kxx,kyy,kxy. These prescribed
// fixed-geometry operator probes intentionally do not integrate nodal positions.
using Strain = std::array<double,8>;
inline Strain Scale(Strain a,double s){for(auto& value:a)value*=s;return a;}
inline Strain Add(Strain a,const Strain& b){for(int i=0;i<8;++i)a[i]+=b[i];return a;}
inline void Rates(Motion& m,const Strain& increment,double dt){
  m.v.fill(0);m.w.fill(0);
  for(std::size_t i=0;i<m.count;++i){const auto p=Get(m.x,i);
    Put(m.v,i,{(increment[0]*p.x+.5*increment[2]*p.y)/dt,
               (.5*increment[2]*p.x+increment[1]*p.y)/dt,0});
    Put(m.w,i,{(-increment[6]*p.y-.5*increment[7]*p.x-increment[3])/dt,
               (increment[5]*p.x+.5*increment[7]*p.y+increment[4])/dt,0});
  }
}
inline Request MakeRequest(const PersistentShell& shell,const Motion& motion,double dt,
                           RejectAfter reject=RejectAfter::None){
  Request request;request.kinematics=motion.view();request.time_begin=shell.accepted().time;
  request.time_end=request.time_begin+dt;request.reject_after=reject;return request;
}
inline bool Evaluate(PersistentShell& shell,const Motion& motion,double dt,TrialToken* token){
  const Report report=shell.Evaluate(MakeRequest(shell,motion,dt),token);
  EXPECT_EQ(report.status,Status::Ok)<<report.message;
  if(report.status!=Status::Ok)return false;
  EXPECT_NE(shell.trial(),nullptr);return shell.trial()!=nullptr;
}
inline bool Step(PersistentShell& shell,const Motion& motion,double dt,Snapshot* out){
  TrialToken token;if(!Evaluate(shell,motion,dt,&token))return false;
  *out=*shell.trial();const Status status=shell.Commit(token);EXPECT_EQ(status,Status::Ok);
  return status==Status::Ok;
}

inline std::array<double,3> PlaneStress(double xx,double yy,double xy){
  const double a=Young/(1-Poisson*Poisson),g=Young/(2*(1+Poisson));
  return {a*(xx+Poisson*yy),a*(yy+Poisson*xx),g*xy};
}
inline double Area(const Motion& m,const std::array<int,4>& nodes){
  double twice=0;for(int i=0;i<4;++i){const auto a=Get(m.x,nodes[i]),b=Get(m.x,nodes[(i+1)%4]);twice+=a.x*b.y-b.x*a.y;}
  return .5*twice;
}
inline void CheckElastic(const ElementState& e,const Strain& total,double area,double t){
  const auto membrane=PlaneStress(total[0],total[1],total[2]);
  const auto bending=PlaneStress(total[5],total[6],total[7]);
  const double gs=ShearFactor*Young/(2*(1+Poisson));
  const double zeta[]={-.5*std::sqrt(3.0/5.0),0,.5*std::sqrt(3.0/5.0)};
  for(int j=0;j<8;++j)Near(e.generalized_strain[j],total[j],2e-12);
  for(int ip=0;ip<3;++ip){
    for(int j=0;j<3;++j)Near(e.points[ip].stress[j],membrane[j]+zeta[ip]*t*bending[j],2e-5);
    Near(e.points[ip].stress[3],gs*total[3],2e-5);Near(e.points[ip].stress[4],gs*total[4],2e-5);
    EXPECT_EQ(e.points[ip].plastic_strain,0);EXPECT_EQ(e.points[ip].plastic_increment,0);
    EXPECT_EQ(e.points[ip].plastic_rate,0);Near(e.points[ip].temperature,Temperature,1e-10);
    for(double b:e.points[ip].backstress)EXPECT_EQ(b,0);
  }
  for(int j=0;j<3;++j){Near(e.normalized_force[j],membrane[j],2e-5);Near(e.normalized_moment[j],t*bending[j]/12,2e-7);}
  Near(e.normalized_force[3],gs*total[3],2e-5);Near(e.normalized_force[4],gs*total[4],2e-5);
  const double membrane_work=.5*area*t*(total[0]*membrane[0]+total[1]*membrane[1]+total[2]*membrane[2]
                                      +gs*(total[3]*total[3]+total[4]*total[4]));
  const double bending_work=area*t*t*t/24*(total[5]*bending[0]+total[6]*bending[1]+total[7]*bending[2]);
  Near(e.work[0],membrane_work,2e-9);Near(e.work[1],bending_work,2e-9);
}

// Independent boundary integration: b_i=1/2(y_next-y_prev,x_prev-x_next).
// No K1 PX/PY values enter these expected forces or couples.
inline void AddExpectedForces(const Motion& motion,const Element& element,const Strain& total,double t,
                             std::array<Vec,MaxNodes>* forces,std::array<Vec,MaxNodes>* couples){
  auto n=PlaneStress(total[0],total[1],total[2]);for(auto& v:n)v*=t;
  auto b=PlaneStress(total[5],total[6],total[7]);for(auto& v:b)v*=t*t*t/12;
  const double area=Area(motion,element.nodes),gs=ShearFactor*Young/(2*(1+Poisson));
  const double qx=t*gs*total[4],qy=t*gs*total[3];
  for(int i=0;i<4;++i){
    const Vec prev=Get(motion.x,element.nodes[(i+3)%4]),next=Get(motion.x,element.nodes[(i+1)%4]);
    const double bx=.5*(next.y-prev.y),by=.5*(prev.x-next.x);
    const Vec f{-(n[0]*bx+n[2]*by),-(n[2]*bx+n[1]*by),-(qx*bx+qy*by)};
    const Vec c{b[2]*bx+b[1]*by+area*qy/4,-b[0]*bx-b[2]*by-area*qx/4,0};
    const int node=element.nodes[i];(*forces)[node]=Add((*forces)[node],f);(*couples)[node]=Add((*couples)[node],c);
  }
}
inline void Balance(const Motion& motion,const Snapshot& state){
  Vec f{},moment{};for(std::size_t i=0;i<state.node_count;++i){
    const Vec fi=Get(state.force,i);f=Add(f,fi);moment=Add(moment,Add(Cross(Get(motion.x,i),fi),Get(state.couple,i)));
  }
  Near(f,{},2e-6);Near(moment,{},2e-7);
}

// Serialize named numeric fields only; padding bytes are not part of the oracle.
inline std::vector<unsigned char> Bytes(const Snapshot& s){
  std::vector<unsigned char> out;
  const auto append=[&](const auto& value){const auto* p=reinterpret_cast<const unsigned char*>(&value);out.insert(out.end(),p,p+sizeof(value));};
  append(s.element_count);append(s.node_count);append(s.device_bytes);append(s.epoch);append(s.time);append(s.completed_dt);
  append(s.position);append(s.velocity);append(s.angular_velocity);append(s.force);append(s.couple);
  for(const auto& e:s.elements){
    append(e.off);append(e.area);append(e.thickness);append(e.step_thickness);append(e.px1);append(e.px2);append(e.py1);append(e.py2);append(e.vhx);append(e.vhy);
    append(e.frame);append(e.reference_coordinates);append(e.generalized_strain);append(e.normalized_force);append(e.normalized_moment);append(e.hour);append(e.work);
    append(e.material_work_increment);append(e.hourglass_work_increment);
    for(const auto& p:e.points){append(p.stress);append(p.plastic_strain);append(p.plastic_rate);append(p.plastic_increment);append(p.temperature);append(p.backstress);}
  }
  return out;
}
}  // namespace tl::qualification::shell::test
