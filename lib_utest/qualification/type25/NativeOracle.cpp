// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <algorithm>
#include <cmath>

namespace type25_test {
namespace {
void Pack(spring::Vec3 v,double factor,double* out) { out[0]=v.x/factor;out[1]=v.y/factor;out[2]=v.z/factor; }
spring::Vec3 Unpack(const double* p,double factor) { return {p[0]*factor,p[1]*factor,p[2]*factor}; }
double Radius(double y,double z) { return std::sqrt(y*y+z*z); }
}
spring::Evaluation NativeEvaluate(spring::SourceUnits u,const spring::Property& p,const spring::Reference& ref,
    const spring::History& old,const spring::EndpointKinematics (&nodes)[2],double dt) {
  const double m=u.mass_to_kg,l=u.length_to_m,t=u.time_to_s;
  const double j=m*l*l,force=m*l/(t*t),moment=j/(t*t),velocity=l/t,spin=1/t,step=dt/t;
  double positions[6],velocities[6],omegas[6],y[3],lengths[2],oldmotion[6],motion[6];
  for(unsigned k=0;k<2;++k){Pack(nodes[k].position,l,positions+3*k);Pack(nodes[k].velocity,velocity,velocities+3*k);Pack(nodes[k].angular_velocity,spin,omegas+3*k);}
  Pack(old.transverse_axis,1,y);Pack(old.displacement_m,l,oldmotion);Pack(old.rotation_rad,1,oldmotion+3);
  spring::Evaluation out;
  type25_native_frame(positions,velocities,omegas,y,&step,out.frame.axes.v,out.frame.midpoint_axes.v,lengths);
  const double ref_length=ref.length_m/l;
  type25_native_deformation(oldmotion,velocities,omegas,out.frame.midpoint_axes.v,lengths,&ref_length,&step,motion);
  const double r=Radius(motion[1],motion[2]),rr=Radius(motion[4],motion[5]);
  const double x[4]={motion[0],r,motion[3],rr};
  const double oldx[4]={oldmotion[0],Radius(oldmotion[1],oldmotion[2]),oldmotion[3],Radius(oldmotion[4],oldmotion[5])};
  double oldlocalforce[3],oldlocalmoment[3];Pack(old.local_force_N,force,oldlocalforce);Pack(old.local_couple_Nm,moment,oldlocalmoment);
  const double oldf[4]={oldlocalforce[0],Radius(oldlocalforce[1],oldlocalforce[2]),oldlocalmoment[0],Radius(oldlocalmoment[1],oldlocalmoment[2])};
  double k[4],c[4],lo[4],hi[4],e[4],f[4],work[4];
  for(unsigned a=0;a<4;++a){const double sf=a<2?force:moment,sx=a<2?l:1;
    k[a]=p.stiffness[a]/(sf/sx);c[a]=p.damping[a]/(sf*t/sx);
    lo[a]=p.failure_negative[a]/sf;hi[a]=p.failure_positive[a]/sf;e[a]=old.internal_work_J[a]/moment;}
  int active=old.active?1:0,nextactive;
  type25_native_response(k,c,x,oldx,oldf,e,lo,hi,p.failure_weight,p.failure_exponent,&step,&active,&old.failure_criterion,
      f,work,&nextactive,&out.history.failure_criterion);
  const double localf[3]={f[0],f[1]*(r>0?motion[1]/r:0),f[1]*(r>0?motion[2]/r:1)};
  const double localm[3]={f[2],f[3]*(rr>0?motion[4]/rr:0),f[3]*(rr>0?motion[5]/rr:1)};
  double wrenches[12];type25_native_scatter(out.frame.axes.v,&lengths[0],localf,localm,wrenches);
  for(unsigned a=0;a<2;++a)out.endpoints[a]={Unpack(wrenches+6*a,force),Unpack(wrenches+6*a+3,moment)};
  out.history.displacement_m=Unpack(motion,l);out.history.rotation_rad=Unpack(motion+3,1);
  out.history.transverse_axis={out.frame.axes.v[1],out.frame.axes.v[4],out.frame.axes.v[7]};
  out.history.local_force_N=Unpack(localf,force);out.history.local_couple_Nm=Unpack(localm,moment);
  out.history.active=nextactive!=0;
  for(unsigned a=0;a<4;++a)out.history.internal_work_J[a]=work[a]*moment;
  const double mass=p.mass_kg/m,inertia=p.isotropic_inertia_kg_m2/j;
  const double kt=std::max(k[0],k[1]),kr=std::max(k[2],k[3])+k[1]*lengths[0]*lengths[0];
  const double ct=std::max(c[0],c[1]),cr=std::max(c[2],c[3])+c[1]*lengths[0]*lengths[0];
  type25_native_dt(&mass,&inertia,&kt,&kr,&ct,&cr,&out.critical_dt_s);out.critical_dt_s*=t;
  out.translation_stiffness_N_per_m=kt*force/l;out.rotation_stiffness_Nm_per_rad=kr*moment;
  out.frame.length_m=lengths[0]*l;out.frame.midpoint_length_m=lengths[1]*l;
  return out;
}
} // namespace type25_test
