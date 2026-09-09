#include "ResponseSamples.h"
#include "ResponseFieldVisitor.h"
#include "ResponseBounds.h"
#include "lib_src/math/Quaternion.h"
#include "lib_src/elements/qeph/QephHistory.h"
#include <algorithm>
#include <cmath>

namespace tl::qualification::qeph::response {
bool Observe(const Model& m,double h,std::uint64_t epoch,const State& state,const Results& result,
             long double external_work,Sample& output,Limits& observed,std::string& error) {
  if((m.cells!=1&&m.cells!=2)||m.nodes!=2*(m.cells+1)||!std::isfinite(h)||h<=0||epoch>32768) {
    error="Invalid observation model/time"; return false;
  }
  Sample s; Limits limits; s.epoch=epoch; s.time=epoch*h; s.carried_velocity_time=epoch?s.time-h/2:0;
  s.kick_dt=epoch?(epoch==1?h/2:h):0; s.interval_available=epoch!=0;
  s.external_work=static_cast<double>(external_work);
  std::array<double,18> rhs{},torque{},vs=state.v,ws=state.omega,angle{};
  const double factor=PulseFactor(s.time);
  for(unsigned e=0;e<m.cells;++e) {
    const auto& r=result[e]; const auto& history=r.proposed_history;
    if(!history.prepared()||!history.matches_reference(m.reference[e])||history.stamp().sample_index!=epoch||history.stamp().time!=s.time) {
      error="Observation history does not belong to this accepted endpoint"; return false;
    }
    for(unsigned i=0;i<4;++i) { const auto n=m.connectivity[e][i];
      if(n>=m.nodes) { error="Invalid observation connectivity"; return false; }
      const auto f=r.internal_force[i],c=r.internal_couple[i];
      const double fs[]{f.x,f.y,f.z},cs[]{c.x,c.y,c.z};
      for(unsigned a=0;a<3;++a) { rhs[3*n+a]-=fs[a]; torque[3*n+a]-=cs[a]; }
    }
    const auto& values=history.data();
    s.source_work[0]+=values.internal_work[0]; s.source_work[1]+=values.internal_work[1]; s.source_work[2]+=values.hourglass_viscous_work;
    const double area=epoch?r.kinematics.area/m.reference[e].area:1.;
    const double thickness=values.thickness/Thickness;
    limits.minimum_area_ratio=std::min(limits.minimum_area_ratio,area); limits.maximum_area_ratio=std::max(limits.maximum_area_ratio,area);
    limits.minimum_thickness_ratio=std::min(limits.minimum_thickness_ratio,thickness); limits.maximum_thickness_ratio=std::max(limits.maximum_thickness_ratio,thickness);
    for(unsigned c=0;c<5;++c) limits.strain=std::max(limits.strain,std::abs(values.strain_curvature[c]));
    for(unsigned c=5;c<8;++c) limits.thickness_curvature=std::max(limits.thickness_curvature,Thickness*std::abs(values.strain_curvature[c]));
  }
  long double carried[4]{},synchronous[4]{};
  for(unsigned n=0;n<m.nodes;++n) {
    if(!(m.mass[n]>0&&m.inertia[n]>0&&m.physical[n]>0&&m.added[n]>0)) { error="Invalid global mass/inertia"; return false; }
    const auto station=n/2;
    if(m.cells==1) torque[3*n+1]+=(station?1.:-1.)*.5*BendingScale()*Theta*factor;
    else rhs[3*n+2]+=(station==1?-1.:.5)*BendingScale()*Delta/(Side*Side)*factor;
    const auto* q=state.q.data()+4*n;
    if(!tl::math::UnitQuaternion({q[0],q[1],q[2],q[3]})) { error="Invalid endpoint quaternion"; return false; }
    const double vector=std::hypot(std::hypot(q[1],q[2]),q[3]);
    const double signed_angle=2*std::atan2(vector,std::abs(q[0]))*(q[0]<0?-1.:1.);
    limits.rotation_angle=std::max(limits.rotation_angle,std::abs(signed_angle));
    double dx[3]{};
    for(unsigned a=0;a<3;++a) {
      const auto i=3*n+a; dx[a]=state.x[i]-m.initial_position[i]; angle[i]=vector?signed_angle*q[a+1]/vector:0;
      if(epoch) { // Scalar output reconstruction, not a second physical step.
        vs[i]=static_cast<double>(state.v[i]+.5L*h*rhs[i]/m.mass[n]);
        ws[i]=static_cast<double>(state.omega[i]+.5L*h*torque[i]/m.inertia[n]);
      }
      const long double v=state.v[i],w=state.omega[i],v_sync=vs[i],w_sync=ws[i];
      carried[0]+=.5L*m.mass[n]*v*v; synchronous[0]+=.5L*m.mass[n]*v_sync*v_sync;
      const double inertias[]{m.inertia[n],m.physical[n],m.added[n]};
      for(unsigned c=0;c<3;++c) { carried[c+1]+=.5L*inertias[c]*w*w; synchronous[c+1]+=.5L*inertias[c]*w_sync*w_sync; }
    }
    limits.displacement_over_side=std::max(limits.displacement_over_side,std::hypot(std::hypot(dx[0],dx[1]),dx[2])/Side);
  }
  for(unsigned c=0;c<4;++c) { s.carried_kinetic[c]=static_cast<double>(carried[c]); s.synchronous_kinetic[c]=static_cast<double>(synchronous[c]); }
  s.residual=static_cast<double>(synchronous[0]+synchronous[1]+s.source_work[0]+s.source_work[1]+s.source_work[2]-external_work);
  unsigned count=0; bool finite=true;
  detail::Visit(m,state,result,vs,ws,angle,[&](const char*,unsigned,unsigned,double value,const char*,double,bool) {
    if(count>=MaxFields||!std::isfinite(value)) { finite=false; return; } s.values[count++]=value;
  });
  for(double k:s.carried_kinetic) finite&=std::isfinite(k);
  for(double k:s.synchronous_kinetic) finite&=std::isfinite(k);
  for(double w:s.source_work) finite&=std::isfinite(w);
  finite&=std::isfinite(s.external_work)&&std::isfinite(s.residual)&&std::isfinite(s.time);
  if(!finite) { error="Nonfinite endpoint observation"; return false; }
  output=s; observed=limits; error.clear(); return true;
}
bool InsideLimits(const Limits& l) noexcept {
  return l.displacement_over_side<=1e-3&&l.rotation_angle<=1e-3&&l.strain<=1e-3&&l.thickness_curvature<=1e-3&&
    l.minimum_area_ratio>=.999&&l.maximum_area_ratio<=1.001&&l.minimum_thickness_ratio>=.999&&l.maximum_thickness_ratio<=1.001;
}
void AccumulateLimits(Limits& a,const Limits& b) noexcept {
  a.displacement_over_side=std::max(a.displacement_over_side,b.displacement_over_side);
  a.rotation_angle=std::max(a.rotation_angle,b.rotation_angle); a.strain=std::max(a.strain,b.strain);
  a.thickness_curvature=std::max(a.thickness_curvature,b.thickness_curvature);
  a.minimum_area_ratio=std::min(a.minimum_area_ratio,b.minimum_area_ratio); a.maximum_area_ratio=std::max(a.maximum_area_ratio,b.maximum_area_ratio);
  a.minimum_thickness_ratio=std::min(a.minimum_thickness_ratio,b.minimum_thickness_ratio);
  a.maximum_thickness_ratio=std::max(a.maximum_thickness_ratio,b.maximum_thickness_ratio);
}
Extremum ResponseMaximum(const std::vector<Field>& fields,const Sample& initial,const Sample& s) {
  Extremum result; result.time=s.time;
  for(unsigned i=0;i<fields.size();++i) if(fields[i].compare) {
    bounds::Interval interval;
    if(!bounds::NormalizedDifference(s.values[i],initial.values[i],fields[i].scale,interval))
      return {std::numeric_limits<double>::max(),s.time,i,0};
    result.lower=std::max(result.lower,interval.lower);
    if(interval.upper>result.value) { result.value=interval.upper; result.field=i; }
  }
  return result;
}
} // namespace tl::qualification::qeph::response
