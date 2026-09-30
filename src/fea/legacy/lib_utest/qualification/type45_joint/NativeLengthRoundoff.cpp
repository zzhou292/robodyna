// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeLengthRoundoff.h"
#include <algorithm>
#include <limits>

namespace type45_test {
namespace {
namespace d=tl::fea::type45::detail;
static_assert(std::numeric_limits<long double>::digits>=64,"Packet check needs extended precision");
constexpr long double Eps=std::numeric_limits<double>::epsilon();
constexpr long double Gamma=12*Eps/(1-12*Eps);
long double Abs(long double v) { return std::abs(v); }
long double Component(Vec3 v,unsigned i) { return d::Get(v,i); }
struct Projection { long double value=0, error=0; };
Projection Project(const double* frame,bool native,unsigned axis,
                   const Vec3* positions,double length) {
  Projection out;
  for(unsigned j=0;j<3;++j) {
    // These are exactly the represented inputs passed to each implementation.
    const double a=d::Get(positions[0],j)/length;
    const double b=d::Get(positions[1],j)/length;
    const long double r=frame[native ? 3*axis+j : 3*j+axis];
    const long double term=r*((long double)b-a);
    out.value+=term;
    out.error+=Abs(term);
  }
  out.value*=length;
  out.error*=Gamma*length;
  return out;
}
long double NativeDamping(const NativeOracle& n,unsigned axis) {
  const double a=n.state.uvar[33],b=n.state.uvar[34];
  const double mass=a*b/(a+b);
  const double critical=d::Blocked(static_cast<Kind>(n.kind),axis) ? n.property[1] : 0;
  return (critical*std::sqrt(n.state.uvar[18+axis]*mass)+n.property[8+axis])*n.mass;
}
long double ProdDamping(const NativeOracle& n,unsigned axis) {
  const auto& r=n.compared_reference;
  const double a=r.damping(0).mass_kg,b=r.damping(1).mass_kg;
  const double mass=a*b/(a+b);
  const double critical=d::Blocked(r.property().kind,axis) ? r.property().critical_damping_ratio : 0;
  return critical*std::sqrt(d::Get(r.stiffness().translation,axis)*mass)+
    d::Get(r.property().free_viscosity.translation,axis);
}
} // namespace
bool WithinLengthBound(double actual,double expected,double bound) {
  return std::isfinite(actual) && std::isfinite(expected) && std::isfinite(bound) && bound>=0 &&
    std::abs(actual-expected)<=2e-12*std::max(1.,std::abs(expected))+bound;
}
LengthRoundoff CheckLengthRoundoff(const Evaluation& actual,const NativeOracle& n,const type45_native::Step& expected) {
  LengthRoundoff result{};
  if(n.length==1) return result; // Preserve every previous SI comparison.
  EXPECT_TRUE(n.comparison_ready);
  const auto& h=actual.history.values();
  const auto& reference=n.compared_reference;
  const long double dt=n.last_interval.dt_s;
  long double separation_error[3]{},force_error[3]{},lever_error[3]{};
  long double sp[3]{},sn[3]{},fp[3]{},fn[3]{};
  for(unsigned i=0;i<3;++i) {
    SCOPED_TRACE(i);
    const auto p=Project(h.frame.v,false,i,n.last_interval.position_m,1);
    const auto q=Project(n.state.uvar.data()+21,true,i,n.last_interval.position_m,n.length);
    const auto p0=Project(reference.frame().v,false,i,n.geometry.position_m,1);
    const auto q0=Project(n.initial_state.uvar.data()+21,true,i,n.geometry.position_m,n.length);
    EXPECT_LE(Abs(Component(actual.diagnostics.local_separation_m,i)-p.value),p.error);
    EXPECT_LE(Abs((long double)expected.values[20+i]*n.length-q.value),q.error);
    const long double dp=Component(h.local_displacement_m,i);
    const long double dn=(long double)n.state.history[i]*n.length;
    // Two projections, their subtraction, and the native SI output conversion.
    const long double ep=p.error+p0.error+Gamma*(Abs(p.value)+Abs(p0.value));
    const long double en=q.error+q0.error+Gamma*(Abs(q.value)+Abs(q0.value));
    EXPECT_LE(Abs(dp-(p.value-p0.value)),ep);
    EXPECT_LE(Abs(dn-(q.value-q0.value)),en);
    const long double displacement_error=Abs((p.value-p0.value)-(q.value-q0.value))+ep+en;
    separation_error[i]=Abs(p.value-q.value)+p.error+q.error;
    const long double oldp=Component(n.compared_history.local_displacement_m,i);
    const long double oldn=(long double)n.previous_state.history[i]*n.length;
    const long double kp=Component(reference.stiffness().translation,i);
    const long double kn=(long double)n.state.uvar[18+i]*n.mass;
    const long double cp=ProdDamping(n,i),cn=NativeDamping(n,i);
    const long double vp=(dp-oldp)/dt,vn=(dn-oldn)/dt;
    d::Set(result.projection_force_error[0],i,static_cast<double>(
      (Abs(kp)+Abs(cp)/dt)*ep+Gamma*(Abs(kp*dp)+Abs(cp*vp))));
    d::Set(result.projection_force_error[1],i,static_cast<double>(
      (Abs(kn)+Abs(cn)/dt)*en+Gamma*(Abs(kn*dn)+Abs(cn*vn))));
    force_error[i]=Abs(kp)*displacement_error+Abs(kp-kn)*Abs(dn)+
      Abs(cp)*(displacement_error+Abs(oldp-oldn))/dt+Abs(cp-cn)*Abs(vn)+
      Gamma*(Abs(kp*dp)+Abs(kn*dn)+Abs(cp*vp)+Abs(cn*vn));
    d::Set(result.local_force,i,static_cast<double>(force_error[i]));
    sp[i]=Component(actual.diagnostics.local_separation_m,i);
    sn[i]=(long double)expected.values[20+i]*n.length;
    fp[i]=Component(h.local_force_n,i);
    fn[i]=(long double)n.state.history[6+i]*n.force;
  }
  for(unsigned i=0;i<3;++i) {
    const unsigned j=(i+1)%3,k=(i+2)%3;
    lever_error[i]=.5L*(Abs(sp[j])*force_error[k]+Abs(fn[k])*separation_error[j]+
      Abs(sp[k])*force_error[j]+Abs(fn[j])*separation_error[k]+
      Gamma*(Abs(sp[j]*fp[k])+Abs(sp[k]*fp[j])+Abs(sn[j]*fn[k])+Abs(sn[k]*fn[j])));
  }
  for(unsigned row=0;row<3;++row) {
    long double world_error=0,moment_error[2]{};
    for(unsigned col=0;col<3;++col) {
      const long double rp=h.frame.v[3*row+col],rn=n.state.uvar[21+3*col+row];
      world_error+=Abs(rp)*force_error[col]+Abs(rp-rn)*Abs(fn[col])+
        Gamma*(Abs(rp*fp[col])+Abs(rn*fn[col]));
      const unsigned j=(col+1)%3,k=(col+2)%3;
      const long double mp=Component(h.local_couple_nm,col);
      const long double mn=(long double)n.state.history[9+col]*n.inertia;
      const long double lp=.5L*(sp[j]*fp[k]-sp[k]*fp[j]);
      const long double ln=.5L*(sn[j]*fn[k]-sn[k]*fn[j]);
      for(unsigned end=0;end<2;++end) {
        const long double sign=end==0 ? 1 : -1;
        moment_error[end]+=Abs(rp)*(Abs(mp-mn)+lever_error[col])+Abs(rp-rn)*Abs(mn+sign*ln)+
          Gamma*(Abs(rp)*(Abs(mp)+Abs(lp))+Abs(rn)*(Abs(mn)+Abs(ln)));
      }
    }
    d::Set(result.world_force,row,static_cast<double>(world_error));
    for(unsigned end=0;end<2;++end)
      d::Set(result.endpoint_couple[end],row,static_cast<double>(moment_error[end]));
  }
  return result;
}
} // namespace type45_test
