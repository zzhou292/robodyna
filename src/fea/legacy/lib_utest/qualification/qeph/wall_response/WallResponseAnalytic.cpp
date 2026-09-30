#include "WallResponseInternal.h"

namespace tl::qualification::qeph::wall_response::detail {
// Independent continuous per-node spring. Trigonometric evaluation uses host
// long double and the predeclared256epsilon dimensional numerical allowance;
// this is not an exact-libm enclosure theorem. Rate uncertainty is propagated
// with global derivative bounds through contact and the continuous release.
bool Analytic(const Model& m,const Config&,Sample& s,std::string& error) {
  const auto& f=m.fields(); const long double speed=wr::ImpactSpeed,pi=std::acos(-1.L);
  const long double tau=static_cast<long double>(s.time)-wr::InitialGap/speed;
  long double total_u=0,total_j=0; double uncertainty_u=0,uncertainty_j=0;
  s.analytic_difference={};
  auto check=[&](double actual,double eta,long double expected,long double expected_eta,double scale) {
    Interval a,z,difference;
    const double nominal=static_cast<double>(expected);
    const double radius=static_cast<double>(expected_eta+Roundoff*(std::abs(expected)+scale));
    if(!Enclose(actual,eta,a)||!Enclose(nominal,radius,z)||!Difference(a,z,scale,difference))return false;
    Maximum(s.analytic_difference,difference); return true;
  };
  for(unsigned n=0;n<f.nodes;++n) {
    const auto& certificate=m.screened().mass_rates()[n];
    if(!contact::nodal_wall_detail::Certificate(certificate,true)||certificate.lower<=0)return Fail(error,"Invalid analytic mass-rate certificate");
    const long double rate=certificate.value,low=certificate.lower,high=certificate.upper;
    const long double omega=std::sqrt(rate),omega_low=std::sqrt(low),rate_error=std::max(rate-low,high-rate);
    long double x=-wr::InitialGap+speed*s.time,v=speed;
    if(tau>0) {
      if(tau<pi/omega) { x=speed/omega*std::sin(omega*tau); v=speed*std::cos(omega*tau); }
      else { x=-speed*(tau-pi/omega); v=-speed; }
    }
    const long double x_error=tau>0?speed*(1+std::sqrt(high)*Horizon+pi)/(2*omega_low*omega_low*omega_low)*rate_error:0;
    const long double v_error=tau>0?speed*Horizon/(2*omega_low)*rate_error:0;
    const long double depth=std::max(x,0.L),bound_depth=depth+x_error;
    const long double force=f.mass[n]*rate*depth;
    const long double force_error=f.mass[n]*(high*x_error+bound_depth*rate_error);
    const long double u=.5L*f.mass[n]*rate*depth*depth;
    const long double u_error=.5L*f.mass[n]*(high*(2*depth*x_error+x_error*x_error)+rate_error*bound_depth*bound_depth);
    const long double impulse=f.mass[n]*(speed-v),j_error=f.mass[n]*v_error;
    if(!check(s.state.x[3*n]-m.screened().law().wall_x,0,x,x_error,wr::TargetDepth)||
       !check(s.synchronous_velocity[3*n],s.velocity_error[3*n],v,v_error,wr::ImpactSpeed)||
       !check(s.contact.nodes[n].force.value,s.contact.nodes[n].force.error,force,force_error,m.force_scale()))return Fail(error,"Analytic spring comparison overflow");
    total_u+=u; total_j+=impulse;
    double next=0;
    if(!b::AddScalar(uncertainty_u,static_cast<double>(u_error+Roundoff*std::abs(u)),true,&next))return Fail(error,"Analytic potential uncertainty overflow");
    uncertainty_u=next;
    if(!b::AddScalar(uncertainty_j,static_cast<double>(j_error+Roundoff*std::abs(impulse)),true,&next))return Fail(error,"Analytic impulse uncertainty overflow");
    uncertainty_j=next;
  }
  if(!check(s.contact.potential.value,s.contact.potential.error,total_u,uncertainty_u,m.energy())||
     !check(s.synchronous_wall_impulse,s.synchronous_wall_impulse_error,total_j,uncertainty_j,2*m.momentum()))return Fail(error,"Analytic total comparison overflow");
  return true;
}
bool SameObservation(const Sample& a,const Sample& z) noexcept {
  auto same=[](Interval x,Interval y) { return x.lower==y.lower&&x.upper==y.upper; };
  return a.values==z.values&&a.errors==z.errors&&a.synchronous_velocity==z.synchronous_velocity&&
    a.synchronous_omega==z.synchronous_omega&&a.rotation_vector==z.rotation_vector&&a.velocity_error==z.velocity_error&&
    a.omega_error==z.omega_error&&a.endpoint_rhs==z.endpoint_rhs&&a.endpoint_couple==z.endpoint_couple&&
    a.endpoint_native_force_error==z.endpoint_native_force_error&&a.endpoint_native_couple_error==z.endpoint_native_couple_error&&a.endpoint_contact_force_error==z.endpoint_contact_force_error&&
    a.carried_kinetic==z.carried_kinetic&&a.synchronous_kinetic==z.synchronous_kinetic&&a.kinetic_error==z.kinetic_error&&a.source_work==z.source_work&&
    same(a.residual,z.residual)&&same(a.absolute_residual,z.absolute_residual)&&same(a.analytic_difference,z.analytic_difference)&&
    a.minimum_gap==z.minimum_gap&&a.maximum_gap==z.maximum_gap&&a.minimum_velocity==z.minimum_velocity&&a.maximum_velocity==z.maximum_velocity&&
    a.relative_displacement==z.relative_displacement&&a.rotation_angle==z.rotation_angle&&a.strain==z.strain&&a.thickness_curvature==z.thickness_curvature&&
    a.minimum_area_ratio==z.minimum_area_ratio&&a.maximum_area_ratio==z.maximum_area_ratio&&
    a.minimum_thickness_ratio==z.minimum_thickness_ratio&&a.maximum_thickness_ratio==z.maximum_thickness_ratio&&a.mask==z.mask&&
    a.synchronous_wall_impulse==z.synchronous_wall_impulse&&a.synchronous_wall_impulse_error==z.synchronous_wall_impulse_error;
}
} // namespace tl::qualification::qeph::wall_response::detail
