#include "WallResponseInternal.h"
#include "../response/ResponseFieldVisitor.h"
#include "lib_src/elements/qeph/QephHistory.h"
#include "lib_src/math/Quaternion.h"
#include <utility>

namespace tl::qualification::qeph::wall_response {
namespace d=detail;
namespace b=contact::q4_bounds;
namespace {
bool Certificate(const contact::Q4CertifiedIntegral& x) {
  return contact::nodal_wall_detail::Certificate(x);
}
}
bool ObserveEndpoint(const Model& m,const Config& config,const EndpointInput& input,Sample& output,std::string& error) {
  if(!m.prepared()||!ValidConfig(config)||m.fields().cells!=config.cells) return d::Fail(error,"Invalid wall response model/configuration");
  for(unsigned e=0;e<config.cells;++e) {
    const auto& history=input.elements[e].proposed_history;
    if(!history.prepared()||!history.matches_reference(m.fields().reference[e])||
       history.stamp().sample_index!=input.epoch||history.stamp().time!=input.time)
      return d::Fail(error,"Endpoint native history/reference/phase mismatch");
  }
  Sample s; s.epoch=input.epoch; s.time=input.time; s.carried_velocity_time=input.carried_velocity_time;
  s.kick_dt=input.kick_dt; s.state=input.state; s.contact=input.contact;
  s.wall_impulse=input.wall_impulse; s.wall_impulse_error=input.wall_impulse_error;
  unsigned count=0;
  free_response::detail::Visit(m.fields(),s.state,input.elements,s.synchronous_velocity,s.synchronous_omega,s.rotation_vector,
    [&](const char*,unsigned,unsigned,double value,const char*,double,bool) {
      if(count<MaxFields) s.values[count]=value; ++count;
    });
  if(count!=m.native_field_count()||!d::Reconstruct(m,config,s,error)) return false;
  output=s; error.clear(); return true;
}
bool ValidateSample(const Model& m,const Config& config,const Sample& sample,std::string& error) {
  Sample expected=sample;
  if(!d::Reconstruct(m,config,expected,error)) return false;
  if(!d::SameObservation(sample,expected)) return d::Fail(error,"Retained endpoint fields do not reproduce observation arithmetic");
  error.clear(); return true;
}
namespace detail {
bool Reconstruct(const Model& m,const Config& config,Sample& s,std::string& error) {
  if(!m.prepared()||!ValidConfig(config)||config.cells!=m.fields().cells) return Fail(error,"Unprepared observation model");
  const auto& f=m.fields(); const double h=Step(config);
  if(s.epoch>Steps(config)||s.time!=s.epoch*h||s.carried_velocity_time!=(s.epoch?s.time-h/2:0)||
     s.kick_dt!=(s.epoch?(s.epoch==1?h/2:h):0)||s.contact.node_count!=f.nodes||!s.contact.attempt||
     s.contact.candidate!=(s.epoch!=0)||s.contact.base_epoch!=(s.epoch?s.epoch-1:0)) return Fail(error,"Endpoint/contact phase mismatch");
  if(!Certificate(s.contact.resultant)||!Certificate(s.contact.potential)||
     !std::isfinite(s.wall_impulse)||s.wall_impulse<0||!std::isfinite(s.wall_impulse_error)||s.wall_impulse_error<0)
    return Fail(error,"Invalid contact/impulse certificate");
  for(unsigned i=0;i<m.native_field_count();++i) if(!std::isfinite(s.values[i])) return Fail(error,"Nonfinite native field");
  s.errors.fill(0); s.endpoint_rhs.fill(0); s.endpoint_couple.fill(0);
  s.endpoint_native_force_error.fill(0);s.endpoint_native_couple_error.fill(0);s.endpoint_contact_force_error.fill(0);
  s.carried_kinetic.fill(0); s.synchronous_kinetic.fill(0); s.kinetic_error.fill(0); s.source_work.fill(0);
  s.relative_displacement=s.rotation_angle=s.strain=s.thickness_curvature=0;
  s.minimum_area_ratio=s.maximum_area_ratio=s.minimum_thickness_ratio=s.maximum_thickness_ratio=1;
  std::array<Interval,18> rhs{},torque{}; std::array<long double,18> rhs_value{},torque_value{};
  long double source[3]{}; Interval source_interval{};
  for(unsigned e=0;e<f.cells;++e) {
    const auto& offsets=m.element_fields(e);
    double length=0;
    const auto first=f.connectivity[e][0];
    for(unsigned i=1;i<4;++i) {
      const auto n=f.connectivity[e][i];
      const double dx=s.state.x[3*n]-s.state.x[3*first],dy=s.state.x[3*n+1]-s.state.x[3*first+1],dz=s.state.x[3*n+2]-s.state.x[3*first+2];
      length=std::max(length,std::hypot(std::hypot(dx,dy),dz));
    }
    if(!std::isfinite(length)||length<=0)return Fail(error,"Invalid source force length scale");
    const long double force_scale=static_cast<long double>(f.reference[e].input.young_modulus)*f.reference[e].input.thickness*length;
    const long double couple_scale=force_scale*length;
    if(s.values[offsets.active]!=1||s.values[offsets.thickness]<=0) return Fail(error,"Invalid native activity/thickness");
    const double area=s.epoch?s.values[offsets.geometry]/f.reference[e].area:1;
    const double thickness=s.values[offsets.thickness]/f.reference[e].input.thickness;
    if(!std::isfinite(area)||area<=0||!std::isfinite(thickness)||thickness<=0) return Fail(error,"Invalid endpoint area/thickness");
    s.minimum_area_ratio=std::min(s.minimum_area_ratio,area); s.maximum_area_ratio=std::max(s.maximum_area_ratio,area);
    s.minimum_thickness_ratio=std::min(s.minimum_thickness_ratio,thickness); s.maximum_thickness_ratio=std::max(s.maximum_thickness_ratio,thickness);
    for(unsigned i=0;i<8;++i) {
      if(i<5)s.strain=std::max(s.strain,std::abs(s.values[offsets.strain+i]));
      else s.thickness_curvature=std::max(s.thickness_curvature,f.reference[e].input.thickness*std::abs(s.values[offsets.strain+i]));
    }
    const double work[]{s.values[offsets.work],s.values[offsets.work+1],s.values[offsets.viscous]};
    for(unsigned i=0;i<3;++i) { source[i]+=work[i]; if(!b::Add(source_interval,{work[i],work[i]},&source_interval))return Fail(error,"Source-work sum overflow"); }
    for(unsigned i=0;i<4;++i) for(unsigned a=0;a<3;++a) {
      const auto n=f.connectivity[e][i],j=3*n+a;
      // The owning field visitor emits force XYZ then couple XYZ for EACH
      // local node; each family's next node is six fields later.
      const double force=-s.values[offsets.force+6*i+a],couple=-s.values[offsets.couple+6*i+a];
      double force_error=0,couple_error=0;
      if(!b::Round(static_cast<double>(2e-12L*(force_scale+std::abs(force))),true,&force_error)||
         !b::Round(static_cast<double>(2e-12L*(couple_scale+std::abs(couple))),true,&couple_error)||
         !b::AddScalar(s.endpoint_native_force_error[j],force_error,true,&s.endpoint_native_force_error[j])||
         !b::AddScalar(s.endpoint_native_couple_error[j],couple_error,true,&s.endpoint_native_couple_error[j]))return Fail(error,"Native/scatter allowance overflow");
      s.errors[offsets.force+6*i+a]=force_error;s.errors[offsets.couple+6*i+a]=couple_error;
      rhs_value[j]+=force; torque_value[j]+=couple;
      Interval force_range,couple_range;
      if(!Enclose(force,force_error,force_range)||!Enclose(couple,couple_error,couple_range)||
         !b::Add(rhs[j],force_range,&rhs[j])||!b::Add(torque[j],couple_range,&torque[j])) return Fail(error,"Endpoint cache reduction overflow");
    }
  }
  Interval kinetic[4]{}; long double carried[4]{},sync[4]{};
  s.mask=0; s.minimum_gap=INFINITY; s.maximum_gap=-INFINITY;
  s.minimum_velocity=INFINITY; s.maximum_velocity=-INFINITY;
  const long double translation=static_cast<long double>(s.state.x[0])-f.initial_position[0];
  for(unsigned n=0;n<f.nodes;++n) {
    const auto& p=s.contact.nodes[n]; const auto& offsets=m.node_fields(n);
    if(!p.valid||p.fixed||p.node!=n||p.base_epoch!=s.contact.base_epoch||p.attempt!=s.contact.attempt||
       !Certificate(p.force)||!Certificate(p.potential)||!Certificate(p.stiffness)||p.force_world.x!=-p.force.value||
       p.force_world.y!=0||p.force_world.z!=0) return Fail(error,"Endpoint nodal contact identity/value mismatch");
    if(!p.row.valid||p.row.count!=1||p.row.nodes[0]!=n||p.row.base_epoch!=p.base_epoch||p.row.attempt!=p.attempt||
       !std::isfinite(p.row.stiffness[0])||p.row.stiffness[0]<=0||p.row.damping[0]!=0||!std::isfinite(p.surface_power)||
       p.local_velocity_first_timestep!=0)return Fail(error,"Invalid retained contact row/power");
    for(unsigned i=1;i<tl::fea::stability::MaxNodes;++i)
      if(p.row.nodes[i]||p.row.stiffness[i]!=0||p.row.damping[i]!=0)return Fail(error,"Nonzero unused contact-row tail");
    const auto& stiffness=m.screened().touching().nodes[n].stiffness;
    if(p.stiffness.value!=stiffness.value||p.stiffness.lower!=stiffness.lower||p.stiffness.upper!=stiffness.upper||p.stiffness.error!=stiffness.error)
      return Fail(error,"Changed immutable nodal contact stiffness");
    const double gap=s.state.x[3*n]-m.screened().law().wall_x;
    if(!std::isfinite(gap)||p.touching_or_penetrating!=(gap>=0)||gap>wr::MaximumDepth) return Fail(error,"Contact mask/depth mismatch");
    if(p.touching_or_penetrating)s.mask|=1u<<n;
    s.minimum_gap=std::min(s.minimum_gap,gap); s.maximum_gap=std::max(s.maximum_gap,gap);
    rhs_value[3*n]+=p.force_world.x;
    s.endpoint_contact_force_error[3*n]=p.force.error;
    if(!b::Add(rhs[3*n],{-p.force.upper,-p.force.lower},&rhs[3*n])) return Fail(error,"Total endpoint RHS overflow");
    const auto* q=s.state.q.data()+4*n;
    if(!tl::math::UnitQuaternion({q[0],q[1],q[2],q[3]})) return Fail(error,"Invalid endpoint quaternion");
    const double length=std::hypot(std::hypot(q[1],q[2]),q[3]);
    const double angle=2*std::atan2(length,std::abs(q[0]))*(q[0]<0?-1.:1.);
    s.rotation_angle=std::max(s.rotation_angle,std::abs(angle));
    long double relative_square=0;
    for(unsigned a=0;a<4;++a) if(s.values[offsets.q+a]!=q[a])return Fail(error,"Raw quaternion field mismatch");
    for(unsigned a=0;a<3;++a) {
      const auto j=3*n+a;
      if(!std::isfinite(s.state.x[j])||!std::isfinite(s.state.v[j])||!std::isfinite(s.state.omega[j])||
         s.values[offsets.x+a]!=s.state.x[j]||s.values[offsets.v+a]!=s.state.v[j]||s.values[offsets.w+a]!=s.state.omega[j])return Fail(error,"Raw nodal field mismatch");
      const long double relative=static_cast<long double>(s.state.x[j])-f.initial_position[j]-(a?0:translation);
      relative_square+=relative*relative;
      s.endpoint_rhs[j]=static_cast<double>(rhs_value[j]); s.endpoint_couple[j]=static_cast<double>(torque_value[j]);
      Interval vi{s.state.v[j],s.state.v[j]},wi{s.state.omega[j],s.state.omega[j]},increment;
      s.synchronous_velocity[j]=s.state.v[j]; s.synchronous_omega[j]=s.state.omega[j];
      if(s.epoch) {
        if(!b::Scale(rhs[j],h/2,&increment)||!DivideSigned(increment,f.mass[n],increment)||!b::Add(vi,increment,&vi)||
           !b::Scale(torque[j],h/2,&increment)||!DivideSigned(increment,f.inertia[n],increment)||!b::Add(wi,increment,&wi)) return Fail(error,"Endpoint reconstruction overflow");
        s.synchronous_velocity[j]=static_cast<double>(s.state.v[j]+.5L*h*rhs_value[j]/f.mass[n]);
        s.synchronous_omega[j]=static_cast<double>(s.state.omega[j]+.5L*h*torque_value[j]/f.inertia[n]);
      }
      if(!Radius(s.synchronous_velocity[j],vi,s.velocity_error[j])||!Radius(s.synchronous_omega[j],wi,s.omega_error[j]))return Fail(error,"Endpoint velocity uncertainty overflow");
      const double inertias[]{f.inertia[n],f.physical[n],f.added[n]};
      Interval square,term;
      if(!Square(vi,square)||!b::Scale(square,.5*f.mass[n],&term)||!b::Add(kinetic[0],term,&kinetic[0]))return Fail(error,"Translation kinetic overflow");
      carried[0]+=.5L*f.mass[n]*s.state.v[j]*s.state.v[j];
      sync[0]+=.5L*f.mass[n]*s.synchronous_velocity[j]*s.synchronous_velocity[j];
      for(unsigned k=0;k<3;++k) {
        if(!Square(wi,square)||!b::Scale(square,.5*inertias[k],&term)||!b::Add(kinetic[k+1],term,&kinetic[k+1]))return Fail(error,"Rotation kinetic overflow");
        carried[k+1]+=.5L*inertias[k]*s.state.omega[j]*s.state.omega[j];
        sync[k+1]+=.5L*inertias[k]*s.synchronous_omega[j]*s.synchronous_omega[j];
      }
      s.rotation_vector[j]=length?angle*q[a+1]/length:0;
      s.values[offsets.vs+a]=s.synchronous_velocity[j]; s.errors[offsets.vs+a]=s.velocity_error[j];
      s.values[offsets.ws+a]=s.synchronous_omega[j]; s.errors[offsets.ws+a]=s.omega_error[j];
      s.values[offsets.angle+a]=s.rotation_vector[j];
    }
    s.relative_displacement=std::max(s.relative_displacement,static_cast<double>(std::sqrt(relative_square)));
    s.minimum_velocity=std::min(s.minimum_velocity,s.synchronous_velocity[3*n]-s.velocity_error[3*n]);
    s.maximum_velocity=std::max(s.maximum_velocity,s.synchronous_velocity[3*n]+s.velocity_error[3*n]);
  }
  for(unsigned k=0;k<4;++k) {
    s.carried_kinetic[k]=static_cast<double>(carried[k]); s.synchronous_kinetic[k]=static_cast<double>(sync[k]);
    if(!std::isfinite(s.carried_kinetic[k])||!Radius(s.synchronous_kinetic[k],kinetic[k],s.kinetic_error[k]))return Fail(error,"Kinetic output overflow");
  }
  for(unsigned k=0;k<3;++k)s.source_work[k]=static_cast<double>(source[k]);
  Interval total;
  if(!b::Add(kinetic[0],kinetic[1],&total)||!b::Add(total,source_interval,&total)||
     !b::Add(total,{s.contact.potential.lower,s.contact.potential.upper},&total)||
     !b::Add(total,{-m.energy(),-m.energy()},&s.residual))return Fail(error,"Energy residual overflow");
  s.absolute_residual=Magnitude(s.residual);
  Interval impulse,correction{};
  if(!Enclose(s.wall_impulse,s.wall_impulse_error,impulse)||
     (s.epoch&&!b::Scale({s.contact.resultant.lower,s.contact.resultant.upper},h/2,&correction))||
     !b::Add(impulse,correction,&impulse))return Fail(error,"Synchronous impulse overflow");
  s.synchronous_wall_impulse=static_cast<double>(static_cast<long double>(s.wall_impulse)+(s.epoch?.5L*h*s.contact.resultant.value:0));
  if(!Radius(s.synchronous_wall_impulse,impulse,s.synchronous_wall_impulse_error))return Fail(error,"Synchronous impulse radius overflow");
  unsigned i=m.native_field_count();
  for(unsigned n=0;n<f.nodes;++n) {s.values[i]=s.contact.nodes[n].force.value;s.errors[i++]=s.contact.nodes[n].force.error;}
  const double values[]{s.contact.resultant.value,s.contact.potential.value,s.wall_impulse,s.synchronous_wall_impulse};
  const double errors[]{s.contact.resultant.error,s.contact.potential.error,s.wall_impulse_error,s.synchronous_wall_impulse_error};
  for(unsigned k=0;k<4;++k) {s.values[i]=values[k];s.errors[i++]=errors[k];}
  for(;i<MaxFields;++i) {s.values[i]=0;s.errors[i]=0;}
  if(!s.epoch) {
    if(s.state.x!=f.initial_position||s.wall_impulse!=0||s.wall_impulse_error!=0||s.contact.resultant.upper!=0||s.contact.potential.upper!=0)
      return Fail(error,"Initial position/contact/impulse is not the declared separated startup");
    for(unsigned n=0;n<f.nodes;++n) {
      for(unsigned a=0;a<3;++a)if(s.state.v[3*n+a]!=(a?0:wr::ImpactSpeed)||s.state.omega[3*n+a]!=0)return Fail(error,"Initial physical velocity changed");
      if(s.state.q[4*n]!=1||s.state.q[4*n+1]!=0||s.state.q[4*n+2]!=0||s.state.q[4*n+3]!=0)return Fail(error,"Initial orientation changed");
    }
    for(unsigned e=0;e<f.cells;++e) {
      const auto& o=m.element_fields(e);
      for(const auto range:{std::pair<unsigned,unsigned>{o.stress,5},{o.material,5},{o.bending,3},{o.hourglass,12},{o.strain,8},{o.work,2},{o.viscous,1}})
        for(unsigned k=0;k<range.second;++k)if(s.values[range.first+k]!=0)return Fail(error,"Initial source history/cache is not zero");
      for(unsigned i=0;i<4;++i)for(unsigned a=0;a<3;++a)
        if(s.values[o.force+6*i+a]!=0||s.values[o.couple+6*i+a]!=0)return Fail(error,"Initial source history/cache is not zero");
      if(s.values[o.thickness]!=f.reference[e].input.thickness)return Fail(error,"Initial thickness changed");
    }
  }
  if(!Analytic(m,config,s,error))return false;
  error.clear(); return true;
}
} // namespace detail
} // namespace tl::qualification::qeph::wall_response
