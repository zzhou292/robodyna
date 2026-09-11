#include "NativeOracle.h"
namespace type13_recurrence_test {
namespace {
void Pack(t::Vec3 v,double* p){p[0]=v.x;p[1]=v.y;p[2]=v.z;}
t::Vec3 Vector(const double* p,double scale){return {p[0]*scale,p[1]*scale,p[2]*scale};}
}
t::Evaluation NativeEvaluate(const t::Property& property,const t::Reference& reference,const t::NativeHistory& old,
    const t::NativeEndpointKinematics (&nodes)[2],double dt,bool fresh) {
  double positions[6],velocities[6],spin[6],transverse[3],lengths[2],oldmotion[6],motion[6];
  for(unsigned k=0;k<2;++k){Pack(nodes[k].position,positions+3*k);Pack(nodes[k].velocity,velocities+3*k);Pack(nodes[k].angular_velocity,spin+3*k);}
  Pack(old.transverse_axis,transverse);
  for(unsigned k=0;k<6;++k)oldmotion[k]=old.channels[k].deformation;
  t::Evaluation next;int initial=fresh?1:0;
  double native_reference_length=0;
  type25_native_frame(positions,velocities,spin,transverse,&dt,next.native_frame.axes.v,next.native_frame.midpoint_axes.v,lengths);
  type13_native_deformation(next.native_frame.midpoint_axes.v,lengths,velocities,spin,oldmotion,&reference.length_native,&dt,&initial,motion,&native_reference_length);
  double limits[12],forces[6],work[6],k[6];
  const double active=old.active?1:0;
  for(unsigned c=0;c<6;++c) {
    const auto& channel=property.channel(c);const auto& accepted=old.channels[c];
    const double base[5]={accepted.deformation,accepted.accumulated_plastic_deformation,
        accepted.elastic_plastic_force,accepted.force,accepted.signed_work};
    double curve[10],result[5];int pos=static_cast<int>(accepted.curve_position),nextpos=0;
    for(unsigned i=0;i<5;++i){curve[2*i]=property.curve(channel.declaration.curve_index).points[i].x;
      curve[2*i+1]=property.curve(channel.declaration.curve_index).points[i].y;}
    type13_h1_channel(base,&motion[c],&channel.native_stiffness,&native_reference_length,&dt,&active,curve,&pos,result,&nextpos);
    next.native_history.channels[c]={result[0],result[1],result[2],result[3],result[4],static_cast<unsigned>(nextpos)};
    motion[c]=result[0];forces[c]=result[3];work[c]=result[4];k[c]=channel.native_stiffness;
    limits[2*c]=channel.declaration.failure_negative;limits[2*c+1]=channel.declaration.failure_positive;
  }
  double failure[2];type13_native_failure(motion,limits,&native_reference_length,&old.failure_criterion,&active,failure);
  next.native_history.failure_criterion=failure[0];next.native_history.active=failure[1]!=0;
  next.native_history.transverse_axis={next.native_frame.axes.v[1],next.native_frame.axes.v[4],next.native_frame.axes.v[7]};
  next.native_frame.length=lengths[0];next.native_frame.midpoint_length=lengths[1];
  next.newly_failed=old.active&&!next.native_history.active;
  double world[12];type25_native_scatter(next.native_frame.axes.v,&lengths[0],forces,forces+3,world);
  const auto u=property.units();const double force=u.mass_to_kg*u.length_to_m/(u.time_to_s*u.time_to_s),moment=force*u.length_to_m;
  for(unsigned i=0;i<2;++i)next.endpoints[i]={Vector(world+6*i,force),Vector(world+6*i+3,moment)};
  next.local_force_N=Vector(forces,force);next.local_couple_Nm=Vector(forces+3,moment);
  for(unsigned c=0;c<6;++c)next.signed_work_J[c]=work[c]*moment;
  next.total_signed_work_J=(work[0]+work[1]+work[2]+work[3]+work[4]+work[5])*moment;
  double stable[3];const double mass=property.mass_per_length(),inertia=property.inertia_per_length();
  type13_native_stability(&mass,&inertia,k,&native_reference_length,&lengths[0],stable);
  next.stability={stable[0]*u.time_to_s,stable[1]*force/u.length_to_m,stable[2]*moment};
  return next;
}
} // namespace type13_recurrence_test
