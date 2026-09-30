#include "WallResponseJson.h"

namespace tl::qualification::qeph::wall_response::json {
namespace {
constexpr NumberField<Sample> Numbers[]{
  {"time_s",&Sample::time},{"carried_velocity_time_s",&Sample::carried_velocity_time},
  {"completed_kick_dt_s",&Sample::kick_dt},{"wall_impulse_N_s",&Sample::wall_impulse},
  {"wall_impulse_error_N_s",&Sample::wall_impulse_error},
  {"synchronous_wall_impulse_N_s",&Sample::synchronous_wall_impulse},
  {"synchronous_wall_impulse_error_N_s",&Sample::synchronous_wall_impulse_error},
  {"minimum_signed_penetration_m",&Sample::minimum_gap},{"maximum_signed_penetration_m",&Sample::maximum_gap},
  {"minimum_sync_normal_velocity_m_s",&Sample::minimum_velocity},{"maximum_sync_normal_velocity_m_s",&Sample::maximum_velocity},
  {"relative_deformation_m",&Sample::relative_displacement},{"rotation_angle_rad",&Sample::rotation_angle},
  {"strain",&Sample::strain},{"thickness_curvature",&Sample::thickness_curvature},
  {"minimum_area_ratio",&Sample::minimum_area_ratio},{"maximum_area_ratio",&Sample::maximum_area_ratio},
  {"minimum_thickness_ratio",&Sample::minimum_thickness_ratio},{"maximum_thickness_ratio",&Sample::maximum_thickness_ratio}};
void Vec(io::Document& d,const char* key,contact::Vec3 v) {
  const double values[]{v.x,v.y,v.z}; io::FiniteArray(d,key,values,3);
}
contact::Vec3 ReadVec(const io::Value& d,const char* key) {
  double v[3]; p::ReadArray(d,key,v,3); return {v[0],v[1],v[2]};
}
io::Document Row(const tl::fea::stability::RowContribution& row) {
  io::Require(row.count<=tl::fea::stability::MaxNodes,"Invalid contact row capacity");
  io::Document d; d.SetObject(); io::Integer(d,"base_epoch",row.base_epoch);
  io::Integer(d,"attempt",row.attempt); io::Boolean(d,"valid",row.valid);
  io::Integer(d,"count",row.count); io::Value nodes(rapidjson::kArrayType);
  for(unsigned i=0;i<row.count;++i) nodes.PushBack(row.nodes[i],d.GetAllocator());
  d.AddMember("nodes",nodes,d.GetAllocator());
  io::FiniteArray(d,"stiffness_s2",row.stiffness,row.count);
  io::FiniteArray(d,"damping_s1",row.damping,row.count); return d;
}
tl::fea::stability::RowContribution ReadRow(const io::Value& d) {
  tl::fea::stability::RowContribution row;
  row.count=Count(d,"count",tl::fea::stability::MaxNodes);
  row.base_epoch=p::Integer(d,"base_epoch"); row.attempt=p::Integer(d,"attempt"); row.valid=p::Boolean(d,"valid");
  const auto& nodes=p::Array(d,"nodes",row.count);
  for(unsigned i=0;i<row.count;++i) {
    io::Require(nodes[i].IsUint(),"Invalid contact row node"); row.nodes[i]=nodes[i].GetUint();
  }
  p::ReadArray(d,"stiffness_s2",row.stiffness,row.count); p::ReadArray(d,"damping_s1",row.damping,row.count);
  return row;
}
io::Document Contact(const ContactState& c,unsigned nodes) {
  io::Require(c.node_count==nodes,"Contact owner node count mismatch");
  io::Document d; d.SetObject(); io::Integer(d,"base_epoch",c.base_epoch); io::Integer(d,"attempt",c.attempt);
  io::Integer(d,"node_count",c.node_count); io::Boolean(d,"candidate",c.candidate);
  p::Object(d,"resultant_N",raw::Certificate(c.resultant)); p::Object(d,"potential_J",raw::Certificate(c.potential));
  io::Value points(rapidjson::kArrayType);
  for(unsigned i=0;i<nodes;++i) {
    const auto& n=c.nodes[i]; io::Document v; v.SetObject();
    io::Integer(v,"node",n.node); io::Integer(v,"base_epoch",n.base_epoch); io::Integer(v,"attempt",n.attempt);
    io::Boolean(v,"fixed",n.fixed); io::Boolean(v,"touching_or_penetrating",n.touching_or_penetrating);
    io::Boolean(v,"valid",n.valid); p::Object(v,"force_N",raw::Certificate(n.force));
    p::Object(v,"potential_J",raw::Certificate(n.potential)); p::Object(v,"stiffness_N_m",raw::Certificate(n.stiffness));
    Vec(v,"force_world_N",n.force_world); Vec(v,"wall_point_m",n.wall_point);
    Vec(v,"wall_reaction_N",n.wall_reaction); Vec(v,"wall_moment_N_m",n.wall_moment);
    io::Number(v,"surface_power_W",n.surface_power); io::Number(v,"local_velocity_first_timestep_s",n.local_velocity_first_timestep);
    p::Object(v,"diagnostic_row",Row(n.row)); p::Push(d,points,v);
  }
  d.AddMember("nodes",points,d.GetAllocator()); return d;
}
ContactState ReadContact(const io::Value& d,unsigned nodes) {
  ContactState c; c.base_epoch=p::Integer(d,"base_epoch"); c.attempt=p::Integer(d,"attempt");
  c.node_count=Count(d,"node_count",MaxNodes); io::Require(c.node_count==nodes,"Contact owner count mismatch");
  c.candidate=p::Boolean(d,"candidate"); c.resultant=read::Certificate(p::Member(d,"resultant_N"));
  c.potential=read::Certificate(p::Member(d,"potential_J")); const auto& points=p::Array(d,"nodes",nodes);
  for(unsigned i=0;i<nodes;++i) {
    const auto& v=points[i]; auto& n=c.nodes[i]; n.node=Count(v,"node",MaxNodes-1);
    n.base_epoch=p::Integer(v,"base_epoch"); n.attempt=p::Integer(v,"attempt");
    n.fixed=p::Boolean(v,"fixed"); n.touching_or_penetrating=p::Boolean(v,"touching_or_penetrating");
    n.valid=p::Boolean(v,"valid"); n.force=read::Certificate(p::Member(v,"force_N"));
    n.potential=read::Certificate(p::Member(v,"potential_J")); n.stiffness=read::Certificate(p::Member(v,"stiffness_N_m"));
    n.force_world=ReadVec(v,"force_world_N"); n.wall_point=ReadVec(v,"wall_point_m");
    n.wall_reaction=ReadVec(v,"wall_reaction_N"); n.wall_moment=ReadVec(v,"wall_moment_N_m");
    n.surface_power=p::Number(v,"surface_power_W"); n.local_velocity_first_timestep=p::Number(v,"local_velocity_first_timestep_s");
    n.row=ReadRow(p::Member(v,"diagnostic_row"));
  }
  return c;
}
}
io::Document SampleReport(const Sample& s,const Model& m) {
  const auto n=m.fields().nodes; io::Document d; d.SetObject(); io::Integer(d,"epoch",s.epoch);
  io::Integer(d,"active_mask",s.mask); WriteNumbers(d,s,Numbers);
  Array(d,"position_xyz_m",s.state.x,3*n); Array(d,"quaternion_wxyz",s.state.q,4*n);
  Array(d,"carried_velocity_m_s",s.state.v,3*n); Array(d,"carried_omega_rad_s",s.state.omega,3*n);
  Array(d,"synchronous_velocity_m_s",s.synchronous_velocity,3*n); Array(d,"synchronous_omega_rad_s",s.synchronous_omega,3*n);
  Array(d,"rotation_vector_rad",s.rotation_vector,3*n); Array(d,"velocity_error_m_s",s.velocity_error,3*n);
  Array(d,"omega_error_rad_s",s.omega_error,3*n); Array(d,"endpoint_rhs_N",s.endpoint_rhs,3*n);
  Array(d,"endpoint_couple_N_m",s.endpoint_couple,3*n);
  Array(d,"endpoint_native_force_error_N",s.endpoint_native_force_error,3*n);
  Array(d,"endpoint_native_couple_error_N_m",s.endpoint_native_couple_error,3*n);
  Array(d,"endpoint_contact_force_error_N",s.endpoint_contact_force_error,3*n);
  Array(d,"carried_kinetic_J",s.carried_kinetic); Array(d,"synchronous_kinetic_J",s.synchronous_kinetic);
  Array(d,"kinetic_error_J",s.kinetic_error); Array(d,"source_work_EINT0_EINT1_EVIS_J",s.source_work);
  Array(d,"values",s.values,m.dictionary().size()); Array(d,"absolute_errors",s.errors,m.dictionary().size());
  IntervalField(d,"residual_J",s.residual); IntervalField(d,"absolute_residual_J",s.absolute_residual);
  IntervalField(d,"normalized_analytic_difference",s.analytic_difference); p::Object(d,"contact",Contact(s.contact,n));
  return d;
}
Sample ReadSample(const io::Value& d,const Model& m) {
  const auto n=m.fields().nodes; Sample s; s.epoch=p::Integer(d,"epoch"); s.mask=Count(d,"active_mask",(1u<<n)-1);
  ReadNumbers(d,s,Numbers); ReadArray(d,"position_xyz_m",s.state.x,3*n); ReadArray(d,"quaternion_wxyz",s.state.q,4*n);
  ReadArray(d,"carried_velocity_m_s",s.state.v,3*n); ReadArray(d,"carried_omega_rad_s",s.state.omega,3*n);
  ReadArray(d,"synchronous_velocity_m_s",s.synchronous_velocity,3*n); ReadArray(d,"synchronous_omega_rad_s",s.synchronous_omega,3*n);
  ReadArray(d,"rotation_vector_rad",s.rotation_vector,3*n); ReadArray(d,"velocity_error_m_s",s.velocity_error,3*n);
  ReadArray(d,"omega_error_rad_s",s.omega_error,3*n); ReadArray(d,"endpoint_rhs_N",s.endpoint_rhs,3*n);
  ReadArray(d,"endpoint_couple_N_m",s.endpoint_couple,3*n);
  ReadArray(d,"endpoint_native_force_error_N",s.endpoint_native_force_error,3*n);
  ReadArray(d,"endpoint_native_couple_error_N_m",s.endpoint_native_couple_error,3*n);
  ReadArray(d,"endpoint_contact_force_error_N",s.endpoint_contact_force_error,3*n);
  ReadArray(d,"carried_kinetic_J",s.carried_kinetic); ReadArray(d,"synchronous_kinetic_J",s.synchronous_kinetic);
  ReadArray(d,"kinetic_error_J",s.kinetic_error); ReadArray(d,"source_work_EINT0_EINT1_EVIS_J",s.source_work);
  ReadArray(d,"values",s.values,m.dictionary().size()); ReadArray(d,"absolute_errors",s.errors,m.dictionary().size());
  s.residual=ReadInterval(d,"residual_J"); s.absolute_residual=ReadInterval(d,"absolute_residual_J");
  s.analytic_difference=ReadInterval(d,"normalized_analytic_difference"); s.contact=ReadContact(p::Member(d,"contact"),n);
  return s;
}
} // namespace tl::qualification::qeph::wall_response::json
