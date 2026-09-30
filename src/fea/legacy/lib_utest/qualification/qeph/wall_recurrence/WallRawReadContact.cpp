#include "WallRawReadFields.h"

namespace tl::qualification::qeph::wall_recurrence::read_detail {
namespace {
contact::NodalWallPointResult Point(const io::Value& v) {
  contact::NodalWallPointResult p;
  p.node=Count(Field(v,"node"),recurrence::MaxNodes-1);
  p.valid=Boolean(Field(v,"valid")); p.fixed=Boolean(Field(v,"fixed"));
  p.touching_or_penetrating=Boolean(Field(v,"touching_or_penetrating"));
  p.base_epoch=Unsigned(Field(v,"base_epoch")); p.attempt=Unsigned(Field(v,"attempt"));
  p.force=Certificate(Field(v,"force_n")); p.potential=Certificate(Field(v,"potential_j"));
  p.stiffness=Certificate(Field(v,"stiffness_n_m"));
  p.force_world=Vec<contact::Vec3>(Field(v,"force_world_n")); p.wall_point=Vec<contact::Vec3>(Field(v,"wall_point_m"));
  p.wall_reaction=Vec<contact::Vec3>(Field(v,"wall_reaction_n")); p.wall_moment=Vec<contact::Vec3>(Field(v,"wall_moment_n_m"));
  p.surface_power=Number(Field(v,"surface_power_w")); p.local_velocity_first_timestep=Number(Field(v,"raw_local_velocity_first_dt_s"));
  const auto& row=Field(v,"source_row_diagnostic");
  p.row.count=Count(Field(row,"count"),1); io::Require(p.row.count==1,"Invalid used raw row count");
  p.row.valid=Boolean(Field(row,"valid")); p.row.base_epoch=Unsigned(Field(row,"base_epoch"));
  p.row.attempt=Unsigned(Field(row,"attempt"));
  const auto& nodes=Field(row,"nodes"); const auto& stiffness=Field(row,"stiffness_s_inverse_squared");
  const auto& damping=Field(row,"damping_s_inverse");
  io::Require(nodes.IsArray()&&nodes.Size()==1&&stiffness.IsArray()&&stiffness.Size()==1&&damping.IsArray()&&damping.Size()==1,
              "Invalid raw row arrays");
  p.row.nodes[0]=Count(nodes[0],recurrence::MaxNodes-1); p.row.stiffness[0]=Number(stiffness[0]); p.row.damping[0]=Number(damping[0]);
  io::Require(Same(raw_detail::DescribePoint(p),v),"Raw point fields fail owning round trip"); return p;
}
contact::NodalWallParentResult Parent(const io::Value& v) {
  contact::NodalWallParentResult p;
  p.parent_element_id=Unsigned(Field(v,"parent_element_id")); p.feature_id=Unsigned(Field(v,"feature_id"));
  p.parent_face_id=Count(Field(v,"parent_face_id"),UINT32_MAX); p.arity=Count(Field(v,"arity"),4);
  io::Require(Text(Field(v,"family"))=="q4-center-area"&&p.arity==4,"Invalid raw parent family/arity");
  p.family=contact::NodalWallParentFamily::Q4CenterArea; p.valid=Boolean(Field(v,"valid"));
  p.resultant=Certificate(Field(v,"resultant_n")); p.potential=Certificate(Field(v,"potential_j"));
  const auto& forces=Field(v,"force_n"); io::Require(forces.IsArray()&&forces.Size()==4,"Invalid raw parent force count");
  for(unsigned i=0;i<4;++i) p.force[i]=Certificate(forces[i]);
  io::Require(Same(raw_detail::DescribeParent(p),v),"Raw parent fields fail owning round trip"); return p;
}
}
ContactMapSample Sample(const io::Value& v,unsigned cells,bool required) {
  ContactMapSample out;
  if(!Boolean(Field(v,"present"))) {
    io::Require(!required,"Missing completed physical sample"); Keys(v,{"present"}); return out;
  }
  out.state=Vector(Field(v,"normalized_state")); out.base_epoch=Unsigned(Field(v,"base_epoch")); out.attempt=Unsigned(Field(v,"attempt"));
  out.resultant=Certificate(Field(v,"resultant_n")); out.potential=Certificate(Field(v,"potential_j"));
  out.wall_reaction=Vec<contact::Vec3>(Field(v,"wall_reaction_n")); out.wall_moment=Vec<contact::Vec3>(Field(v,"wall_moment_n_m"));
  out.surface_power=Number(Field(v,"surface_power_w"));
  const auto& nodes=Field(v,"nodes"); const auto& parents=Field(v,"parents");
  io::Require(nodes.IsArray()&&nodes.Size()==2*(cells+1)&&parents.IsArray()&&parents.Size()==cells,"Invalid raw physical sample counts");
  for(const auto& node:nodes.GetArray()) out.nodes.push_back(Point(node));
  for(const auto& parent:parents.GetArray()) out.parents.push_back(Parent(parent));
  io::Require(Same(raw_detail::DescribeContactSample(out,cells,required),v),"Raw physical sample fails owning round trip"); return out;
}
} // namespace tl::qualification::qeph::wall_recurrence::read_detail
