#include "WallRawJson.h"

namespace tl::qualification::qeph::wall_recurrence::raw_detail {
namespace {
bool Zero(contact::Q4CertifiedIntegral v) { return v.value==0&&v.lower==0&&v.upper==0&&v.error==0; }
bool Zero(contact::Vec3 v) { return v.x==0&&v.y==0&&v.z==0; }
}
io::Document DescribePoint(const contact::NodalWallPointResult& p) {
  io::Require(p.valid&&!p.fixed&&p.node<recurrence::MaxNodes&&p.row.valid&&p.row.count==1&&p.row.nodes[0]==p.node&&
    p.row.base_epoch==p.base_epoch&&p.row.attempt==p.attempt,"Invalid retained point record/row identity");
  io::Document d; d.SetObject(); io::Integer(d,"node",p.node); io::Boolean(d,"valid",p.valid);
  io::Boolean(d,"fixed",p.fixed); io::Boolean(d,"touching_or_penetrating",p.touching_or_penetrating);
  io::Integer(d,"base_epoch",p.base_epoch); io::Integer(d,"attempt",p.attempt);
  Field(d,"force_n",Certificate(p.force)); Field(d,"potential_j",Certificate(p.potential));
  Field(d,"stiffness_n_m",Certificate(p.stiffness)); Vec(d,"force_world_n",p.force_world);
  Vec(d,"wall_point_m",p.wall_point); Vec(d,"wall_reaction_n",p.wall_reaction); Vec(d,"wall_moment_n_m",p.wall_moment);
  io::Number(d,"surface_power_w",p.surface_power); io::Number(d,"raw_local_velocity_first_dt_s",p.local_velocity_first_timestep);
  io::Document row; row.SetObject(); io::Integer(row,"count",p.row.count); io::Boolean(row,"valid",p.row.valid);
  io::Integer(row,"base_epoch",p.row.base_epoch); io::Integer(row,"attempt",p.row.attempt);
  Indices(row,"nodes",p.row.nodes,p.row.count); io::FiniteArray(row,"stiffness_s_inverse_squared",p.row.stiffness,p.row.count);
  io::FiniteArray(row,"damping_s_inverse",p.row.damping,p.row.count); Field(d,"source_row_diagnostic",row); return d;
}
io::Document DescribeParent(const contact::NodalWallParentResult& p) {
  io::Require(p.valid&&p.arity==4&&p.family==contact::NodalWallParentFamily::Q4CenterArea,"Invalid retained Q4 parent result");
  io::Document d; d.SetObject(); io::Integer(d,"parent_element_id",p.parent_element_id);
  io::Integer(d,"feature_id",p.feature_id); io::Integer(d,"parent_face_id",p.parent_face_id);
  io::Integer(d,"arity",p.arity); io::String(d,"family","q4-center-area"); io::Boolean(d,"valid",p.valid);
  Field(d,"resultant_n",Certificate(p.resultant)); Field(d,"potential_j",Certificate(p.potential));
  io::Value forces(rapidjson::kArrayType);
  for(unsigned i=0;i<p.arity;++i) Append(d,forces,Certificate(p.force[i]));
  d.AddMember("force_n",forces,d.GetAllocator()); return d;
}
io::Document DescribeContactSample(const ContactMapSample& sample,unsigned cells,bool required) {
  const unsigned count=2*(cells+1),dimension=12*count+61*cells;
  const bool present=sample.state.size()!=0;
  io::Require(!required||present,"Missing completed contact/native sample");
  io::Document d; d.SetObject(); io::Boolean(d,"present",present);
  if(!present) {
    io::Require(sample.nodes.empty()&&sample.parents.empty()&&sample.base_epoch==0&&sample.attempt==0&&
      Zero(sample.resultant)&&Zero(sample.potential)&&Zero(sample.wall_reaction)&&Zero(sample.wall_moment)&&sample.surface_power==0,
                "Unpublished sample contains partial identity/records");
    return d;
  }
  io::Require(sample.state.size()==dimension&&sample.state.allFinite()&&sample.nodes.size()==count&&
    sample.parents.size()==cells&&sample.base_epoch==1&&sample.attempt==1,"Invalid completed raw sample shape/identity");
  d.AddMember("normalized_state",Vector(d,sample.state),d.GetAllocator());
  io::Integer(d,"base_epoch",sample.base_epoch); io::Integer(d,"attempt",sample.attempt);
  Field(d,"resultant_n",Certificate(sample.resultant)); Field(d,"potential_j",Certificate(sample.potential));
  Vec(d,"wall_reaction_n",sample.wall_reaction); Vec(d,"wall_moment_n_m",sample.wall_moment);
  io::Number(d,"surface_power_w",sample.surface_power); io::Value nodes(rapidjson::kArrayType),parents(rapidjson::kArrayType);
  for(unsigned n=0;n<count;++n) {
    const auto& p=sample.nodes[n];
    io::Require(p.node==n&&p.base_epoch==1&&p.attempt==1,"Changed point identity in raw sample"); Append(d,nodes,DescribePoint(p));
  }
  for(unsigned p=0;p<cells;++p) {
    io::Require(sample.parents[p].parent_element_id==1001+p&&sample.parents[p].feature_id==2001+p&&
      sample.parents[p].parent_face_id==0,"Changed parent identity in raw sample");
    Append(d,parents,DescribeParent(sample.parents[p]));
  }
  d.AddMember("nodes",nodes,d.GetAllocator()); d.AddMember("parents",parents,d.GetAllocator()); return d;
}
} // namespace tl::qualification::qeph::wall_recurrence::raw_detail
