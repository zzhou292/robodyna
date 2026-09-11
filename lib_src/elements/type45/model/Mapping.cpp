// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"

namespace tl::fea::type45::model_detail {
ModelReport PrepareJoint(const NodalRigidAssemblyBinding& rigid,const JointInput& input,Joint& output) {
  using namespace detail;
  if(!Valid(input.property)) return Error(ModelStatus::InvalidInput,"Invalid joint scalar property");
  for(unsigned axis=0;axis<6;++axis)
    if(Get(input.property.free_stiffness,axis)!=0 || Get(input.property.free_viscosity,axis)!=0)
      return Error(ModelStatus::UnsupportedProfile,"First physical joint profile requires zero optional free K/C");
  const auto& geometry=input.geometry;
  if(!geometry.source_joint_id) return Error(ModelStatus::InvalidInput,"Missing joint source identity");
  const unsigned count=input.property.kind==Kind::Spherical?2:3;
  if(count==2 && (geometry.source_node_id[2] || !Same(geometry.position_m[2],Vec3{})))
    return Error(ModelStatus::InvalidInput,"Spherical unused third slot is not canonical");
  Joint next;
  next.property=input.property; next.geometry=geometry;
  for(unsigned slot=0;slot<count;++slot) {
    if(!geometry.source_node_id[slot] || !Finite(geometry.position_m[slot]))
      return Error(ModelStatus::InvalidInput,"Invalid joint source node",SIZE_MAX,slot);
    for(unsigned prior=0;prior<slot;++prior)
      if(geometry.source_node_id[prior]==geometry.source_node_id[slot])
        return Error(ModelStatus::InvalidInput,"Repeated consumed joint source node",SIZE_MAX,slot);
    const auto node=rigid.domain()->Find(geometry.source_node_id[slot]);
    if(node==SIZE_MAX || !Same(geometry.position_m[slot],rigid.domain()->nodes()[node].position))
      return Error(ModelStatus::SourceMismatch,"Joint source position/domain identity differs",SIZE_MAX,slot);
    next.domain_nodes[slot]=node;
    if(slot==2) continue; // Axis only: no force, coefficient or rigid membership contribution.
    const auto* member=rigid.FindMember(node);
    if(!member) return Error(ModelStatus::UnsupportedProfile,"Joint endpoint is not an actual rigid member",SIZE_MAX,slot);
    const auto offset=static_cast<std::size_t>(member-rigid.members().data());
    std::size_t group=0;
    while(group<rigid.groups().size() && offset>=rigid.groups()[group].member_offset+rigid.groups()[group].member_count)
      ++group;
    if(group==rigid.groups().size()) return Error(ModelStatus::SourceMismatch,"Joint member has no rigid group",SIZE_MAX,slot);
    const auto& body=rigid.groups()[group];
    if(!input.body[slot].source_id || input.body[slot].source_id!=body.source_id || input.body[slot].kind!=body.source_kind)
      return Error(ModelStatus::SourceMismatch,"Joint source body kind or identity differs",SIZE_MAX,slot);
    next.body_groups[slot]=group;
    // Exact RINI45_RB order: body M, then (J1+J2+J3)/3 for damping only.
    const auto j=body.principal.inertia;
    next.damping[slot]={body.mass_kg,(j.x+j.y+j.z)/3.};
    if(!Positive(next.damping[slot].mass_kg) || !Nonnegative(next.damping[slot].mean_principal_inertia_kg_m2))
      return Error(ModelStatus::InvalidInput,"Invalid initial rigid damping coefficients",SIZE_MAX,slot);
  }
  Matrix3 frame;
  if(!InitialFrame(geometry,input.property,frame))
    return Error(ModelStatus::InvalidInput,"Degenerate joint initial axis geometry");
  const auto separation=ToLocal(frame,Subtract(geometry.position_m[1],geometry.position_m[0]));
  if(!Finite(separation) || !Finite(Dot(separation,separation)))
    return Error(ModelStatus::InvalidInput,"Joint initial separation overflows");
  output=next;
  return {};
}
} // namespace tl::fea::type45::model_detail
