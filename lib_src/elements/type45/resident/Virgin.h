// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Type45Reference.h"

namespace tl::fea::type45::resident_detail {
struct VirginCache {
  HistoryValues history;
  EndpointResult endpoint[2]{};
};
// Selected zero/free profile before TT0 automatic assignment. This constructs
// no Reference and performs no material increment or simulated time advance.
TL_TYPE45_HD inline Status PrepareVirgin(const Property& property,const GeometryInput& geometry,
                                         VirginCache& output) {
  using namespace detail;
  if(!Valid(property)) return Status::InvalidProperty;
  for(unsigned dof=0;dof<6;++dof)
    if(Get(property.free_stiffness,dof)!=0 || Get(property.free_viscosity,dof)!=0)
      return Status::InvalidProperty;
  const unsigned count=property.kind==Kind::Spherical?2:3;
  if(!geometry.source_joint_id || (count==2 &&
      (geometry.source_node_id[2] || !Same(geometry.position_m[2],Vec3{})))) return Status::InvalidGeometry;
  for(unsigned i=0;i<count;++i) {
    if(!geometry.source_node_id[i] || !Finite(geometry.position_m[i])) return Status::InvalidGeometry;
    for(unsigned j=0;j<i;++j)
      if(geometry.source_node_id[i]==geometry.source_node_id[j]) return Status::InvalidGeometry;
  }
  VirginCache next;
  if(!InitialFrame(geometry,property,next.history.frame)) return Status::InvalidGeometry;
  const auto separation=ToLocal(next.history.frame,Subtract(geometry.position_m[1],geometry.position_m[0]));
  if(!Finite(separation) || !Finite(Dot(separation,separation))) return Status::NonfiniteResult;
  output=next;
  return Status::Success;
}
} // namespace tl::fea::type45::resident_detail
