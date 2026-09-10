// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalRigidAssemblyInternal.h"

namespace tl::fea::rigid {
AssemblyValueStatus MergeAssemblyRawBodies(const AssemblyRawBody& parent,
    const AssemblyRawBody& child,AssemblyRawBody& out) noexcept {
  using namespace assembly_detail;using S=AssemblyValueStatus;
  if(!Disjoint(&parent,sizeof parent,&out,sizeof out)||!Disjoint(&child,sizeof child,&out,sizeof out)||
      !Raw(parent)||!Raw(child))return S::InvalidInput;
  const auto& a=parent.ledger;const auto& b=child.ledger;
  const auto an=a.part_members+a.extra_members,bn=b.part_members+b.extra_members;
  if(bn>MaxAssemblyBodyMembers-an)return S::ResourceLimit;
  AssemblyRawBody next=parent;
  next.center={parent.center.x*parent.mass+child.center.x*child.mass,
    parent.center.y*parent.mass+child.center.y*child.mass,parent.center.z*parent.mass+child.center.z*child.mass};
  next.mass=parent.mass+child.mass;
  if(!std::isfinite(next.mass)||!detail::Finite(next.center))return S::NonfiniteResult;
  next.center={next.center.x/next.mass,next.center.y/next.mass,next.center.z/next.mass};
  Shift(next.tensor,parent.mass,detail::Subtract(parent.center,next.center));
  const auto r=detail::Subtract(child.center,next.center);
  const double xx=r.x*r.x,xy=r.x*r.y,xz=r.x*r.z,yy=r.y*r.y,yz=r.y*r.z,zz=r.z*r.z;
  const auto& c=child.tensor;auto& t=next.tensor;
  t.v[0]=t.v[0]+c.v[0]+(yy+zz)*child.mass;t.v[1]=t.v[1]+c.v[1]-xy*child.mass;t.v[2]=t.v[2]+c.v[2]-xz*child.mass;
  t.v[3]=t.v[3]+c.v[3]-xy*child.mass;t.v[4]=t.v[4]+c.v[4]+(zz+xx)*child.mass;t.v[5]=t.v[5]+c.v[5]-yz*child.mass;
  t.v[6]=t.v[6]+c.v[6]-xz*child.mass;t.v[7]=t.v[7]+c.v[7]-yz*child.mass;t.v[8]=t.v[8]+c.v[8]+(xx+yy)*child.mass;
  next.ledger={a.part_mass+b.part_mass,a.extra_mass+b.extra_mass,a.native_member_inertia+b.native_member_inertia,
    a.primary_mass+b.primary_mass,a.primary_inertia+b.primary_inertia,a.part_members+b.part_members,
    a.extra_members+b.extra_members,a.primaries+b.primaries};
  if(!Raw(next))return S::NonfiniteResult;
  out=next;return S::Success;
}
} // namespace tl::fea::rigid
