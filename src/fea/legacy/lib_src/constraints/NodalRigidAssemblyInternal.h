// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidAssemblyValues.h"
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <limits>

namespace tl::fea::rigid::assembly_detail {
inline bool Disjoint(const void* a,std::size_t as,const void* b,std::size_t bs) {
  const auto x=reinterpret_cast<std::uintptr_t>(a),y=reinterpret_cast<std::uintptr_t>(b);
  return a&&b&&as&&bs&&x<=UINTPTR_MAX-as&&y<=UINTPTR_MAX-bs&&(x>=y+bs||y>=x+as);
}
inline bool Nonnegative(double x) {return std::isfinite(x)&&x>=0;}
inline bool Point(AssemblyMassPoint p) {
  return detail::Finite(p.position)&&Nonnegative(p.mass)&&Nonnegative(p.inertia);
}
inline bool Finite(const AssemblyRawBody& b) {
  if(!std::isfinite(b.mass)||b.mass<=0||!detail::Finite(b.center))return false;
  for(auto x:b.tensor.v)if(!std::isfinite(x))return false;
  return true;
}
inline bool Ledger(const AssemblyMassLedger& l) {
  return Nonnegative(l.part_mass)&&Nonnegative(l.extra_mass)&&Nonnegative(l.native_member_inertia)&&
    std::isfinite(l.primary_mass)&&l.primary_mass>0&&std::isfinite(l.primary_inertia)&&l.primary_inertia>0&&
    l.primaries&&l.primaries<=MaxAssemblyBodyMembers/2&&l.part_members>=2*l.primaries&&
    l.part_members<=MaxAssemblyBodyMembers&&l.extra_members<=MaxAssemblyBodyMembers-l.part_members;
}
inline bool Raw(const AssemblyRawBody& b) {
  const auto& l=b.ledger;
  if(!Finite(b)||!Ledger(l))return false;
  const double accounted=(l.part_mass+l.extra_mass)+l.primary_mass;
  const double scale=std::max(accounted,b.mass);
  // Native member reductions and separately retained partitions round differently.
  const double factor=16*std::numeric_limits<double>::epsilon()*
    static_cast<double>(l.part_members+l.extra_members+l.primaries);
  if(!std::isfinite(accounted)||!std::isfinite(scale)||
      std::abs(b.mass-accounted)>factor*scale)return false;
  return b.tensor.v[0]>0&&b.tensor.v[4]>0&&b.tensor.v[8]>0&&
    b.tensor.v[1]==b.tensor.v[3]&&b.tensor.v[2]==b.tensor.v[6]&&b.tensor.v[5]==b.tensor.v[7];
}
// Native scalar order: existing + J + orbital; do not reassociate J+orbital.
inline void AddMember(tl::math::Matrix3& t,double mass,double j,Vec3 r) {
  const double xx=r.x*r.x,xy=r.x*r.y,xz=r.x*r.z,yy=r.y*r.y,yz=r.y*r.z,zz=r.z*r.z;
  t.v[0]=t.v[0]+j+(yy+zz)*mass;t.v[1]=t.v[1]-xy*mass;t.v[2]=t.v[2]-xz*mass;
  t.v[3]=t.v[3]-xy*mass;t.v[4]=t.v[4]+j+(zz+xx)*mass;t.v[5]=t.v[5]-yz*mass;
  t.v[6]=t.v[6]-xz*mass;t.v[7]=t.v[7]-yz*mass;t.v[8]=t.v[8]+j+(xx+yy)*mass;
}
inline void Shift(tl::math::Matrix3& t,double mass,Vec3 r) {
  const double xx=r.x*r.x,xy=r.x*r.y,xz=r.x*r.z,yy=r.y*r.y,yz=r.y*r.z,zz=r.z*r.z;
  t.v[0]=t.v[0]+(yy+zz)*mass;t.v[1]=t.v[1]-xy*mass;t.v[2]=t.v[2]-xz*mass;
  t.v[3]=t.v[3]-xy*mass;t.v[4]=t.v[4]+(zz+xx)*mass;t.v[5]=t.v[5]-yz*mass;
  t.v[6]=t.v[6]-xz*mass;t.v[7]=t.v[7]-yz*mass;t.v[8]=t.v[8]+(xx+yy)*mass;
}
inline void AddPointLedger(AssemblyMassLedger& l,AssemblyMassPoint p,bool extra) {
  if(extra)l.extra_mass+=p.mass;else l.part_mass+=p.mass;
  l.native_member_inertia+=p.inertia;
}
} // namespace tl::fea::rigid::assembly_detail
