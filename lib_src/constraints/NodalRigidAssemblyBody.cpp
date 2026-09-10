// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalRigidAssemblyInternal.h"

namespace tl::fea::rigid {
AssemblyValueStatus PrepareAssemblyRawBody(const AssemblyBodyInput& in,AssemblyRawBody& out) noexcept {
  using namespace assembly_detail;using S=AssemblyValueStatus;
  if(in.part_count<2||in.part_count>MaxAssemblyBodyMembers||
      in.extra_count>MaxAssemblyBodyMembers-in.part_count)return S::ResourceLimit;
  if(!Disjoint(&in,sizeof in,&out,sizeof out)||
      !Disjoint(in.part,in.part_count*sizeof(*in.part),&out,sizeof out)||
      (in.extra_count?!Disjoint(in.extra,in.extra_count*sizeof(*in.extra),&out,sizeof out):in.extra!=nullptr)||
      !detail::Finite(in.primary.position)||!std::isfinite(in.primary.mass)||in.primary.mass<=0||
      !std::isfinite(in.primary.inertia)||in.primary.inertia<=0||
      !std::isfinite(in.source_units.mass_to_kg)||in.source_units.mass_to_kg<=0||
      !std::isfinite(in.source_units.length_to_m)||in.source_units.length_to_m<=0)return S::InvalidInput;
  const double minimum_mass=1e-30*in.source_units.mass_to_kg;
  if(!std::isfinite(minimum_mass)||minimum_mass<=0)return S::InvalidInput;
  AssemblyRawBody next;next.mass=in.primary.mass;
  next.center={in.primary.position.x*next.mass,in.primary.position.y*next.mass,in.primary.position.z*next.mass};
  next.ledger={0,0,0,in.primary.mass,in.primary.inertia,in.part_count,in.extra_count,1};
  for(std::size_t i=0;i<in.part_count;++i) {
    const auto p=in.part[i];if(!Point(p))return S::InvalidInput;
    next.center.x=next.center.x+p.position.x*p.mass;
    next.center.y=next.center.y+p.position.y*p.mass;
    next.center.z=next.center.z+p.position.z*p.mass;next.mass=next.mass+p.mass;
    AddPointLedger(next.ledger,p,false);
  }
  if(!std::isfinite(next.mass)||next.mass<=0||!detail::Finite(next.center))return S::NonfiniteResult;
  if(next.mass<=minimum_mass)return S::InvalidInput;
  next.center={next.center.x/next.mass,next.center.y/next.mass,next.center.z/next.mass};
  Vec3 xg=in.primary.position;double xmg=in.primary.mass;
  if(in.extra_count) {
    xg=next.center;xmg=next.mass;
    next.center={next.center.x*next.mass,next.center.y*next.mass,next.center.z*next.mass};
    for(std::size_t i=0;i<in.extra_count;++i) {
      const auto p=in.extra[i];if(!Point(p))return S::InvalidInput;
      next.center.x=next.center.x+p.position.x*p.mass;
      next.center.y=next.center.y+p.position.y*p.mass;
      next.center.z=next.center.z+p.position.z*p.mass;next.mass=next.mass+p.mass;
      AddPointLedger(next.ledger,p,true);
    }
    if(!std::isfinite(next.mass)||next.mass<=0||!detail::Finite(next.center))return S::NonfiniteResult;
    next.center={next.center.x/next.mass,next.center.y/next.mass,next.center.z/next.mass};
    xg=next.center; // INIRBY:308, intentional source extra-node branch.
  }
  next.tensor.v[0]=next.tensor.v[4]=next.tensor.v[8]=in.primary.inertia;
  Shift(next.tensor,xmg,detail::Subtract(xg,next.center));
  for(std::size_t i=0;i<in.part_count;++i) {
    const auto p=in.part[i];AddMember(next.tensor,p.mass,p.inertia,detail::Subtract(p.position,next.center));
  }
  for(std::size_t i=0;i<in.extra_count;++i) {
    const auto p=in.extra[i];AddMember(next.tensor,p.mass,p.inertia,detail::Subtract(p.position,next.center));
  }
  if(!Raw(next))return S::NonfiniteResult;
  out=next;return S::Success;
}
} // namespace tl::fea::rigid
