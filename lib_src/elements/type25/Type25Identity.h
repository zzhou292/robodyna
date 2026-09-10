// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type25Types.h"
#include <cstring>

namespace tl::fea::type25::detail {
inline bool Same(double a,double b) noexcept { return std::memcmp(&a,&b,sizeof(a))==0; }
inline bool Same(Vec3 a,Vec3 b) noexcept { return Same(a.x,b.x)&&Same(a.y,b.y)&&Same(a.z,b.z); }
inline bool Same(SourceUnits a,SourceUnits b) noexcept {
  return Same(a.mass_to_kg,b.mass_to_kg)&&Same(a.length_to_m,b.length_to_m)&&Same(a.time_to_s,b.time_to_s);
}
inline bool Same(const PropertyInput& a,const PropertyInput& b) noexcept {
  if(a.source_property_id!=b.source_property_id||!Same(a.property.mass_kg,b.property.mass_kg)||
     !Same(a.property.isotropic_inertia_kg_m2,b.property.isotropic_inertia_kg_m2))return false;
  for(unsigned i=0;i<4;++i)
    if(!Same(a.property.stiffness[i],b.property.stiffness[i])||!Same(a.property.damping[i],b.property.damping[i])||
       !Same(a.property.failure_negative[i],b.property.failure_negative[i])||
       !Same(a.property.failure_positive[i],b.property.failure_positive[i])||
       !Same(a.property.failure_weight[i],b.property.failure_weight[i])||
       !Same(a.property.failure_exponent[i],b.property.failure_exponent[i]))return false;
  return true;
}
inline bool Same(const ConnectionInput& a,const ConnectionInput& b) noexcept {
  if(a.source_element_id!=b.source_element_id||a.property_index!=b.property_index||
     !Same(a.seed.x,b.seed.x)||!Same(a.seed.y,b.seed.y))return false;
  for(unsigned k=0;k<2;++k)if(a.source_node_id[k]!=b.source_node_id[k]||a.global_node[k]!=b.global_node[k]||
      !Same(a.position[k],b.position[k]))return false;
  return true;
}
} // namespace tl::fea::type25::detail
