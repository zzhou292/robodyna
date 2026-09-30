// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type13Model.h"
#include "../../../lib_utils/BoundedArena.h"
#include "../../../lib_utils/SourceIdentityIndex.h"
#include <cstring>

namespace tl::fea::type13::model_detail {
using Index=util::SourceIdentityIndex<0>;
struct OwnedProperty {
  ModelPropertyInput declaration{};
  Curve curves[CurveCount]{};
  Property value{};
};
struct Layout {
  util::ArenaRegion nodes,properties,connections,startup;
  std::size_t arena_bytes=0,owned_bytes=0,startup_bytes=0;
};
struct Scratch {
  Index node_ids,global_nodes,property_ids,element_ids;
  util::BoundedStartupArray<bool,0> used_nodes,used_properties;
};
inline ModelReport Error(ModelStatus status,const char* message,ModelEntry kind=ModelEntry::None,
                         std::size_t entry=SIZE_MAX,Status native=Status::Success) {
  return {status,kind,entry,native,message};
}
template<class T> bool Range(const T* p,std::size_t n) {
  const auto address=reinterpret_cast<std::uintptr_t>(p);
  return p&&address%alignof(T)==0&&n<=(UINTPTR_MAX-address)/sizeof(T);
}
inline bool Same(double a,double b) { return std::memcmp(&a,&b,sizeof(double))==0; }
inline bool Same(Vec3 a,Vec3 b) { return Same(a.x,b.x)&&Same(a.y,b.y)&&Same(a.z,b.z); }
inline bool Same(WorkingUnits a,WorkingUnits b) {
  return Same(a.mass_to_kg,b.mass_to_kg)&&Same(a.length_to_m,b.length_to_m)&&Same(a.time_to_s,b.time_to_s);
}
bool Same(const ModelPropertyInput&,const ModelPropertyInput&) noexcept;
bool Same(const ModelConnection&,const ModelConnection&) noexcept;
ModelReport Preflight(const ModelInput&,ModelLimits,std::size_t fixed_bytes,Layout&);
ModelReport CheckNodes(const ModelInput&,Scratch&);
ModelReport CheckConnection(const ModelInput&,std::size_t,const Scratch&);
ReferenceInput ReferenceFor(const ModelInput&,const ModelConnection&);
} // namespace tl::fea::type13::model_detail
