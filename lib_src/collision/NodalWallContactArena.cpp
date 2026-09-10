#include "NodalWallContactArena.h"
#include <limits>

namespace tlfea::contact::nodal_wall_device_detail {
namespace {
bool AppendResult(tl::util::BoundedArenaLayout& b,std::size_t p,std::size_t n,ResultRegions& r) {
  return b.Append<NodalWallParentResult>(p,r.parents)&&b.Append<NodalWallPointResult>(n,r.nodes)&&
      b.Append<std::uint64_t>(n,r.wall_face);
}
bool ConstructResult(tl::util::HostArena& a,const ResultRegions& r,ActiveResults& v) {
  v.parents=a.Construct<NodalWallParentResult>(r.parents); v.nodes=a.Construct<NodalWallPointResult>(r.nodes);
  v.wall_face=a.Construct<std::uint64_t>(r.wall_face); return v.parents&&v.nodes&&v.wall_face;
}
void RebaseResult(void* base,const ResultRegions& r,ActiveResults& v) {
  v.parents=tl::util::ArenaPointer<NodalWallParentResult>(base,r.parents);
  v.nodes=tl::util::ArenaPointer<NodalWallPointResult>(base,r.nodes);
  v.wall_face=tl::util::ArenaPointer<std::uint64_t>(base,r.wall_face);
}
}
bool BuildArenaLayout(std::size_t p,std::size_t n,std::size_t g,std::size_t cap,ArenaLayout& output) noexcept {
  if(!p||!n||!g||p>MaxActiveNodalWallDeviceParents||n>g||g>MaxActiveNodalWallDeviceNodes||
      !cap||cap>MaxActiveNodalWallDeviceBytes) return false;
  ArenaLayout next; tl::util::BoundedArenaLayout b(cap);
  if(!b.Append<Storage>(1,next.header)||!b.Append<NodalWallParentWeight>(p,next.parents)||
      !b.Append<NodalWallNodeWeight>(n,next.nodes)||
      !b.Append<std::uint32_t>(n+1,next.incident_offsets)||!b.Append<std::uint32_t>(4*p,next.incident_slots)||
      !b.Append<Vec3>(g,next.positions)||
      !b.Append<double>(g,next.inverse)||!b.Append<std::uint8_t>(g,next.fixed)||
      !b.Append<Control>(n,next.status)||!b.Append<NodalWallPointResult>(4*p,next.shares)||
      !b.Append<double>(6*g,next.force)||!b.Append<double>(n,next.error)||
      !AppendResult(b,p,n,next.base)||!AppendResult(b,p,n,next.result)) return false;
  next.bytes=b.bytes(); next.parent_count=p; next.node_count=n; next.global_count=g;
  output=next; return true;
}
std::size_t HostPreparationBytes(const ArenaLayout& layout) noexcept {
  // Full retained host arena plus a conservative fixed reserve for both header
  // shadows, Impl, layout, and the unchanged bounded planar-wall validator's
  // temporary faces/edges/maps. Caller inputs and allocator bookkeeping excluded.
  constexpr std::size_t reserve=512*1024;
  return layout.bytes>std::numeric_limits<std::size_t>::max()-reserve?
      std::numeric_limits<std::size_t>::max():layout.bytes+reserve;
}
bool PreparedModel::Initialize(const ArenaLayout& layout) {
  if(arena_) return false;
  auto a=std::make_unique<tl::util::HostArena>();
  if(!a->Initialize(layout.bytes)) return false;
  auto* s=a->Construct<Storage>(layout.header); if(!s) return false;
  auto& m=s->model;
  m.parents=a->Construct<NodalWallParentWeight>(layout.parents);
  m.nodes=a->Construct<NodalWallNodeWeight>(layout.nodes);
  m.incident_offsets=a->Construct<std::uint32_t>(layout.incident_offsets);
  m.incident_slots=a->Construct<std::uint32_t>(layout.incident_slots);
  m.initial_position=a->Construct<Vec3>(layout.positions);
  m.inverse_mass=a->Construct<double>(layout.inverse); m.fixed=a->Construct<std::uint8_t>(layout.fixed);
  s->node_status=a->Construct<Control>(layout.status); s->shares=a->Construct<NodalWallPointResult>(layout.shares);
  s->staged_force=a->Construct<double>(layout.force); s->addition_error=a->Construct<double>(layout.error);
  if(!m.parents||!m.nodes||!m.incident_offsets||!m.incident_slots||!m.initial_position||!m.inverse_mass||!m.fixed||!s->node_status||!s->shares||
      !s->staged_force||!s->addition_error||!ConstructResult(*a,layout.base,s->base)||
      !ConstructResult(*a,layout.result,s->result)) return false;
  layout_=layout; arena_=std::move(a); return true;
}
Storage PreparedModel::Rebase(void* base) const noexcept {
  Storage s=storage(); auto& m=s.model; const auto& l=layout_;
  m.parents=tl::util::ArenaPointer<NodalWallParentWeight>(base,l.parents);
  m.nodes=tl::util::ArenaPointer<NodalWallNodeWeight>(base,l.nodes);
  m.incident_offsets=tl::util::ArenaPointer<std::uint32_t>(base,l.incident_offsets);
  m.incident_slots=tl::util::ArenaPointer<std::uint32_t>(base,l.incident_slots);
  m.initial_position=tl::util::ArenaPointer<Vec3>(base,l.positions);
  m.inverse_mass=tl::util::ArenaPointer<double>(base,l.inverse);
  m.fixed=tl::util::ArenaPointer<std::uint8_t>(base,l.fixed);
  s.node_status=tl::util::ArenaPointer<Control>(base,l.status);
  s.shares=tl::util::ArenaPointer<NodalWallPointResult>(base,l.shares);
  s.staged_force=tl::util::ArenaPointer<double>(base,l.force);
  s.addition_error=tl::util::ArenaPointer<double>(base,l.error);
  RebaseResult(base,l.base,s.base); RebaseResult(base,l.result,s.result); return s;
}
} // namespace tlfea::contact::nodal_wall_device_detail
