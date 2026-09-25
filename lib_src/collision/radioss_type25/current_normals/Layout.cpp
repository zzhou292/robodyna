// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Layout.h"
#include "Admission.h"
#include "../../self_contact_filters/Environment.h"
#include <new>
namespace tlfea::contact::radioss_type25::current_normals::detail {
Report Plan(const Input& in,Limits limits,Layout& output,double& length) noexcept {
  if ((in.profile!=Profile::OrdinaryShellLocal && in.profile!=Profile::ResolvedShellSidesLocal) ||
      in.free_roster!=normal_activation::FreeRosterPolicy::FreshComplete)
    return {Status::UnsupportedProfile};
  if(!self_contact_filters::CompatibleHostArithmetic())return {Status::UnsupportedArithmetic};
  const auto admitted=Validate(in,limits,length);if(admitted.status!=Status::Ok)return admitted;
  Layout next;tl::util::BoundedArenaLayout arena(limits.scratch_bytes);
  const auto& t=in.topology;
  if(!arena.Append<StoredNormal>(4*t.main_count,next.normal)||!arena.Append<StoredNormal>(4*t.main_count,next.neighbor)||
     !arena.Append<unsigned char>(4*t.main_count,next.eligible)||!arena.Append<unsigned char>(t.primary_count,next.tage)||
     !arena.Append<std::uint32_t>(2*t.references,next.slots)||!arena.Append<startup::NormalReference>(t.references,next.references))
    return {Status::ResourceLimit};
  next.forecast.scratch_bytes=arena.bytes();
  next.forecast.output_bytes=4*t.main_count*sizeof(StoredNormal)+t.references*sizeof(startup::NormalReference);
  output=next;return {Status::Ok};
}
Work Construct(void* scratch,const Layout& l) noexcept {
  using tl::util::ArenaPointer;
  Work w;
#define MAKE(type,field) w.field=::new(static_cast<void*>(ArenaPointer<type>(scratch,l.field))) type[l.field.bytes/sizeof(type)]{}
  MAKE(StoredNormal,normal);MAKE(StoredNormal,neighbor);MAKE(unsigned char,eligible);MAKE(unsigned char,tage);
  MAKE(std::uint32_t,slots);MAKE(startup::NormalReference,references);
#undef MAKE
  return w;
}
bool InputDisjoint(const Input& in,const void* target,std::size_t bytes) noexcept {
  namespace g=geometry_detail;const auto& t=in.topology;const auto& csr=t.normal_to_main;
  const auto last=(in.positions.node_count-1)*in.positions.node_stride+2*in.positions.component_stride;
  const g::Range inputs[]{
    {&in,sizeof(in),alignof(Input)},
    {t.mains,t.main_count*sizeof(startup::Main),alignof(startup::Main)},
    {t.primary_roles,t.primary_role_count*sizeof(startup::ShellSideRole),alignof(startup::ShellSideRole)},
    {csr.offsets,csr.offset_count*sizeof(std::uint32_t),alignof(std::uint32_t)},
    {csr.entries,csr.entry_count*sizeof(std::uint32_t),alignof(std::uint32_t)},
    {in.positions.data,(std::size_t(last)+1)*sizeof(double),alignof(double)},
    {in.main_coefficients,in.coefficient_count*sizeof(double),alignof(double)},
    {in.main_active,in.active_count*sizeof(std::uint32_t),alignof(std::uint32_t)},
    {in.node_tag,in.tag_count*sizeof(std::uint32_t),alignof(std::uint32_t)},
    {in.free_main_ids,in.free_count*sizeof(std::uint32_t),alignof(std::uint32_t)},
    {in.prior_normals,in.prior_count*sizeof(StoredNormal),alignof(StoredNormal)}};
  for(const auto& input:inputs)if(!g::Disjoint({target,bytes,1},input))return false;
  return true;
}
Report Storage(const Input& in,const Layout& layout,void* scratch,std::size_t bytes,Output out) noexcept {
  namespace g=geometry_detail;
  const auto& t=in.topology;
  if(bytes<layout.forecast.scratch_bytes)return {Status::ResourceLimit};
  if(out.normal_count!=4*t.main_count||out.reference_count!=t.references||
     !ld::Span(out.face_normals,out.normal_count)||!ld::Span(out.references,out.reference_count))return {Status::InvalidInput};
  const g::Range buffers[]{
    {scratch,layout.forecast.scratch_bytes,alignof(std::max_align_t)},
    {out.face_normals,out.normal_count*sizeof(StoredNormal),alignof(StoredNormal)},
    {out.references,out.reference_count*sizeof(startup::NormalReference),alignof(startup::NormalReference)}};
  for(unsigned i=0;i<3;++i) {
    if(!g::RangeValid(buffers[i]))return {Status::InvalidInput};
    for(unsigned j=0;j<i;++j)if(!g::Disjoint(buffers[i],buffers[j]))return {Status::InvalidInput};
    if(!InputDisjoint(in,buffers[i].data,buffers[i].bytes))return {Status::InvalidInput};
  }
  return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::current_normals::detail
