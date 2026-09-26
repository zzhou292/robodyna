// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss STIFINT_ICONTROL, a62b27e6, Copyright (C) 2026 Siemens.
#include "../../RadiossType25NodalCorrection.h"
#include "../coefficients/Common.h"
#include "../search/Ranges.h"
#include "HostRanges.h"
#include "lib_utils/BoundedArena.h"
#include <cstring>
#include <new>
namespace tlfea::contact::radioss_type25::source_nodal::correction {
namespace {
namespace c=coefficient_detail;
struct Layout { tl::util::ArenaRegion values, tags; Forecast forecast; };
Report Prepare(const Input& in, Limits limits, Layout& out) noexcept {
  const Limits max;
  if(!limits.nodes || limits.nodes>max.nodes || limits.solids>max.solids ||
      limits.type24_occurrences>max.type24_occurrences || !limits.scratch_bytes ||
      limits.scratch_bytes>max.scratch_bytes || in.node_count>limits.nodes ||
      in.solid_count>limits.solids || in.type24_secondary_count>limits.type24_occurrences)
    return {Status::ResourceLimit};
  if(!in.node_count || !search::detail::Span(in.coefficients,in.node_count) ||
      !search::detail::Span(in.solids,in.solid_count) ||
      !search::detail::Span(in.type24_secondaries,in.type24_secondary_count))
    return {Status::InvalidInput};
  Layout next;tl::util::BoundedArenaLayout arena(limits.scratch_bytes);
  if(!arena.Append<double>(in.node_count,next.values) ||
      !arena.Append<unsigned char>(in.node_count,next.tags))return {Status::ResourceLimit};
  next.forecast={arena.bytes(),in.node_count*sizeof(double)};
  for(std::size_t n=0;n<in.node_count;++n)
    if(!c::Nonnegative(in.coefficients[n]))return {Status::InvalidInput,SIZE_MAX,SIZE_MAX,std::uint32_t(n)};
  for(std::size_t i=0;i<in.solid_count;++i) {
    const auto& s=in.solids[i];if(s.control!=1)continue;
    if(!c::Nonnegative(s.bulk)||!c::Nonnegative(s.controlled_bulk))return {Status::InvalidInput,i};
    for(auto n:s.nodes)if(n>=in.node_count)return {Status::InvalidInput,i,SIZE_MAX,n};
  }
  out=next;return {Status::Ok};
}
bool Separate(const Input& in,const Layout& layout,void* scratch,std::size_t bytes,Output out) noexcept {
  if(!scratch || reinterpret_cast<std::uintptr_t>(scratch)%alignof(std::max_align_t) ||
      bytes<layout.forecast.scratch_bytes || bytes>UINTPTR_MAX-reinterpret_cast<std::uintptr_t>(scratch) ||
      out.node_count!=in.node_count || !search::detail::Span(out.coefficients,out.node_count))return false;
  using source_nodal::detail::Range;using source_nodal::detail::Disjoint;
  const Range reads[]{{&in,sizeof(in)},{in.coefficients,in.node_count*sizeof(double)},
      {in.solids,in.solid_count*sizeof(Solid)},
      {in.type24_secondaries,in.type24_secondary_count*sizeof(std::uint32_t)}};
  const Range writes[]{{scratch,bytes},{out.coefficients,layout.forecast.output_bytes}};
  for(const auto write:writes)for(const auto read:reads)if(!Disjoint(write,read))return false;
  return Disjoint(writes[0],writes[1]);
}
}
Report Preflight(const Input& in,Limits limits,Forecast& out) noexcept {
  Layout layout;const auto report=Prepare(in,limits,layout);
  if(report.status==Status::Ok)out=layout.forecast;
  return report;
}
Report Apply(const Input& in,Limits limits,void* scratch,std::size_t bytes,Output out) noexcept {
  Layout layout;auto report=Prepare(in,limits,layout);if(report.status!=Status::Ok)return report;
  if(!Separate(in,layout,scratch,bytes,out))return {Status::InvalidInput};
  auto* values=::new(static_cast<void*>(tl::util::ArenaPointer<double>(scratch,layout.values))) double[in.node_count];
  auto* tags=::new(static_cast<void*>(tl::util::ArenaPointer<unsigned char>(scratch,layout.tags))) unsigned char[in.node_count]{};
  std::memcpy(values,in.coefficients,layout.forecast.output_bytes);
  double maximum_factor=1.;
  for(std::size_t i=0;i<in.solid_count;++i) {
    const auto& s=in.solids[i];if(s.control!=1)continue;
    // EM20 is in native pressure units; correction is after the complete sum.
    const double factor=s.controlled_bulk/c::Max(native_constant::em20,s.bulk);
    if(!c::Finite(factor))return {Status::NonfiniteResult,i};
    maximum_factor=c::Max(maximum_factor,factor);
    for(auto n:s.nodes)if(!tags[n]) {
      const double value=factor*values[n];
      if(!c::Finite(value))return {Status::NonfiniteResult,i,SIZE_MAX,n};
      values[n]=value;tags[n]=1;
    }
  }
  for(std::size_t i=0;i<in.type24_secondary_count;++i) {
    const auto n=in.type24_secondaries[i];if(n>=in.node_count || tags[n])continue;
    const double value=maximum_factor*values[n];
    if(!c::Finite(value))return {Status::NonfiniteResult,SIZE_MAX,i,n};
    values[n]=value; // Native TYPE24 does not mark this node: repeats matter.
  }
  std::memcpy(out.coefficients,values,layout.forecast.output_bytes);return {Status::Ok};
}
}
