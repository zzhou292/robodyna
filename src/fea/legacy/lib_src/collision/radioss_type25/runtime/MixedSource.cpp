// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Source.h"
#include "../current_normals/Types.h"
#include "../normal_math/FloatNormals.h"
#include "../selection/lifecycle/Admission.h"
#include "../normal_activation/Values.h"
#include "lib_src/math/ScalarBits.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>
#include <new>
#include <stdexcept>
namespace tlfea::contact::radioss_type25::runtime_detail {
namespace {
namespace ld=lifecycle::detail;
TransactionReport Fail(TransactionStatus status,const char* message,std::size_t row=SIZE_MAX) {
  return {status,message,row};
}
bool Same(StoredNormal a,StoredNormal b) {
  return tl::math::SameScalarBits(a.x,b.x)&&tl::math::SameScalarBits(a.y,b.y)&&tl::math::SameScalarBits(a.z,b.z);
}
TransactionReport Snapshot(const MixedMovingMainSource& source,current_normals::Topology& topology,std::size_t cap) {
  const auto& s=source.selection;const auto& t=source.starter;
  const auto p=source.primary_main_count,g=s.main_count,r=s.normal_count;
  if(t.profile!=startup::Profile::MixedSurface||t.topology!=startup::TopologyPolicy::NativeMixedSurface||
      !p||p>INT_MAX/8||t.node_count!=s.node_count||t.primary_count!=p||t.main_count!=g||
      t.source_generation!=s.generation||t.starter.reference_count!=r||!r||
      t.normal_incidence_count!=s.normal_to_main.entry_count||
      !ld::Span(t.mains,g)||!ld::Span(t.primary_to_partner,p)||!ld::Span(t.expanded_to_primary,g)||
      !ld::Span(t.normal_offsets,r+1)||!ld::Span(t.normal_mains,t.normal_incidence_count)||
      !ld::Span(t.starter.face_normals,4*g)||!ld::Span(t.starter.references,r)||
      !ld::Span(s.mains,g)||!ld::Span(s.normals,r)||
      !ld::Span(s.normal_to_main.offsets,r+1)||s.normal_to_main.offset_count!=r+1||
      !ld::Span(s.normal_to_main.entries,t.normal_incidence_count))
    return Fail(TransactionStatus::SourceMismatch,"Incomplete mixed Starter/selection snapshot");
  topology={t.mains,t.node_count,p,g,r,{t.normal_offsets,r+1,t.normal_mains,t.normal_incidence_count}};
  topology.source_profile=t.profile;topology.source_topology=t.topology;
  topology.primary_roles=t.primary_roles;topology.primary_role_count=t.primary_role_count;
  topology.mixed_maps={t.primary_to_partner,p};
  const auto checked=current_normals::ValidateMixedSource(topology,t,std::min(cap,current_normals::Limits{}.source_validation_bytes));
  if(checked.status!=current_normals::Status::Ok)
    return Fail(TransactionStatus::SourceMismatch,"Mixed post-GAPM normal topology is not authenticated",checked.main);
  for(std::size_t i=0;i<r+1;++i)if(t.normal_offsets[i]!=s.normal_to_main.offsets[i])
    return Fail(TransactionStatus::SourceMismatch,"Mixed normal CSR offsets differ from selection",i);
  for(std::size_t i=0;i<t.normal_incidence_count;++i)if(t.normal_mains[i]!=s.normal_to_main.entries[i])
    return Fail(TransactionStatus::SourceMismatch,"Mixed normal CSR order differs from selection",i);
  for(std::size_t i=0;i<g;++i) {
    const auto& main=t.mains[i];const auto& selected=s.mains[i];
    if(main.global_id!=selected.global_id||main.segment_type!=selected.segment_type)
      return Fail(TransactionStatus::SourceMismatch,"Mixed main contact identity or role differs from selection",i);
    for(unsigned k=0;k<4;++k)
      if(main.nodes[k]!=selected.nodes[k]||main.normal_reference[k]!=selected.normal_reference[k]||
          main.neighbors[k]!=selected.neighbors[k]||!Same(t.starter.face_normals[4*i+k],selected.normal_slot[k]))
        return Fail(TransactionStatus::SourceMismatch,"Mixed main geometry/normal fields differ from Starter",i);
  }
  for(std::size_t i=0;i<r;++i) {
    const auto& ref=t.starter.references[i];const auto& selected=s.normals[i];
    if(ref.boundary<0||ref.boundary>1||ref.boundary!=selected.boundary||
        !normal_math::Finite(ref.bisector[0])||!normal_math::Finite(ref.bisector[1])||
        !Same(ref.bisector[0],selected.bisector[0])||!Same(ref.bisector[1],selected.bisector[1]))
      return Fail(TransactionStatus::SourceMismatch,"Mixed initial reference differs from Starter cache",i);
  }
  return {TransactionStatus::Ok,"OK"};
}
}
TransactionReport PrepareSource(const TransactionConfig& config,const MixedMovingMainSource& source,
    const tl::fea::ShellPhysicalBinding& physical,TransactionLimits limits,SourceStaging& out) noexcept try {
  const auto& a=source.activation;
  if(a.edge_mode!=0||a.foreign_rows!=0||a.partitions!=1||a.neighbor_removal!=2||a.local_processor!=1||
      a.free_roster!=normal_activation::FreeRosterPolicy::FreshComplete||config.lifecycle.neighbor_removal!=a.neighbor_removal)
    return Fail(TransactionStatus::UnsupportedProfile,"Mixed source requires native local fresh-roster activation");
  SourceStaging next;
  auto report=Snapshot(source,next.moving.topology,limits.max_host_bytes);if(report.status!=TransactionStatus::Ok)return report;
  const auto topology=next.moving.topology;
  report=PrepareSourceChecked(config,source,physical,limits,false,next,&source.starter);
  if(report.status!=TransactionStatus::Ok)return report;
  next.moving.topology=topology;
  tl::util::BoundedArenaLayout host(limits.max_host_bytes);tl::util::ArenaRegion unused;
  const auto g=source.selection.main_count;
  const auto validation_bytes=current_normals::MixedSourceValidationBytes(source.primary_main_count);
  if(!validation_bytes||!host.Append<std::byte>(validation_bytes,unused)||!host.Append<std::byte>(next.bytes,unused)||!host.Append<std::uint32_t>(g,unused)||!host.Append<double>(g,unused))
    return Fail(TransactionStatus::ResourceLimit,"Complete mixed source staging exceeds host cap");
  next.moving.main_coefficients.resize(g);next.moving.free_main_ids.reserve(g);
  for(std::size_t i=0;i<g;++i) {
    const auto& main=source.selection.mains[i];next.moving.main_coefficients[i]=main.coefficient;
    if(normal_activation::detail::FreeMain(main))next.moving.free_main_ids.push_back(std::uint32_t(i+1));
  }
  next.moving.starter=source.starter;next.moving.activation=a;next.moving.enabled=true;next.moving.mixed=true;
  next.bytes=host.bytes();out=std::move(next);return {TransactionStatus::Ok,"OK"};
} catch(const std::bad_alloc&) {return Fail(TransactionStatus::ResourceLimit,"Mixed source staging allocation failed");}
  catch(const std::length_error&) {return Fail(TransactionStatus::ResourceLimit,"Mixed source staging size overflow");}
}
