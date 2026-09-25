// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Source.h"
#include "../current_normals/Admission.h"
#include "../normal_activation/Values.h"
#include "lib_utils/BoundedArena.h"
#include "lib_src/math/ScalarBits.h"
#include <new>
#include <stdexcept>
namespace tlfea::contact::radioss_type25::runtime_detail {
namespace {
namespace ld=lifecycle::detail;
TransactionReport Fail(TransactionStatus status,const char* message,std::size_t row=SIZE_MAX) {
  return {status,message,row};
}
bool Same(StoredNormal a,StoredNormal b) noexcept {
  return tl::math::SameScalarBits(a.x,b.x)&&tl::math::SameScalarBits(a.y,b.y)&&tl::math::SameScalarBits(a.z,b.z);
}
// Validate source shape before dereferencing the additional borrowed snapshot.
// The source factory retains producer/lifetime authority; these checks cannot
// authenticate an arbitrary caller's claim that a deck produced the snapshot.
TransactionReport CheckSnapshot(const MovingMainSource& source,current_normals::Topology& topology) {
  const auto& s=source.selection;const auto& t=source.starter;
  const auto p=source.primary_main_count,g=s.main_count,r=s.normal_count;
  if(p>INT_MAX/8||t.node_count!=s.node_count||t.primary_count!=p||t.main_count!=g||
      t.source_generation!=s.generation||t.starter.reference_count!=r||!r||
      t.normal_incidence_count!=s.normal_to_main.entry_count||
      !ld::Span(t.mains,g)||!ld::Span(t.expanded_to_primary,g)||
      !ld::Span(t.primary_to_partner,p)||!ld::Span(t.normal_offsets,r+1)||
      !ld::Span(t.normal_mains,t.normal_incidence_count)||
      !ld::Span(t.starter.face_normals,4*g)||!ld::Span(t.starter.references,r))
    return Fail(TransactionStatus::SourceMismatch,"Moving normal snapshot shape or generation differs from source");
  topology={t.mains,t.node_count,p,g,r,{t.normal_offsets,r+1,t.normal_mains,t.normal_incidence_count}};
  const auto topology_report=current_normals::detail::ValidateTopology(topology);
  if(topology_report.status!=current_normals::Status::Ok)
    return Fail(TransactionStatus::SourceMismatch,"Moving normal topology is not complete ordinary Q4/T3",topology_report.main);
  for(std::size_t i=0;i<r+1;++i)
    if(t.normal_offsets[i]!=s.normal_to_main.offsets[i])
      return Fail(TransactionStatus::SourceMismatch,"Moving normal CSR offsets differ from selection",i);
  for(std::size_t i=0;i<t.normal_incidence_count;++i)
    if(t.normal_mains[i]!=s.normal_to_main.entries[i])
      return Fail(TransactionStatus::SourceMismatch,"Moving normal CSR order differs from selection",i);
  for(std::size_t i=0;i<g;++i) {
    const auto parent=i<p?i:i-p;const auto& main=t.mains[i];const auto& selected=s.mains[i];
    if(t.expanded_to_primary[i]!=parent||main.source_id!=source.primary_parent_ids[parent]||
        main.global_id!=selected.global_id||main.segment_type!=selected.segment_type)
      return Fail(TransactionStatus::SourceMismatch,"Moving main physical identity or role differs from selection",i);
    for(unsigned k=0;k<4;++k) {
      if(main.nodes[k]!=selected.nodes[k]||main.normal_reference[k]!=selected.normal_reference[k]||
          main.neighbors[k]!=selected.neighbors[k]||
          !Same(t.starter.face_normals[4*i+k],selected.normal_slot[k]))
        return Fail(TransactionStatus::SourceMismatch,"Moving main topology or initial float cache differs from selection",i);
      if(main.neighbors[k]) {
        const auto j=std::size_t(main.neighbors[k]-1);const auto edge=unsigned(main.neighbor_edges[k]-1);
        const auto& neighbor=t.mains[j];
        if(neighbor.nodes[edge]!=main.nodes[(k+1)%4]||neighbor.nodes[(edge+1)%4]!=main.nodes[k]||
            neighbor.neighbors[edge]!=int(i+1)||neighbor.neighbor_edges[edge]!=int(k+1)||
            neighbor.normal_reference[edge]!=main.normal_reference[(k+1)%4]||
            neighbor.normal_reference[(edge+1)%4]!=main.normal_reference[k])
          return Fail(TransactionStatus::SourceMismatch,"Moving normal neighbor is not a reciprocal edge",i);
      }
    }
  }
  for(std::size_t i=0;i<p;++i)
    if(t.primary_to_partner[i]!=p+i+1)
      return Fail(TransactionStatus::SourceMismatch,"Moving normal partner map differs from source",i);
  for(std::size_t i=0;i<r;++i) {
    const auto& ref=t.starter.references[i];const auto& selected=s.normals[i];
    if(ref.boundary<0||ref.boundary>1||ref.boundary!=selected.boundary||
        !normal_math::Finite(ref.bisector[0])||!normal_math::Finite(ref.bisector[1])||
        !Same(ref.bisector[0],selected.bisector[0])||!Same(ref.bisector[1],selected.bisector[1]))
      return Fail(TransactionStatus::SourceMismatch,"Moving initial references are not the same Starter cache",i);
  }
  return {TransactionStatus::Ok,"OK"};
}
}
TransactionReport PrepareSource(const TransactionConfig& config,const MovingMainSource& source,
    const tl::fea::ShellPhysicalBinding& physical,TransactionLimits limits,SourceStaging& out) noexcept try {
  const auto& a=source.activation;
  if(a.edge_mode!=0||a.foreign_rows!=0||a.partitions!=1||a.neighbor_removal!=2||
      a.local_processor!=1||a.free_roster!=normal_activation::FreeRosterPolicy::FreshComplete||
      config.lifecycle.neighbor_removal!=a.neighbor_removal)
    return Fail(TransactionStatus::UnsupportedProfile,"Moving source requires ordinary local fresh-roster activation");
  SourceStaging next;
  auto report=PrepareSourceChecked(config,source,physical,limits,false,next);
  if(report.status!=TransactionStatus::Ok)return report;
  report=CheckSnapshot(source,next.moving.topology);
  if(report.status!=TransactionStatus::Ok)return report;
  // Charge worst-case complete free roster before allocation. The common
  // startup peak remains charged conservatively even though some temporaries
  // have already ended. No cap is substituted for an omitted native face.
  tl::util::BoundedArenaLayout host(limits.max_host_bytes);tl::util::ArenaRegion ignored;
  const auto g=source.selection.main_count;
  if(!host.Append<std::byte>(next.bytes,ignored)||!host.Append<std::uint32_t>(g,ignored)||
      !host.Append<double>(g,ignored))
    return Fail(TransactionStatus::ResourceLimit,"Complete moving source startup exceeds host byte cap");
  next.moving.main_coefficients.resize(g);next.moving.free_main_ids.reserve(g);
  for(std::size_t i=0;i<g;++i) {
    const auto& main=source.selection.mains[i];next.moving.main_coefficients[i]=main.coefficient;
    if(normal_activation::detail::FreeMain(main))next.moving.free_main_ids.push_back(std::uint32_t(i+1));
  }
  next.moving.starter=source.starter;next.moving.activation=a;next.moving.enabled=true;
  next.bytes=host.bytes();out=std::move(next);return {TransactionStatus::Ok,"OK"};
} catch(const std::bad_alloc&) {
  return Fail(TransactionStatus::ResourceLimit,"Moving source startup allocation failed");
} catch(const std::length_error&) {
  return Fail(TransactionStatus::ResourceLimit,"Moving source startup length overflow");
}
} // namespace tlfea::contact::radioss_type25::runtime_detail
