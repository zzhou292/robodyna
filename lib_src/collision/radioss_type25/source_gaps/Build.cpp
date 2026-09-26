// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss I25STI3 secondary gaps and I25INI_GAP_N main gaps.
// Copyright (C) 2026 Siemens; see ../LICENSE.md.
#include "../../RadiossType25GapSource.h"
#include "Layout.h"
#include <iterator>
#include <type_traits>
#include "Values.h"
#include <cmath>
#include <cstring>
#include <new>
namespace tlfea::contact::radioss_type25::source_gaps {
Report Preflight(const Input& in,Limits limits,Forecast& out) noexcept {
  detail::Layout plan;const auto report=detail::Prepare(in,limits,plan);
  if(report.status==Status::Ok)out=plan.forecast;
  return report;
}
Report Build(const Input& in,Limits limits,void* scratch,std::size_t bytes,Output out) noexcept {
  detail::Layout plan;auto report=detail::Prepare(in,limits,plan);
  if(report.status!=Status::Ok)return report;
  if(!detail::Disjoint(in,plan,scratch,bytes,out))return {Status::InvalidInput};
  const auto construct=[&](auto* pointer,std::size_t count) {
    using T=std::remove_pointer_t<decltype(pointer)>;
    if(count)::new(static_cast<void*>(pointer))T[count]{};
    return pointer;
  };
  using tl::util::ArenaPointer;
  auto* sec=construct(ArenaPointer<double>(scratch,plan.secondary),in.secondary_count);
  auto* main_nodes=construct(ArenaPointer<double>(scratch,plan.main_nodes),in.main_node_count);
  auto* mains=construct(ArenaPointer<MainGapFields>(scratch,plan.mains),in.main_count);
  auto* wa=construct(ArenaPointer<double>(scratch,plan.secondary_work),in.node_count);
  auto* wm=construct(ArenaPointer<double>(scratch,plan.main_work),in.node_count);
  auto* tags=construct(ArenaPointer<unsigned char>(scratch,plan.tags),in.node_count);
  report=detail::Rows(in,tags);
  if(report.status!=Status::Ok)return report;
  for(std::size_t i=0;i<in.shell_count;++i) {
    const auto& row=in.shells[i];
    const double dx=detail::HalfShell(row,in.profile.input_thickness_mode);
    if(!coefficient_detail::Finite(dx))return {Status::NonfiniteResult,Field::Shell,i};
    for(unsigned k=0;k<detail::Slots(row.layout);++k) {
      const auto node=row.nodes[k];
      wa[node]=detail::Max(wa[node],dx);
      wm[node]=detail::Max(wm[node],dx);
    }
  }
  // Native ILEV1 shell masking happens before all line/spring additions.
  for(std::size_t node=0;node<in.node_count;++node)if(!(tags[node]&4))wa[node]=0.;
  for(unsigned family=0;family<2;++family) {
    const auto* rows=family?in.beams:in.trusses;
    const auto count=family?in.beam_count:in.truss_count;
    for(std::size_t i=0;i<count;++i) {
      const auto& row=rows[i];
      const double dx=row.part_contact_thickness>0?.5*row.part_contact_thickness:.5*std::sqrt(row.native_area);
      if(!coefficient_detail::Finite(dx))return {Status::NonfiniteResult,family?Field::Beam:Field::Truss,i};
      for(const auto node:row.nodes)wa[node]=detail::Max(wa[node],dx);
    }
  }
  for(std::size_t i=0;i<in.spring_count;++i) {
    const auto& row=in.springs[i];
    if(row.part_contact_thickness>0) {
      const double dx=.5*row.part_contact_thickness;
      for(const auto node:row.nodes)wa[node]=detail::Max(wa[node],dx);
    }
  }
  report={Status::Ok};report.minimum_secondary=native_constant::ep20*native_constant::ep10;report.maximum_secondary=0.;
  for(std::size_t i=0;i<in.secondary_count;++i) {
    const double scaled=in.profile.scale*wa[in.secondary_nodes[i]];
    if(!coefficient_detail::Finite(scaled))return {Status::NonfiniteResult,Field::SecondaryRoster,i};
    sec[i]=detail::Min(scaled,in.profile.maximum_secondary);
    report.minimum_secondary=detail::Min(report.minimum_secondary,sec[i]);
    report.maximum_secondary=detail::Max(report.maximum_secondary,sec[i]);
  }
  // Main gap scratch is independent of the masked/augmented secondary field.
  for(std::size_t node=0;node<in.node_count;++node) {
    wm[node]=in.profile.scale*wm[node];
    if(!coefficient_detail::Finite(wm[node]))return {Status::NonfiniteResult,Field::Node,node};
  }
  for(std::size_t i=0;i<in.main_count;++i)if(in.mains[i].segment_type==0)
    for(const auto node:in.mains[i].nodes)wm[node]=0.;
  for(std::size_t i=0;i<in.main_node_count;++i) {
    const auto node=in.main_nodes[i];
    wm[node]=detail::Min(wm[node],in.profile.maximum_main);
    main_nodes[i]=wm[node];
  }
  for(std::size_t i=0;i<in.main_count;++i)for(unsigned k=0;k<4;++k) {
    const double value=wm[in.mains[i].nodes[k]];
    mains[i].corner[k]=value;
    mains[i].maximum=detail::Max(mains[i].maximum,value);
  }
  if(in.secondary_count)std::memcpy(out.secondary,sec,plan.secondary.bytes);
  if(in.main_node_count)std::memcpy(out.main_nodes,main_nodes,plan.main_nodes.bytes);
  if(in.main_count)std::memcpy(out.mains,mains,plan.mains.bytes);
  report.completed=true;
  return report;
}
} // namespace tlfea::contact::radioss_type25::source_gaps
