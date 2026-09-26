// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../current_normals/Types.h"
namespace tlfea::contact::radioss_type25::initial_source::detail {
Status MakeLayout(const Input& in,Limits limits,std::size_t cub_bytes,Layout& output) noexcept {
  const auto n=in.contact.node_count,g=in.contact.main_count,s=in.contact.secondary_count,r=in.contact.normal_count;
  const auto removals=std::min(g*s,std::max(limits.geometric.max_removals,limits.tied.search.max_removals));
  Layout q;tl::util::BoundedArenaLayout work(limits.max_device_bytes),seed(limits.max_device_bytes);
#define ADD(field,type,count) if(!work.Append<type>(count,q.field))return Status::ResourceLimit
  ADD(positions,Vector,n);ADD(nodes,Node,n);ADD(mains,Main,g);ADD(secondary,Secondary,s);ADD(references,Reference,r);
  ADD(main_nodes,std::uint32_t,in.main_node_count);ADD(main_gap,double,g);ADD(solids,EightSlotSolid,in.solid_count);ADD(solid_offsets,std::uint32_t,n+1);
  ADD(solid_incidence,std::uint32_t,8*in.solid_count);ADD(support_solid,std::uint32_t,g);
  ADD(internal_main,std::uint32_t,g);ADD(node_cells,int,3*s);ADD(main_cells,int,6*g);
  ADD(edge_squared,unsigned long long,n);ADD(edge_length,double,s);ADD(main_node_tags,std::uint32_t,n);
  ADD(solid_tags,std::uint32_t,n);ADD(large_tags,std::uint32_t,s);ADD(ordered_edge_sum,double,s);
  ADD(original_gap,double,4*g);ADD(final_offsets,std::uint32_t,s+1);ADD(final_mains,std::uint32_t,removals);
  ADD(winners,initial_state::Winner,s);ADD(row_status,initial_state::RowReport,s);ADD(control,Control,1);
#undef ADD
  if(!seed.Append<NativeGeometryHistory>(s,q.seed.history)||!seed.Append<int>(s,q.seed.flags)||
      !seed.Append<double>(4*g,q.seed.corner_gaps))return Status::ResourceLimit;
  q.seed.bytes=seed.bytes();
  candidates::Limits sl;sl.max_nodes=limits.nodes;sl.max_secondaries=limits.secondaries;sl.max_mains=limits.mains;
  sl.max_removals=removals;sl.max_tasks=limits.max_tasks;sl.max_pairs=limits.max_pairs;
  sl.max_device_bytes=limits.max_device_bytes;sl.max_host_bytes=limits.max_host_bytes;
  const auto status=candidates::detail::MakeStorageLayout({n,s,g,removals},sl,cub_bytes,0,q.sweep);
  if(status!=candidates::Status::Ok)return Status::ResourceLimit;
  tl::util::BoundedArenaLayout total(limits.max_device_bytes);tl::util::ArenaRegion ignored;
  if(!total.Append<std::byte>(work.bytes(),ignored)||!total.Append<std::byte>(q.sweep.forecast.device_bytes,ignored)||
      !total.Append<std::byte>(seed.bytes(),ignored))return Status::ResourceLimit;
  q.forecast.status=Status::Ok;q.forecast.source_device_bytes=work.bytes();q.forecast.seed_device_bytes=seed.bytes();
  q.forecast.sweep_device_bytes=q.sweep.forecast.device_bytes;q.forecast.cub_bytes=cub_bytes;q.forecast.peak_device_bytes=total.bytes();
  output=q;return Status::Ok;
}
Device Bind(void* work,void* sweep,void* seed,const Input& in,const Prepared& p,const Layout& l,Limits limits) noexcept {
  Device d;
#define B(field,type) d.field=tl::util::ArenaPointer<type>(work,l.field)
  B(positions,Vector);B(nodes,Node);B(mains,Main);B(secondary,Secondary);B(references,Reference);B(main_nodes,std::uint32_t);B(main_gap,double);
  B(solids,EightSlotSolid);B(solid_offsets,std::uint32_t);B(solid_incidence,std::uint32_t);B(support_solid,std::uint32_t);
  B(internal_main,std::uint32_t);B(node_cells,int);B(main_cells,int);B(edge_squared,unsigned long long);B(edge_length,double);
  B(main_node_tags,std::uint32_t);B(solid_tags,std::uint32_t);B(large_tags,std::uint32_t);B(ordered_edge_sum,double);
  B(original_gap,double);B(final_offsets,std::uint32_t);B(final_mains,std::uint32_t);
  B(winners,initial_state::Winner);B(row_status,initial_state::RowReport);B(control,Control);
#undef B
  d.seed_history=tl::util::ArenaPointer<NativeGeometryHistory>(seed,l.seed.history);
  d.seed_flags=tl::util::ArenaPointer<int>(seed,l.seed.flags);d.seed_gaps=tl::util::ArenaPointer<double>(seed,l.seed.corner_gaps);
  auto& a=d.sweep;
#define S(field,type) a.field=tl::util::ArenaPointer<type>(sweep,l.sweep.field)
  S(ids,std::uint64_t);S(codes,int);S(secondary,std::uint32_t);S(mains,candidates::detail::MainEntry);S(ranks,std::uint32_t);
  S(removal_offsets,std::uint64_t);S(removals,std::uint32_t);S(keys,double);S(sorted_keys,double);
  S(ordinals,std::uint32_t);S(sorted_ordinals,std::uint32_t);S(ranges,candidates::detail::Range);
  S(task_counts,unsigned long long);S(task_offsets,unsigned long long);S(tasks,candidates::detail::Task);
  S(pair_counts,unsigned long long);S(pair_offsets,unsigned long long);S(pair_keys,std::uint64_t);S(sorted_pair_keys,std::uint64_t);
  S(pairs,candidates::Pair);S(secondary_offsets,std::uint64_t);S(control,candidates::detail::Control);S(cub,std::byte);
#undef S
  a.nodes=in.contact.node_count;a.secondary_count=in.contact.secondary_count;a.main_count=in.contact.main_count;
  a.task_capacity=limits.max_tasks;a.pair_capacity=limits.max_pairs;a.cub_bytes=l.sweep.cub.bytes;
  // Engine primary_main_count is intentionally unused in the closed Starter policy.
  a.primary_main_count=0;a.si=false;a.length=a.velocity=a.time=1;
  d.rows.profile={in.controls.gap_mode,in.controls.initial_penetration,in.controls.damping_flag,
      in.controls.sharp,in.controls.arithmetic_precision,in.controls.partitions};
  d.rows.source=in.contact;d.rows.source.nodes=d.nodes;d.rows.source.mains=d.mains;d.rows.source.secondary=d.secondary;
  d.rows.source.normals=d.references;d.rows.source.normal_to_main={};d.rows.source.removed_main_by_secondary={};
  d.rows.native_positions={reinterpret_cast<const double*>(d.positions),std::uint32_t(a.nodes),3,1};
  d.rows.pairs=a.pairs;d.rows.row_offsets=a.secondary_offsets;d.rows.offset_count=a.secondary_count+1;
  d.nodes_count=a.nodes;d.mains_count=a.main_count;d.secondary_count=a.secondary_count;d.reference_count=in.contact.normal_count;
  d.solid_count=in.solid_count;d.main_node_count=in.main_node_count;d.final_removal_count=p.final_mains.size();d.added_removals=p.added_removals;
  d.controls=in.controls;d.stamp=in.stamp;d.initial_margin=p.diagnostics.initial_margin;d.engine_margin=p.diagnostics.engine_margin;
  d.global_search_gap=in.global_search_gap;
  return d;
}
cudaError_t Upload(const Prepared& p,Device d,cudaStream_t stream) noexcept {
  cudaError_t e=cudaSuccess;
  const auto copy=[&](void* to,const void* from,std::size_t bytes) {if(e==cudaSuccess&&bytes)e=cudaMemcpyAsync(to,from,bytes,cudaMemcpyHostToDevice,stream);};
#define C(field) copy(d.field,p.field.data(),p.field.size()*sizeof(p.field[0]))
  C(positions);C(nodes);C(mains);C(secondary);C(references);C(main_nodes);C(main_gap);C(solids);C(solid_offsets);C(solid_incidence);
  C(support_solid);C(internal_main);C(final_offsets);C(final_mains);
#undef C
  auto& s=d.sweep;
  copy(const_cast<std::uint64_t*>(s.ids),p.node_ids.data(),p.node_ids.size()*sizeof(std::uint64_t));
  copy(const_cast<int*>(s.codes),p.codes.data(),p.codes.size()*sizeof(int));
  copy(const_cast<std::uint32_t*>(s.secondary),p.secondary_nodes.data(),p.secondary_nodes.size()*sizeof(std::uint32_t));
  copy(const_cast<candidates::detail::MainEntry*>(s.mains),p.sweep_mains.data(),p.sweep_mains.size()*sizeof(p.sweep_mains[0]));
  copy(const_cast<std::uint32_t*>(s.ranks),p.main_ranks.data(),p.main_ranks.size()*sizeof(std::uint32_t));
  copy(const_cast<std::uint64_t*>(s.removal_offsets),p.removal_offsets.data(),p.removal_offsets.size()*sizeof(std::uint64_t));
  copy(const_cast<std::uint32_t*>(s.removals),p.removal_nodes.data(),p.removal_nodes.size()*sizeof(std::uint32_t));
  return e;
}
}
