// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Layout.h"
#include <climits>
namespace tlfea::contact::radioss_type25::runtime_detail {
bool MakeLayout(const FixedMainSource& source,TransactionLimits limits,std::size_t cub,Layout& out) noexcept {
  const auto& s=source.selection;const auto rows=s.secondary_count,cap=limits.optimized_candidates;
  if(!source.force_packet_size||cap>INT_MAX/5||limits.inventory.max_pairs>=INT_MAX||
     rows>=INT_MAX||limits.sliding_entries>=INT_MAX||!limits.max_device_bytes)return false;
  tl::util::BoundedArenaLayout a(limits.max_device_bytes);Layout l;
#define ADD(type,count,name) if(!a.Append<type>(count,l.name))return false
  ADD(lifecycle::Node,s.node_count,nodes);ADD(lifecycle::Main,s.main_count,mains);
  ADD(lifecycle::NormalReference,s.normal_count,normals);
  ADD(std::uint32_t,s.normal_to_main.offset_count,normal_offsets);
  ADD(std::uint32_t,s.normal_to_main.entry_count,normal_entries);
  ADD(std::uint32_t,s.removed_main_by_secondary.offset_count,removed_offsets);
  ADD(std::uint32_t,s.removed_main_by_secondary.entry_count,removed_entries);
  for(unsigned i=0;i<2;++i){ADD(lifecycle::Secondary,rows,secondary[i]);ADD(NativeGeometryHistory,rows,history[i]);}
  ADD(double,s.node_count,native_mass);ADD(Vector,s.node_count,reference_positions);
  ADD(double,rows,secondary_stiffness);ADD(double,rows,secondary_gaps);
  ADD(double,source.primary_main_count,main_stiffness);ADD(double,source.primary_main_count,main_gaps);
  ADD(double,source.primary_main_count,main_curvature);
  ADD(lifecycle::SpatialOccurrence,limits.inventory.max_pairs,spatial);
  ADD(std::uint32_t,rows+1,spatial_offsets);ADD(std::uint32_t,limits.inventory.max_pairs,spatial_entries);
  ADD(lifecycle::PreparedRow,rows,prepared);ADD(lifecycle::RowStageResult,rows,row_results);
  ADD(std::uint64_t,rows+1,sliding_counts);ADD(std::uint64_t,rows+1,sliding_offsets);
  ADD(int,limits.sliding_entries,sliding);ADD(std::uint64_t,rows+1,candidate_counts);ADD(std::uint64_t,rows+1,candidate_offsets);
  ADD(lifecycle::Occurrence,cap,occurrences);ADD(NativeRawGeometryResult,cap,geometry);
  ADD(NativeGeometryFinalResult,cap,finalized);ADD(NativeFrictionResult,cap,responses);ADD(assembly::SiEndpoints,cap,row_packets);
  ADD(std::uint64_t,cap,order_keys);ADD(std::uint64_t,cap,sorted_keys);
  ADD(std::uint32_t,cap,order_slots);ADD(std::uint32_t,cap,sorted_slots);
  ADD(std::uint32_t,cap+1,positive_flags);ADD(std::uint32_t,cap+1,positive_offsets);ADD(std::uint32_t,cap,force_rank);
  ADD(assembly::Connectivity,cap,force_connectivity);ADD(assembly::SiEndpoints,cap,force_packets);
  ADD(std::uint32_t,cap/source.force_packet_size+(cap%source.force_packet_size!=0),cohort_ends);
  ADD(assembly::SiNodalValue,s.node_count,nodal_output);ADD(Control,1,control);
  tl::util::ArenaRegion padding;if(!a.Append<std::byte>((256-a.bytes()%256)%256,padding))return false;
  ADD(std::byte,cub,cub);
#undef ADD
  l.bytes=a.bytes();out=l;return true;
}
Device Bind(void* arena,const Layout& l,const FixedMainSource& source,TransactionLimits limits) noexcept {
  Device d;d.source=source.selection;
#define BIND(name,type) d.name=tl::util::ArenaPointer<type>(arena,l.name)
  d.source.nodes=tl::util::ArenaPointer<lifecycle::Node>(arena,l.nodes);
  d.source.mains=tl::util::ArenaPointer<lifecycle::Main>(arena,l.mains);
  d.source.normals=tl::util::ArenaPointer<lifecycle::NormalReference>(arena,l.normals);
  d.source.normal_to_main.offsets=tl::util::ArenaPointer<std::uint32_t>(arena,l.normal_offsets);
  d.source.normal_to_main.entries=tl::util::ArenaPointer<std::uint32_t>(arena,l.normal_entries);
  d.source.removed_main_by_secondary.offsets=tl::util::ArenaPointer<std::uint32_t>(arena,l.removed_offsets);
  d.source.removed_main_by_secondary.entries=tl::util::ArenaPointer<std::uint32_t>(arena,l.removed_entries);
  for(unsigned i=0;i<2;++i){BIND(secondary[i],lifecycle::Secondary);BIND(history[i],NativeGeometryHistory);}
  d.source.secondary=d.secondary[0];
  BIND(native_mass,double);BIND(reference_positions,Vector);
  BIND(secondary_stiffness,double);BIND(secondary_gaps,double);BIND(main_stiffness,double);BIND(main_gaps,double);BIND(main_curvature,double);
  BIND(spatial,lifecycle::SpatialOccurrence);BIND(spatial_offsets,std::uint32_t);BIND(spatial_entries,std::uint32_t);
  BIND(prepared,lifecycle::PreparedRow);BIND(row_results,lifecycle::RowStageResult);
  BIND(sliding_counts,std::uint64_t);BIND(sliding_offsets,std::uint64_t);BIND(sliding,int);
  BIND(candidate_counts,std::uint64_t);BIND(candidate_offsets,std::uint64_t);
  BIND(occurrences,lifecycle::Occurrence);BIND(geometry,NativeRawGeometryResult);BIND(finalized,NativeGeometryFinalResult);
  BIND(responses,NativeFrictionResult);BIND(row_packets,assembly::SiEndpoints);
  BIND(order_keys,std::uint64_t);BIND(sorted_keys,std::uint64_t);BIND(order_slots,std::uint32_t);BIND(sorted_slots,std::uint32_t);
  BIND(positive_flags,std::uint32_t);BIND(positive_offsets,std::uint32_t);BIND(force_rank,std::uint32_t);
  BIND(force_connectivity,assembly::Connectivity);BIND(force_packets,assembly::SiEndpoints);BIND(cohort_ends,std::uint32_t);
  BIND(nodal_output,assembly::SiNodalValue);BIND(control,Control);BIND(cub,std::byte);
#undef BIND
  d.cub_bytes=l.cub.bytes;d.primary_count=source.primary_main_count;
  d.raw_capacity=limits.inventory.max_pairs;d.candidate_capacity=limits.optimized_candidates;
  d.sliding_capacity=limits.sliding_entries;d.force_packet_size=source.force_packet_size;return d;
}
} // namespace tlfea::contact::radioss_type25::runtime_detail
