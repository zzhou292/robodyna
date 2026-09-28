// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "NormalStorage.h"
#include "../selection/lifecycle/Admission.h"
#include "lib_utils/BoundedArena.h"
namespace tlfea::contact::radioss_type25::runtime_detail {
struct Control {
  unsigned long long failure=~0ull;
  std::uint64_t required_sliding=0,required_candidates=0,kept=0,active=0;
  double elastic_energy=0,damping_work=0,friction_work=0;
};
struct Layout {
  tl::util::ArenaRegion nodes,mains,normals,normal_offsets,normal_entries,removed_offsets,removed_entries;
  tl::util::ArenaRegion secondary[2],history[2],native_mass,reference_positions;
  tl::util::ArenaRegion secondary_stiffness,secondary_gaps,main_stiffness,main_gaps,main_curvature;
  tl::util::ArenaRegion spatial,spatial_offsets,spatial_entries;
  tl::util::ArenaRegion prepared,row_results,sliding_counts,sliding_offsets,sliding;
  tl::util::ArenaRegion candidate_counts,candidate_offsets,occurrences,geometry,finalized,responses,row_packets;
  tl::util::ArenaRegion order_keys,sorted_keys,order_slots,sorted_slots,positive_flags,positive_offsets,force_rank;
  tl::util::ArenaRegion force_connectivity,force_packets,cohort_ends,nodal_output,control,cub;
  NormalLayout normal;
  std::size_t bytes=0;
};
struct Device {
  lifecycle::SourceView source;
  NormalDevice normal;
  lifecycle::Secondary* secondary[2]{};
  NativeGeometryHistory* history[2]{};
  double* native_mass=nullptr;Vector* reference_positions=nullptr;
  const double* secondary_stiffness=nullptr;double* secondary_gaps=nullptr;
  const double* main_stiffness=nullptr;double* main_gaps=nullptr;double* main_curvature=nullptr;
  lifecycle::SpatialOccurrence* spatial=nullptr;
  std::uint32_t* spatial_offsets=nullptr;std::uint32_t* spatial_entries=nullptr;
  lifecycle::PreparedRow* prepared=nullptr;lifecycle::RowStageResult* row_results=nullptr;
  std::uint64_t* sliding_counts=nullptr;std::uint64_t* sliding_offsets=nullptr;int* sliding=nullptr;
  std::uint64_t* candidate_counts=nullptr;std::uint64_t* candidate_offsets=nullptr;
  lifecycle::Occurrence* occurrences=nullptr;NativeRawGeometryResult* geometry=nullptr;
  NativeGeometryFinalResult* finalized=nullptr;NativeFrictionResult* responses=nullptr;
  assembly::SiEndpoints* row_packets=nullptr;
  std::uint64_t* order_keys=nullptr;std::uint64_t* sorted_keys=nullptr;
  std::uint32_t* order_slots=nullptr;std::uint32_t* sorted_slots=nullptr;
  std::uint32_t* positive_flags=nullptr;std::uint32_t* positive_offsets=nullptr;std::uint32_t* force_rank=nullptr;
  assembly::Connectivity* force_connectivity=nullptr;assembly::SiEndpoints* force_packets=nullptr;
  std::uint32_t* cohort_ends=nullptr;assembly::SiNodalValue* nodal_output=nullptr;
  Control* control=nullptr;void* cub=nullptr;std::size_t cub_bytes=0;
  std::size_t primary_count=0,raw_capacity=0,candidate_capacity=0,sliding_capacity=0;
  unsigned force_packet_size=0;
};
bool MakeLayout(const ContactSourceInput&,TransactionLimits,std::size_t cub_bytes,Layout&,NormalShape={},
    ResponseMassPolicy=ResponseMassPolicy::StaticPhysicalLedger) noexcept;
Device Bind(void*,const Layout&,const ContactSourceInput&,TransactionLimits,NormalShape={}) noexcept;
} // namespace tlfea::contact::radioss_type25::runtime_detail
