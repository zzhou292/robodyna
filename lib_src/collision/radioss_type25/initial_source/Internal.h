// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../RadiossType25InitialState.h"
#include "../initial_state/Rows.h"
#include "../initial_state/SearchPacking.h"
#include "../candidates/Launch.h"
#include "lib_utils/BoundedArena.h"
#include <cuda_runtime.h>
#include <vector>
namespace tlfea::contact::radioss_type25::initial_source::detail {
using Main=selection::lifecycle::Main;
using Secondary=selection::lifecycle::Secondary;
using Node=selection::lifecycle::Node;
using Reference=selection::lifecycle::NormalReference;
struct SeedLayout {tl::util::ArenaRegion history,flags,corner_gaps;std::size_t bytes=0;};
struct Layout {
  tl::util::ArenaRegion positions,nodes,mains,secondary,references,main_nodes,main_gap,solids,solid_offsets,solid_incidence;
  tl::util::ArenaRegion support_solid,internal_main,node_cells,main_cells,edge_squared,edge_length,main_node_tags,solid_tags,large_tags,ordered_edge_sum;
  tl::util::ArenaRegion original_gap,final_offsets,final_mains,winners,row_status,control;
  candidates::detail::Layout sweep;
  SeedLayout seed;
  Forecast forecast;
};
struct Control {
  unsigned long long failure=~0ull;
  unsigned long long changed_gaps=0,solid_nodes=0,large_nodes=0,warm_before=0,warm_after=0,reset=0;
  unsigned long long warm_positive=0,warm_zero=0,warm_negative=0;
  double edge_average=0,maximum_edge=0,maximum_secondary_gap=0;
  double minimum[3]{},maximum[3]{};int grid[3]{1,1,1};
};
struct Device {
  initial_state::RowInput rows;
  candidates::detail::Device sweep;
  Vector* positions=nullptr;Node* nodes=nullptr;Main* mains=nullptr;Secondary* secondary=nullptr;Reference* references=nullptr;
  std::uint32_t* main_nodes=nullptr;double* main_gap=nullptr;EightSlotSolid* solids=nullptr;
  std::uint32_t* solid_offsets=nullptr;std::uint32_t* solid_incidence=nullptr;std::uint32_t* support_solid=nullptr;
  std::uint32_t* internal_main=nullptr;int* node_cells=nullptr;int* main_cells=nullptr;
  unsigned long long* edge_squared=nullptr;double* edge_length=nullptr;
  std::uint32_t* main_node_tags=nullptr;std::uint32_t* solid_tags=nullptr;std::uint32_t* large_tags=nullptr;double* ordered_edge_sum=nullptr;
  double* original_gap=nullptr;
  std::uint32_t* final_offsets=nullptr;std::uint32_t* final_mains=nullptr;
  initial_state::Winner* winners=nullptr;initial_state::RowReport* row_status=nullptr;Control* control=nullptr;
  NativeGeometryHistory* seed_history=nullptr;int* seed_flags=nullptr;double* seed_gaps=nullptr;
  std::size_t nodes_count=0,mains_count=0,secondary_count=0,reference_count=0,solid_count=0,main_node_count=0;
  std::size_t final_removal_count=0,added_removals=0;
  SourceStamp stamp;
  Controls controls;
  double initial_margin=0,engine_margin=0,maximum_secondary_gap=0,global_search_gap=0;
};
// Source-only host staging. No accepted physical state or persistent row mirror.
// The provisional zero History array exists solely to reuse genuine CSR
// construction; only GPU-produced winners can enter the opaque seed.
struct Prepared {
  std::vector<Vector> positions;
  std::vector<Node> nodes;
  std::vector<Main> mains;
  std::vector<Secondary> secondary;
  std::vector<Reference> references;
  std::vector<double> main_gap,primary_extent;
  std::vector<EightSlotSolid> solids;
  std::vector<std::uint32_t> solid_offsets,solid_incidence,support_solid,internal_main;
  std::vector<std::uint64_t> removal_offsets;
  std::vector<std::uint32_t> removal_nodes,final_offsets,final_mains;
  std::vector<std::uint32_t> final_main_offsets,final_nodes,normal_offsets,normal_mains;
  std::vector<startup::Main> topology;
  std::vector<std::uint32_t> expanded_to_primary,primary_to_partner;
  std::vector<startup::PrimaryFaceIdentity> primary_identities,raw_origins;
  std::vector<std::uint32_t> raw_origin_to_primary;
  std::vector<startup::ShellSideRole> primary_roles;
  std::vector<startup::PrimaryCornerPermutation> primary_corners;
  std::vector<startup::PreShellSolidSupport> before_shell;
  std::vector<startup::PostGapmMainSupport> final_support;
  startup::PostGapmTopology post_gapm;
  startup::Snapshot starter;
  Input descriptor; // Rebound to owned consumed operands; no borrowed arrays survive.
  std::vector<candidates::detail::MainEntry> sweep_mains;
  std::vector<std::uint32_t> main_ranks,secondary_nodes,main_nodes;
  std::vector<std::uint64_t> node_ids;
  std::vector<int> codes;
  Diagnostics diagnostics;
  std::size_t added_removals=0,native_nodes=0;
  search_startup::NativePopulation population;
  bool native_nodes_exact=true;
};
Report Admit(const Input&,Limits,Forecast&) noexcept;
Report PrepareHost(const Input&,Limits,Prepared&) noexcept;
void BindPrepared(Prepared&,const Input&) noexcept;
Status MakeLayout(const Input&,Limits,std::size_t cub_bytes,Layout&) noexcept;
Device Bind(void* work,void* sweep,void* seed,const Input&,const Prepared&,const Layout&,Limits) noexcept;
cudaError_t Upload(const Prepared&,Device,cudaStream_t) noexcept;
cudaError_t PrepareOperands(Device,cudaStream_t) noexcept;
cudaError_t BuildRanges(Device,cudaStream_t) noexcept;
cudaError_t CountPairs(Device,std::size_t,cudaStream_t) noexcept;
cudaError_t FillPairs(Device,std::size_t,std::size_t,cudaStream_t) noexcept;
cudaError_t ProduceRows(Device,std::size_t,cudaStream_t) noexcept;
}
namespace tlfea::contact::radioss_type25::initial_source {
struct PreparedSource::Impl {
  detail::Prepared source;
  Forecast forecast;
  detail::Layout layout;
  Limits limits;
  SeedIdentity identity;
};
struct DeviceSeed::Impl {
  void* data=nullptr;std::size_t bytes=0;
  SeedIdentity identity;Diagnostics diagnostics;detail::SeedLayout layout;
  ~Impl(){if(data)cudaFree(data);}
};
}
