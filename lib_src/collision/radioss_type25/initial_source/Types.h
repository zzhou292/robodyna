// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../initial_state/Types.h"
#include "../selection/lifecycle/Types.h"
#include "../startup/PostGapmTypes.h"
#include "../tied_removal/Types.h"
#include <cstddef>
#include <cstdint>
namespace tlfea::contact::radioss_type25::initial_source {
enum class Status { Ok,InvalidInput,WrongPhase,UnsupportedProfile,UnsupportedTopology,
  UnsupportedArithmetic,NonfiniteResult,ResourceLimit,DeviceFailure,AlreadyPrepared,NotPrepared };
enum class Phase { Unspecified,StarterNormalsAndPreBucGaps };
enum class SolidScope { Unspecified,CompleteEightSlotModel,ExplicitNoSolids };
struct SourceStamp {
  std::uint64_t source=0,topology=0,physical_domain=0;
  // Runtime reference-topology version is independent of the Starter/selection
  // generation above. Standalone numerical production may leave it unavailable0.
  std::uint64_t runtime_topology=0;
};
enum class EngineHandoff { Unspecified,SourceProvedFreshSerialSearchAtZero };
struct Controls {
  int level=-1,gap_mode=-1,initial_penetration=-1,damping_flag=-1,sharp=-1;
  int arithmetic_precision=-1,partitions=0,starter_workers=0,edge_mode=-1,thermal=-1;
  int neighbor_removal=-1,tied_removal=-1,thickness_update=-1,stiffness_formulation=-1,stiffness_mass_update=-1;
  bool gap_load_cards=true;
  // The declared/default source BMUL0 is explicit. Resolved BUMULT uses actual
  // native node population through the existing qualified reader rule.
  double base_multiplier=0,drad=0,gap_load=0;
  std::uint64_t native_voxel_capacity=0; // Actual LVOXEL source, not GPU allocation allowance.
  int curvature=-1,native_packet_size=0; // Resolved IPARI39 and selected NVSIZ, not MVSIZ storage.
};
// Original pre-INITIA reader connectivity. Repeated raw PENTA/brick slots stay
// present. No post-orientation source_slot or prepared reference substitution.
struct EightSlotSolid {
  std::uint64_t native_source_id=0,part_source_id=0;
  std::uint32_t nodes[8]{};
};
enum class InterfaceKind { Unspecified,Type2,Type25,Unsupported };
enum class InterfaceOrigin { Unspecified,OriginalDefinition,DeclaredAdditionalInterface };
enum class InterfaceCensusPhase { Unspecified,CompleteOriginalAndDeclaredAdditions };
struct InterfaceIdentity {
  std::uint64_t source_id=0;
  std::uint32_t native_storage_ordinal=0;
  InterfaceKind kind=InterfaceKind::Unspecified;
  InterfaceOrigin origin=InterfaceOrigin::Unspecified;
};
struct Input {
  Phase phase=Phase::Unspecified;
  SourceStamp stamp;
  UnitScale units; // Native working coordinates/coefficients; explicit SI conversion identity.
  EngineHandoff engine_handoff=EngineHandoff::Unspecified;
  Controls controls;
  InterfaceCensusPhase interface_phase=InterfaceCensusPhase::Unspecified;
  const InterfaceIdentity* interfaces=nullptr;std::size_t interface_count=0;
  std::uint64_t native_interface_id=0;
  startup::Input mesh;
  startup::Snapshot starter;
  selection::lifecycle::SourceView contact;
  const std::uint32_t* main_nodes=nullptr;std::size_t main_node_count=0; // Actual ordered MSR.
  // Actual GAP_M face scalar supplied to BUC after INI_GAP_N, separate from its4corners.
  const double* main_search_gap=nullptr;std::size_t main_search_gap_count=0;
  // I25STI3 final scalar GAP before INI_GAP_N: original GAPS_MX+GAPM_MX.
  // It is not the maximum of the later four corner fields.
  double global_search_gap=0;
  SolidScope solid_scope=SolidScope::Unspecified;
  const EightSlotSolid* solids=nullptr;std::size_t solid_count=0;
  // Complete source contributor census and generated rigid primary namespace,
  // not extra physical nodes or a caller-created geometry facade.
  search_startup::Contributors contributors;
  search_startup::NativePopulation native_population;
  const std::uint64_t* auxiliary_rigid_primary_ids=nullptr;
  std::size_t auxiliary_rigid_primary_count=0;
  tied_removal::Finalization tied_phase=tied_removal::Finalization::Unspecified;
  const tied_removal::Interface* tied_interfaces=nullptr;std::size_t tied_interface_count=0;
};
struct Limits {
  std::size_t nodes=524288,secondaries=524288,mains=1048576,solids=262144;
  std::size_t max_tasks=8388608,max_pairs=16777216;
  std::size_t max_device_bytes=std::size_t{4}<<30,max_host_bytes=std::size_t{2}<<30;
  search_startup::Limits geometric;
  tied_removal::Limits tied;
};
struct Forecast {
  Status status=Status::InvalidInput;
  std::size_t source_device_bytes=0,operand_device_bytes=0,sweep_device_bytes=0;
  std::size_t seed_device_bytes=0,cub_bytes=0,peak_device_bytes=0;
  std::size_t retained_host_bytes=0,temporary_host_bytes=0,peak_host_bytes=0;
  std::size_t geometric_output_bytes=0,geometric_scratch_bytes=0;
  std::size_t tied_output_bytes=0,tied_scratch_bytes=0;
};
struct Diagnostics {
  std::uint64_t encounters=0,tasks=0,pairs=0,warm_before_tied=0,warm_after_tied=0,tied_reset=0;
  std::uint64_t warm_positive_main=0,warm_zero_main=0,warm_negative_main=0;
  std::uint64_t changed_gap_corners=0,solid_tagged_nodes=0,large_secondaries=0;
  double mean_length=0,engine_margin=0,initial_margin=0,edge_average=0;
  int grid[3]{};
};
struct Report {
  Status status=Status::InvalidInput;
  std::size_t node=SIZE_MAX,main=SIZE_MAX,row=SIZE_MAX;
  bool counts_complete=false;
  Diagnostics diagnostics;
};
struct SeedIdentity {
  SourceStamp source;
  UnitScale units;
  std::size_t nodes=0,primaries=0,mains=0,secondaries=0,references=0;
  Phase input_phase=Phase::Unspecified;
  search_startup::NativePopulation native_population;
  bool native_model_nodes_exact=true;
  EngineHandoff engine_handoff=EngineHandoff::Unspecified;
};
// Both directions are genuinely derived by the geometric and finalized TYPE2
// producers. Entries retain native source order; no row history is exposed.
struct FinalRemovalView {
  const std::uint32_t* main_offsets=nullptr;
  const std::uint32_t* removed_nodes=nullptr;
  selection::lifecycle::Csr by_secondary;
  std::size_t main_count=0,used_count=0,added_by_tied=0,native_model_nodes=0;
  double mean_length=0,engine_margin=0,initial_margin=0;
  search_startup::NativePopulation native_population;
  bool native_model_nodes_exact=true;
  const double* primary_extent=nullptr;
  std::size_t primary_count=0;
};
} // namespace tlfea::contact::radioss_type25::initial_source
