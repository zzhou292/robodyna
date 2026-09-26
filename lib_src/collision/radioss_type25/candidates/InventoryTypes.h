// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "../Types.h"
#include "../UnitConversions.h"
#include "../../SurfaceContactTypes.h"
namespace tlfea::contact::radioss_type25::candidates {
enum class InputUnits { Native, Si };
// AllFinite disables only the optional global domain screen. Native strict
// pair screens/packing/PEN3 and complete bounded inventory counts still apply.
enum class DomainPolicy { Bounded, AllFinite };
struct SourceStamp {std::uint64_t source=0,topology=0;};
struct QueryStamp {
  SourceStamp source;
  std::uint64_t activity=0,gaps=0,geometry=0,attempt=0,reference=0;
};
struct Main {std::uint32_t nodes[4]{};std::uint64_t source_id=0;int segment_type=0;};
// Complete local native roster, copied at Initialize. Physical/source node IDs
// are one-to-one. Repeated role occurrences and repeated T3 slots are retained.
// Removal CSR stores physical node ordinals, translated from the genuine native
// source roster. No absent removal/gap/coefficient producer is invented here.
struct Source {
  SourceStamp stamp;
  UnitScale units;InputUnits input_units=InputUnits::Native;
  std::size_t physical_nodes=0,secondaries=0,mains=0,removals=0;
  const std::uint64_t* node_ids=nullptr;
  const int* constraint_codes=nullptr; // Native ICODT, bit1=Z, bit2=Y, bit4=X.
  const std::uint32_t* secondary_nodes=nullptr;
  const Main* main=nullptr;
  const std::uint64_t* removal_offsets=nullptr; // mains+1
  const std::uint32_t* removal_nodes=nullptr;
  int primary_main_count=0; // P=IPARI4-IPARI42, equal to mains; never classification total G.
  int processors=1,edge_mode=0,gap_mode=1,level=1,neighbor_removal=2;
  MainCoefficientDomain main_coefficient_domain=MainCoefficientDomain::Nonnegative;
};
// DEVICE fields and scalar/domain controls follow Source.input_units, borrowed
// immutably until synchronous
// Stage returns. Current values are never cached as a substitute for source
// binding. Coordinates and velocities share the sole existing physical owner.
struct Current {
  QueryStamp stamp;
  VectorView positions,velocities;
  const double* secondary_stiffness=nullptr;
  const double* secondary_gaps=nullptr;
  const double* main_stiffness=nullptr;
  const double* main_gaps=nullptr;
  const double* main_curvature=nullptr;
  Bounds domain;
  double margin=0,gap_load=0,drad=0,stored_motion=0,previous_dt=0;
  DomainPolicy domain_policy=DomainPolicy::Bounded;
};
struct Pair {std::uint32_t secondary_row=0,main_occurrence=0;};
struct Limits {
  std::size_t max_nodes=524288,max_secondaries=524288,max_mains=1048576;
  std::size_t max_removals=16777216,max_tasks=1048576,max_pairs=16777216;
  std::size_t max_device_bytes=std::size_t{2}<<30,max_host_bytes=std::size_t{128}<<20;
};
struct Forecast {std::size_t device_bytes=0,startup_host_bytes=0,cub_bytes=0;};
struct Report {
  QueryStamp stamp;
  Status status=Status::Ok;
  std::size_t failure_row=SIZE_MAX;
  std::uint64_t active_secondaries=0,envelope_encounters=0,tasks=0,pairs=0;
  double maximum_secondary_gap=0;
  unsigned own_kernel_launches=0,sort_calls=0,scan_calls=0,host_fences=0;
};
} // namespace tlfea::contact::radioss_type25::candidates
