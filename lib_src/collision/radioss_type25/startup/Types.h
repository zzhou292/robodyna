// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../CoefficientTypes.h"
#include "../GeometryTypes.h"
#include "../NormalFields.h"
#include "../Units.h"
#include "lib_src/collision/SurfaceContactTypes.h"
#include <cstddef>
#include <cstdint>
namespace tlfea::contact::radioss_type25::startup {
enum class Status { Ok, InvalidInput, UnsupportedProfile, UnsupportedTopology,
  NonfiniteResult, UnsupportedArithmetic, ResourceLimit };
enum class Profile { Unspecified, OrdinaryExteriorFixedMain, OrdinaryExteriorMovingMain };
enum class Coordinates { Native, Si };
struct PrimaryFace {
  std::uint64_t source_id=0;
  ShellLayout layout=ShellLayout::Unspecified;
  std::uint32_t nodes[4]{}; // Zero-based source-node indices; T3 repeats slot3 in slot4.
};
struct Input {
  Profile profile=Profile::Unspecified;
  const std::uint64_t* node_source_ids=nullptr;
  std::size_t node_count=0;
  VectorView positions;
  const PrimaryFace* primary=nullptr;
  std::size_t primary_count=0;
  Coordinates coordinates=Coordinates::Native;
  UnitScale units; // Required only for the explicit SI position boundary.
  std::uint64_t source_generation=0;
  // Primary order is supplied by the source binding. This numerical producer
  // does not silently sort a deck, weld equal coordinates, or own a clock.
};
struct Main {
  std::uint64_t source_id=0; // Physical primary identity is shared by its two sides.
  std::uint32_t nodes[4]{};
  int global_id=0,segment_type=0; // Original local MSEGLO / signed MSEGTYP.
  int neighbors[4]{},neighbor_edges[4]{},normal_reference[4]{};
};
using NormalReference=normal_fields::Reference;
struct NormalView {
  const StoredNormal* face_normals=nullptr; // Four original source slots per expanded main.
  const NormalReference* references=nullptr;
  std::size_t reference_count=0;
};
struct Snapshot {
  const Main* mains=nullptr;
  std::size_t node_count=0,primary_count=0,main_count=0;
  const std::uint32_t* expanded_to_primary=nullptr; // Zero-based physical parent ordinal.
  const std::uint32_t* primary_to_partner=nullptr; // One-based native opposite main ID.
  const std::uint32_t* normal_offsets=nullptr; // Zero-based native prefix offsets.
  const std::uint32_t* normal_mains=nullptr; // One-based native main IDs, source insertion order.
  std::size_t normal_incidence_count=0;
  NormalView starter; // Native Starter boolean LBOUND and I25NORM float fields.
  std::uint64_t source_generation=0;
};
struct FixedMainInput {
  const double* main_coefficients=nullptr;
  std::size_t main_count=0;
  // Borrowed only for the native STIFM activity condition, never calculated or
  // owned here. First supported ready profile is fixed, non-eroding main faces.
};
struct FixedMainView {
  NormalView normals; // Distinct source NORMP stage; boundary counts are retained.
  std::uint64_t source_generation=0;
};
struct Limits {
  std::size_t max_nodes=1048576,max_primary_faces=1048576;
  std::size_t max_output_bytes=std::size_t{1}<<30;
  std::size_t max_scratch_bytes=std::size_t{1}<<30;
};
struct Forecast {
  Status status=Status::InvalidInput;
  std::size_t output_bytes=0,scratch_bytes=0,ready_output_bytes=0,ready_scratch_bytes=0;
  std::size_t expanded_mains=0,maximum_references=0,maximum_incidence=0;
};
struct Report {
  Status status=Status::InvalidInput;
  std::size_t primary=SIZE_MAX,node=SIZE_MAX;
};
} // namespace tlfea::contact::radioss_type25::startup
