// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../source_surfaces/Types.h"
#include "../startup/Types.h"
namespace tlfea::contact::radioss_type25::surface_interface {
enum class Status { Ok, InvalidInput, UnsupportedProfile, ResourceLimit, NonfiniteResult, UnsupportedArithmetic };
enum class Profile { Unspecified, SingleSurfaceIlev1 };
struct Input {
  Profile profile=Profile::Unspecified;
  // Complete early reader tables and their actual supplied occurrence order.
  // Clause fields are retained provenance, not reevaluated here.
  source_surfaces::Input physical;
  const source_surfaces::Face* raw_faces=nullptr;
  std::size_t raw_face_count=0;
  VectorView positions;
  startup::Coordinates coordinates=startup::Coordinates::Native;
  UnitScale units;
  std::uint64_t source_generation=0;
};
struct RawClassification {
  int role=0; // IN24 result1/3/7 or signed4/8.
  // First matching supplied solid row, not a mechanical support/removal owner.
  std::uint32_t matched_solid=UINT32_MAX;
};
struct Snapshot {
  const startup::PrimaryFace* primary=nullptr;
  const startup::PrimaryFaceIdentity* identities=nullptr;
  std::size_t primary_count=0,shell_primary_count=0,main_count=0;
  const RawClassification* classifications=nullptr;
  const startup::PrimaryFaceIdentity* raw_origins=nullptr; // Every supplied origin; never filtered away.
  // All original occurrences remain mapped even when I25SURFI coalesces them.
  // Primary-to-raw is the actual native stable winner of the supplied order.
  const std::uint32_t* raw_to_primary=nullptr;
  const std::uint32_t* primary_to_raw=nullptr;
  std::size_t raw_face_count=0;
  const std::uint8_t* surface_solid_flags=nullptr; // Complete pre-filter FLAG_ELEM union.
  std::size_t physical_solid_count=0;
  std::uint64_t source_generation=0;
};
struct Limits {
  std::size_t nodes=1048576,solids=65536,shells=1048576,faces=1572864;
  std::size_t output_bytes=std::size_t{512}<<20,scratch_bytes=std::size_t{512}<<20;
};
struct Forecast {
  std::size_t output_bytes=0,scratch_bytes=0;
  std::size_t maximum_primaries=0,maximum_mains=0,solid_incidence=0;
};
struct Report {
  Status status=Status::InvalidInput;
  std::size_t raw_face=SIZE_MAX,physical_solid=SIZE_MAX,node=SIZE_MAX;
  std::size_t context_row=SIZE_MAX; // Family-local table row from complete context admission.
};
}
