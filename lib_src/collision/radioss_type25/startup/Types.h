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
struct PostGapmTopology;
enum class Status { Ok, InvalidInput, UnsupportedProfile, UnsupportedTopology,
  NonfiniteResult, UnsupportedArithmetic, ResourceLimit };
enum class Profile { Unspecified, OrdinaryExteriorFixedMain, OrdinaryExteriorMovingMain, ResolvedShellSides, MixedSurface };
enum class Coordinates { Native, Si };
// Motion profile and topology admission are independent. The zero/default
// policy retains the qualified manifold matcher and its exact arena forecast.
enum class TopologyPolicy { ManifoldTwoSided, NativeOrdinaryShell, NativeResolvedShellSides, NativeMixedSurface };
// Already resolved source roles. This producer does not classify shell/solid membership.
enum class ShellSideRole { Ordinary, CoatingForward, CoatingReversed };
enum class PrimaryFaceKind { Shell, Solid };
enum class PrimaryOrigin { SingleSourceFace, MultipleOrigins };
// Original source parent and contact face are distinct identities. This is
// NOT an INSOL3D/INCOQ3 mechanical support/removal owner. Multiple-origin
// primaries carry unavailable parent/localface0 and retain every raw origin.
struct PrimaryFaceIdentity {
  PrimaryFaceKind kind=PrimaryFaceKind::Shell;
  std::uint64_t physical_parent_id=0;
  std::uint8_t local_face=0; // Single shell0, single solid1..6; multiple0.
  PrimaryOrigin origin=PrimaryOrigin::SingleSourceFace;
  std::uint32_t origin_count=1;
};
struct PrimaryFace {
  // Legacy: nonzero original parent EID. Mixed multi-origin primary:0 is
  // explicitly unavailable, never a synthetic EID or selected removal owner.
  std::uint64_t source_id=0;
  ShellLayout layout=ShellLayout::Unspecified;
  std::uint32_t nodes[4]{}; // Zero-based source-node indices; T3 repeats slot3 in slot4.
  ShellSideRole side_role = ShellSideRole::Ordinary;
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
  TopologyPolicy topology=TopologyPolicy::ManifoldTwoSided;
  // Mixed profile only. Exact filtered I25SURFI primary order is a supplied
  // source operand; this producer does not reorder or classify these faces.
  const PrimaryFaceIdentity* primary_identities=nullptr;
  std::size_t primary_identity_count=0,shell_primary_count=0;
  const PrimaryFaceIdentity* raw_origins=nullptr; // Every entry is SingleSourceFace.
  const std::uint32_t* raw_origin_to_primary=nullptr;
  std::size_t raw_origin_count=0;
};
struct Main {
  // Original source parent when uniquely available. Mixed multi-origin0.
  // global_id is the contact-main identity; source_id is never a support owner.
  std::uint64_t source_id=0;
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
  const std::uint32_t* expanded_to_primary=nullptr; // Zero-based primary FACE ordinal; legacy one face/parent.
  const std::uint32_t* primary_to_partner=nullptr; // One-based native opposite ID; mixed solid0 means absent.
  const std::uint32_t* normal_offsets=nullptr; // Zero-based native prefix offsets.
  const std::uint32_t* normal_mains=nullptr; // One-based native main IDs, source insertion order.
  std::size_t normal_incidence_count=0;
  NormalView starter; // Native Starter boolean LBOUND and I25NORM float fields.
  std::uint64_t source_generation=0;
  Profile profile=Profile::Unspecified;
  TopologyPolicy topology=TopologyPolicy::ManifoldTwoSided;
  // Owned immutable input-role provenance only in the resolved profile. The
  // normalized first-side geometry alone cannot distinguish reversed input scope.
  const ShellSideRole* primary_roles = nullptr;
  std::size_t primary_role_count = 0;
  // Owned mixed source provenance. Legacy snapshots retain null/zero values.
  const PrimaryFaceIdentity* primary_identities=nullptr;
  std::size_t primary_identity_count=0,shell_primary_count=0;
  const PrimaryFaceIdentity* raw_origins=nullptr;
  const std::uint32_t* raw_origin_to_primary=nullptr;
  std::size_t raw_origin_count=0;
  // Owned post-GAPM support/permutation provenance only for the new explicit
  // mixed Starter overload; legacy snapshots keep nullptr.
  const PostGapmTopology* post_gapm=nullptr;
};
// SH2-only output. Neighbor/reference/float-normal channels in Main remain
// API-zero/unavailable, not native observations. Full Starter construction
// needs authentic post-I25GAPM support/internal and final erosion disposition.
struct MixedSidesSnapshot {
  const Main* mains=nullptr;
  std::size_t node_count=0,primary_count=0,main_count=0,shell_primary_count=0;
  const std::uint32_t* expanded_to_primary=nullptr;
  const std::uint32_t* primary_to_partner=nullptr;
  const ShellSideRole* primary_roles=nullptr;
  const PrimaryFaceIdentity* primary_identities=nullptr;
  const PrimaryFaceIdentity* raw_origins=nullptr;
  const std::uint32_t* raw_origin_to_primary=nullptr;
  std::size_t raw_origin_count=0;
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
  Profile profile=Profile::Unspecified;
  TopologyPolicy topology=TopologyPolicy::ManifoldTwoSided;
};
struct Limits {
  std::size_t max_nodes=1048576,max_primary_faces=1048576;
  std::size_t max_output_bytes=std::size_t{1}<<30;
  std::size_t max_scratch_bytes=std::size_t{1}<<30;
  std::size_t max_raw_origins=1572864; // Mixed only; old forecasts unchanged.
};
struct Forecast {
  Status status=Status::InvalidInput;
  // ready_* are zero/unavailable for both native general policies.
  std::size_t output_bytes=0,scratch_bytes=0,ready_output_bytes=0,ready_scratch_bytes=0;
  std::size_t expanded_mains=0,maximum_references=0,maximum_incidence=0;
};
struct NeighborWarnings {
  std::size_t count=0;
  // Native IRR11: the source selects neighbor zero and emits warning1245.
  // Main and edge are zero-based expanded source ordinals, not a deleted face.
  std::size_t first_main=SIZE_MAX,first_edge=SIZE_MAX;
};
struct Report {
  Status status=Status::InvalidInput;
  std::size_t primary=SIZE_MAX,node=SIZE_MAX;
  NeighborWarnings neighbor_warnings;
};
} // namespace tlfea::contact::radioss_type25::startup
