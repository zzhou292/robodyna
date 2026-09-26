// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../startup/Types.h"
#include "../selection/lifecycle/Types.h"
#include "../normal_activation/Types.h"
namespace tlfea::contact::radioss_type25::current_normals {
enum class Status { Ok,InvalidInput,UnsupportedProfile,UnsupportedTopology,NonfiniteResult,ResourceLimit,UnsupportedArithmetic };
enum class Profile { Unspecified,OrdinaryShellLocal,ResolvedShellSidesLocal };
enum class RolePolicy { OrdinaryOnly, ResolvedShellSides };
// Immutable two-sided shell topology. Admission verifies unique primary/partner
// writers, reversed connectivity, node-bound reference identities and the exact
// ordered distinct main/reference CSR. It does not authenticate a source deck.
struct Topology {
  const startup::Main* mains=nullptr;
  std::size_t nodes=0,primary_count=0,main_count=0,references=0;
  selection::lifecycle::Csr normal_to_main;
  // Legacy direct numerical packets may leave origin unspecified. The explicit
  // resolved profile requires the immutable role table and matching origin.
  // Numerical validation checks the coating class/pair relationships. Without
  // original primary connectivity it cannot authenticate a caller-made table
  // or distinguish a forged Forward/Reversed label on identical final geometry.
  startup::Profile source_profile = startup::Profile::Unspecified;
  startup::TopologyPolicy source_topology = startup::TopologyPolicy::ManifoldTwoSided;
  const startup::ShellSideRole* primary_roles = nullptr;
  std::size_t primary_role_count = 0;
  // Reserved for the forthcoming explicit mixed profile. Existing profiles
  // remain unchanged; a supplied pointer alone does not broaden admission.
  // It supplies true optional partner/origin/support maps with the same mains.
  const startup::Snapshot* mixed_snapshot = nullptr;
};
struct Input {
  Profile profile=Profile::Unspecified;
  normal_activation::FreeRosterPolicy free_roster=normal_activation::FreeRosterPolicy::Unspecified;
  Topology topology;
  VectorView positions;
  startup::Coordinates coordinates=startup::Coordinates::Native;
  UnitScale units; // Explicit SI boundary only; the normal arithmetic is native REAL4.
  const double* main_coefficients=nullptr;std::size_t coefficient_count=0;
  const std::uint32_t* main_active=nullptr;std::size_t active_count=0;
  const std::uint32_t* node_tag=nullptr;std::size_t tag_count=0;
  const std::uint32_t* free_main_ids=nullptr;std::size_t free_count=0;
  // Persistent accepted NOD_NORMAL, including defined unused T3 slot bits.
  const StoredNormal* prior_normals=nullptr;std::size_t prior_count=0;
};
struct Output {
  StoredNormal* face_normals=nullptr;std::size_t normal_count=0;
  startup::NormalReference* references=nullptr;std::size_t reference_count=0;
};
struct Limits {
  std::size_t nodes=1048576,primaries=1048576,references=8388608,incidences=8388608;
  std::size_t scratch_bytes=std::size_t{1}<<30;
};
struct Forecast {std::size_t scratch_bytes=0,output_bytes=0;};
struct Report {Status status=Status::InvalidInput;std::size_t main=SIZE_MAX,reference=SIZE_MAX,node=SIZE_MAX;};
// Allocation-free host value producer (declared here; staged implementation in
// the owning module). CUDA kernels share its private per-item arithmetic.
Report Preflight(const Input&,Limits,Forecast&) noexcept;
Report Evaluate(const Input&,Limits,void* scratch,std::size_t scratch_bytes,Output) noexcept;
} // namespace tlfea::contact::radioss_type25::current_normals
