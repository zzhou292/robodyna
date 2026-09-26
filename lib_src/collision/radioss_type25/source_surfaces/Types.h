// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_utils/BoundedArena.h"
#include <cstddef>
#include <cstdint>
namespace tlfea::contact::radioss_type25::source_surfaces {
enum class Status { Ok, InvalidInput, UnsupportedProfile, ResourceLimit, UnsupportedArithmetic };
enum class ReaderPhase { Unspecified, BeforeGroupingAndInitia };
enum class SolidTopology { Hex8, DeclaredPenta6 };
enum class ParentKind { Solid, ShellQuad, ShellTriangle };
enum class ClauseKind { Parts, Solids };
enum class SurfaceMode { Exterior=1, ExteriorShellEdges=2, All=3 };
struct Solid {
  std::uint64_t element_id=0, part_id=0;
  SolidTopology topology=SolidTopology::Hex8;
  std::uint32_t nodes[8]{}; // Reader raw8, including PENTA slots4/8 repeats.
};
struct Shell {
  std::uint64_t element_id=0, part_id=0;
  std::uint32_t nodes[4]{}; // T3 repeats third in fourth; incidence uses3slots.
};
struct Clause {
  ClauseKind kind=ClauseKind::Parts;
  SurfaceMode mode=SurfaceMode::Exterior;
  const std::uint64_t* part_ids=nullptr;
  std::size_t part_count=0;
  // SOLID clause only: original zero-based reader table indices, strictly
  // increasing and unique, as selected CREATE_ELEMENT_FROM_PART/order output.
  const std::uint32_t* solid_rows=nullptr;
  std::size_t solid_row_count=0;
  bool reverse_shell_normals=false;
};
struct Input {
  ReaderPhase phase=ReaderPhase::Unspecified;
  std::size_t node_count=0;
  // Each table is in its explicit early native reader storage order, not
  // mechanics family grouping or later SGR/CGR order. All physical rows stay
  // available for incidence and first matching shell suppression.
  const Solid* solids=nullptr;
  std::size_t solid_count=0;
  const Shell* quads=nullptr;
  std::size_t quad_count=0;
  const Shell* triangles=nullptr;
  std::size_t triangle_count=0;
  Clause clause;
};
struct FaceIdentity {
  ParentKind kind=ParentKind::Solid;
  std::uint64_t element_id=0, part_id=0;
  std::uint32_t reader_row=0;
  // Native FACES ordinal1..6 for solid,0for shell. This is a separate local
  // face identity, NEVER a fabricated source EID or a material/support owner.
  std::uint8_t solid_face=0;
};
struct Face {
  FaceIdentity source;
  std::uint32_t nodes[4]{};
  int raw_role=0; // Native SURF_ELTYP1/3/7, before I25SURFI/IN24/SH2.
  std::uint32_t buffer_ordinal=0; // Original solid→Q4→T3 insertion ordinal.
};
struct Counts {
  std::size_t selected_solids=0, selected_quads=0, selected_triangles=0;
  std::size_t internal_faces=0, degenerate_faces=0, shell_suppressed_faces=0;
  std::size_t solid_faces=0, shell_faces=0;
};
struct Snapshot {
  const Face* faces=nullptr; // Exact CREATE_SURFACE five-word sorted order.
  std::size_t face_count=0;
  // Source FLAG_ELEM_INTER25 input disposition:1only when at least one solid
  // segment was actually emitted. Not the broader selected-solid tag.
  const std::uint8_t* surface_solid_flags=nullptr;
  std::size_t solid_count=0;
  Counts counts;
};
struct Limits {
  std::size_t nodes=1048576, solids=65536, shells=1048576, parts=65536;
  std::size_t faces=1572864;
  std::size_t output_bytes=std::size_t{256}<<20, scratch_bytes=std::size_t{256}<<20;
};
struct Forecast {
  std::size_t output_bytes=0, scratch_bytes=0, maximum_faces=0;
  std::size_t solid_incidence=0, quad_incidence=0, triangle_incidence=0;
};
struct Report {
  Status status=Status::InvalidInput;
  std::size_t row=SIZE_MAX, node=SIZE_MAX;
  std::size_t required_faces=0;
  bool count_complete=false;
};
}
