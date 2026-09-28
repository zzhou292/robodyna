// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../runtime/Types.h"
#include "../startup/PostGapmTypes.h"
#include "lib_src/assembly/ShellPhysicalBinding.h"
#include <array>
namespace tlfea::contact::radioss_type25::activity_source {
// These categories describe the actual bound physical population, not a raw
// deck census. The source producer must separately authenticate its coverage.
enum class Family : std::uint32_t {
  Qeph, T3, Qbat, Solid18, Solid24, Solid6z, Solid18Law44,
  Solid18Law90, Beam18, Type25, Type13, Count
};
inline constexpr std::size_t FamilyCount=static_cast<std::size_t>(Family::Count);
struct ParentIdentity {
  Family family=Family::Qeph;
  std::uint32_t family_index=0;
  std::uint64_t source_element_id=0;
};
inline constexpr std::uint32_t NoParent=UINT32_MAX;
// Final native IELEM_M support, NOT the raw face-origin union. Shell supports
// have no second operand. Both solid operands retain their original order.
struct MainSupport { std::uint32_t first=NoParent,second=NoParent; };
struct Origin {
  std::uint32_t parent=NoParent,primary=0;
  std::uint8_t local_face=0;
};
enum class Deletion : unsigned char { Disabled=0, AllSupports=1, AnySupport=2 };
struct Controls {
  Deletion deletion=Deletion::Disabled;
  bool keep_disconnected_nodes=false;
  startup::SolidErosion solid_erosion=startup::SolidErosion::Unspecified;
};
struct Limits {
  std::size_t nodes=1048576,parents=1048576,mains=2097152,origins=1572864;
  std::size_t incidence=8388608,containing_parents=8388608;
  std::size_t output_bytes=128u<<20,startup_bytes=256u<<20;
};
struct Counts {
  std::size_t nodes=0,parents=0,mains=0,primaries=0,origins=0,incidence=0;
  std::size_t containing_capacity=0; // Conservative first-corner incidence bound.
  std::array<std::size_t,FamilyCount> families{};
};
struct Forecast {
  TransactionReport report;
  Counts counts;
  std::size_t output_bytes=0,startup_bytes=0;
};
struct View {
  tl::util::ConstView<ParentIdentity> parents;
  tl::util::ConstView<std::uint32_t> node_offsets,node_parents;
  tl::util::ConstView<MainSupport> mains;
  // Every eligible bound source parent containing ALL distinct primary corners.
  // These are independent of the final IELEM support operands above.
  tl::util::ConstView<std::uint32_t> main_to_primary,containing_offsets,containing_parents;
  tl::util::ConstView<Origin> origins;
  Controls controls;
  std::uint64_t source_generation=0;
};
} // namespace tlfea::contact::radioss_type25::activity_source
