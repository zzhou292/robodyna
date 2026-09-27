// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/assembly/SolidNodeContributions.h"
#include "lib_src/elements/solid_common/distortion/UnitTypes.h"
namespace tl::fea::solids::control {
using Family=SolidCoefficientFamily;
using UnitScale=solid_common::distortion::UnitScale;
enum class Profile { LegacyNoStructuralIcontrol, SourceDeclared };
struct SourceParent {
  std::uint64_t element_id=0,part_id=0,section_id=0,material_id=0,native_property_id=0;
  std::uint32_t icontrol=0;
};
// Complete solid-only packet roster for each native processor partition. NFT
// is the post-W_IPARG solid-local offset. NG can include intervening other types.
struct NativePacket {
  std::uint64_t group_id=0;
  std::size_t native_first=0,member_begin=0,member_count=0;
  Family family=Family::Solid18;
  std::uint64_t material_id=0,native_property_id=0;
  std::uint32_t icontrol=0;
};
struct NativePartition {
  std::uint64_t partition_id=0; // Contiguous zero-based native PROC; empty partitions explicit.
  std::size_t packet_begin=0,packet_count=0,member_begin=0,member_count=0;
};
struct Input {
  Profile profile=Profile::LegacyNoStructuralIcontrol;
  std::uint64_t source_instance_id=0;
  UnitScale units{};
  std::uint32_t native_nvsiz=0,compiled_mvsiz=0;
  util::ConstView<SourceParent> parents{nullptr,0};
  util::ConstView<NativePartition> partitions{nullptr,0};
  util::ConstView<NativePacket> packets{nullptr,0};
  util::ConstView<std::uint64_t> ordered_element_ids{nullptr,0};
};
struct Parent {
  SourceParent source;
  Family family=Family::Solid18;
  std::size_t family_index=SIZE_MAX,packet_index=SIZE_MAX,packet_slot=SIZE_MAX;
};
struct Packet { NativePacket source;std::size_t partition_index=SIZE_MAX; };
struct Member {std::uint64_t element_id=0;Family family=Family::Solid18;std::size_t family_index=SIZE_MAX,parent_index=SIZE_MAX;};
struct Limits {
  std::size_t max_parents=16384,max_packets=16384,max_partitions=1024;
  std::size_t max_host_bytes=256u<<20;
};
struct Budget {std::size_t owned_bytes=0,startup_bytes=0;};
enum class Status { Success,AlreadyInitialized,InvalidInput,ResourceLimit,DuplicateIdentity,SourceMismatch,UnsupportedProfile };
struct Report {
  Status status=Status::Success;const char* message="OK";
  std::size_t parent=SIZE_MAX,packet=SIZE_MAX;
  explicit operator bool()const noexcept{return status==Status::Success;}
};
} // namespace tl::fea::solids::control
