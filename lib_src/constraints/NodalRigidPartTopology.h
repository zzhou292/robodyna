// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>

namespace tl::fea::rigid {
using SourceNodeId=std::uint64_t;
struct PartTopologyPartInput {
  std::uint64_t source_part_id=0;
  const SourceNodeId* nodes=nullptr;
  std::size_t node_count=0;
};
struct PartTopologyExtraInput {
  std::uint64_t source_part_id=0,source_node_set_id=0;
  const SourceNodeId* nodes=nullptr;
  std::size_t node_count=0;
};
struct PartTopologyMerge {
  std::uint64_t parent_part_id=0,child_part_id=0;
};
struct PartTopologyLimits {
  std::size_t max_parts=1024,max_members=16384,max_other_rigid_members=16384;
  std::size_t max_host_bytes=8*1024*1024;
};
struct PartTopologyInput {
  std::uint64_t source_instance_id=0;
  const PartTopologyPartInput* parts=nullptr;
  std::size_t part_count=0;
  const PartTopologyExtraInput* extras=nullptr;
  std::size_t extra_count=0;
  const PartTopologyMerge* merges=nullptr;
  std::size_t merge_count=0;
  // Complete literal source inventories. Expected contains exactly this model's
  // original members; other contains every declared plain-rigid member once.
  const SourceNodeId* expected_members=nullptr;
  std::size_t expected_member_count=0;
  const SourceNodeId* other_rigid_members=nullptr;
  std::size_t other_rigid_member_count=0;
  PartTopologyLimits limits{};
};
struct PartTopologyPart {
  std::uint64_t source_part_id=0;
  std::size_t member_offset=0,member_count=0,extra_row=SIZE_MAX,root_index=SIZE_MAX;
};
struct PartTopologyExtra {
  std::uint64_t source_node_set_id=0;
  std::size_t part_index=0,member_offset=0,member_count=0;
};
struct PartTopologyRoot {
  std::size_t part_index=0,child_part_index=SIZE_MAX;
  std::size_t member_offset=0,member_count=0,original_primary_count=1;
};
enum class PartTopologyStatus {
  Success,AlreadyInitialized,InvalidInput,ResourceLimit,DuplicateIdentity,
  DuplicateMembership,MissingMember,UnsupportedMerge
};
struct PartTopologyReport {
  PartTopologyStatus status=PartTopologyStatus::Success;
  const char* message="";
  std::size_t row=SIZE_MAX,member=SIZE_MAX;
  explicit operator bool() const noexcept {return status==PartTopologyStatus::Success;}
};

// Immutable literal source topology, not a source authenticator or state owner.
// Keeps supplied native member order and each original PART/extra/merge record.
// This version admits disjoint one-child merges only; chains and multiple-child
// roots are rejected. No generated primary IDs, centroid, M/J or physics are
// invented. Failed Initialize leaves the object empty and retryable.
class NodalRigidPartTopology {
 public:
  NodalRigidPartTopology();
  ~NodalRigidPartTopology();
  NodalRigidPartTopology(const NodalRigidPartTopology&)=delete;
  NodalRigidPartTopology& operator=(const NodalRigidPartTopology&)=delete;
  PartTopologyReport Initialize(const PartTopologyInput&) noexcept;
  bool prepared() const noexcept;
  std::uint64_t source_instance_id() const noexcept;
  std::size_t part_count() const noexcept;
  std::size_t extra_count() const noexcept;
  std::size_t merge_count() const noexcept;
  std::size_t root_count() const noexcept;
  std::size_t member_count() const noexcept;
  std::size_t other_rigid_member_count() const noexcept;
  std::size_t owned_payload_bytes() const noexcept;
  std::size_t startup_payload_bytes() const noexcept;
  const PartTopologyPart* parts() const noexcept;
  const PartTopologyExtra* extras() const noexcept;
  const PartTopologyMerge* merges() const noexcept;
  const PartTopologyRoot* roots() const noexcept;
  const SourceNodeId* original_members() const noexcept;
  const SourceNodeId* root_members() const noexcept;
  const SourceNodeId* expected_members() const noexcept;
  const SourceNodeId* other_rigid_members() const noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace tl::fea::rigid
