// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>

namespace tl::constraints::tied_shell {
template<class T> struct ClassificationView {
  const T* data=nullptr;
  std::size_t count=0;
};
using ClassificationNodes=ClassificationView<std::uint32_t>;

// The five native IKINE blocks, gathered by node. Directions use the original
// decimal encoding: low digit is the three-DOF mask, remaining digits the skew.
struct NativeKinematics {
  std::int32_t conditions=0, translation=0, rotation=0;
  std::int32_t duplicate_conditions=0, incompatible_conditions=0;
};
struct ClassificationNode {
  std::uint32_t source_id=0;
  NativeKinematics kinematics{};
};
struct ClassificationInterface {
  std::uint32_t source_id=0;
  std::int32_t native_type=2, level=28;
  ClassificationNodes slaves, masters;
};
struct ClassificationSection {
  std::int32_t native_type=0;
  ClassificationNodes nodes;
};
struct ClassificationTetraEdge {
  std::uint32_t midpoint=0, first_corner=0, second_corner=0;
};
struct ClassificationRigidGroup {
  std::uint32_t source_id=0;
  ClassificationNodes members;
};
struct ClassificationLimits {
  std::size_t max_nodes=1u<<20, max_interfaces=4096, max_roles=1u<<20;
  std::size_t max_occurrences=4u<<20, max_host_bytes=256u<<20;
};
struct ClassificationContext {
  // Caller association only: this block does not authenticate source files or
  // establish that any omitted native condition is absent.
  std::uint64_t source_instance_id=0;
  ClassificationView<ClassificationNode> nodes;
};
struct ClassificationInput {
  ClassificationContext context;
  ClassificationView<ClassificationInterface> interfaces;
  ClassificationView<ClassificationSection> sections;
  // Empty means NBCSCYC=0. Otherwise exactly one nonnegative native tag/node.
  ClassificationView<std::int32_t> cyclic_tags;
  ClassificationView<ClassificationTetraEdge> tetra_edges;
  // Native signed ITAGND indices are one-based; zero means no midpoint tag.
  // Required for every node when tetra_edges is nonempty.
  ClassificationView<std::int32_t> tetra_tags;
  ClassificationNodes rbe2_nodes;
  ClassificationView<ClassificationNodes> rbe3_members;
};
struct RigidRegistrationInput {
  ClassificationContext context;
  ClassificationView<ClassificationRigidGroup> groups;
  std::int32_t native_iddlevel=0, native_ikrem=0;
};
enum class ClassificationStatus {
  Success, InvalidInput, ResourceLimit, DuplicateIdentity, NativeDomain
};
struct ClassificationReport {
  ClassificationStatus status=ClassificationStatus::Success;
  std::size_t row=SIZE_MAX, member=SIZE_MAX;
  explicit operator bool() const noexcept { return status==ClassificationStatus::Success; }
};
enum class ClassificationPhase {
  Empty, RigidMembersRegistered, InterfaceTaggedBeforeKinChk
};
struct ClassifiedInterface {
  std::uint32_t source_id=0;
  std::int32_t native_type=0, level=0;
  std::size_t slave_offset=0, slave_count=0;
  bool selected=false;
};

// Immutable supplied-context value result. No owner, mass, runtime state or
// original-source readiness is implied. IRUPT 0/1 is CIN/PEN only on selected
// interface rows; nonselected rows retain zero without being called CIN.
class ClassificationResult {
 public:
  ClassificationResult() noexcept=default;
  ClassificationResult(const ClassificationResult&) noexcept=default;
  ClassificationResult(ClassificationResult&&) noexcept=default;
  ClassificationResult& operator=(const ClassificationResult&) noexcept=default;
  ClassificationResult& operator=(ClassificationResult&&) noexcept=default;
  ClassificationPhase phase() const noexcept;
  std::uint64_t source_instance_id() const noexcept;
  ClassificationView<ClassificationNode> nodes() const noexcept;
  ClassificationView<ClassifiedInterface> interfaces() const noexcept;
  ClassificationNodes slave_nodes() const noexcept;
  ClassificationView<std::int32_t> irupt() const noexcept;
  ClassificationView<std::int32_t> interface_decode() const noexcept;
  std::uint64_t native_kinset_warnings() const noexcept;
  std::uint64_t native_penalty_warnings() const noexcept;
  std::size_t owned_payload_bytes() const noexcept;
  std::size_t startup_payload_bytes() const noexcept;
 private:
  struct Data;
  std::shared_ptr<const Data> data_;
  friend ClassificationReport Classify(const ClassificationInput&,ClassificationResult*,ClassificationLimits) noexcept;
  friend ClassificationReport RegisterRigidMembers(const RigidRegistrationInput&,ClassificationResult*,ClassificationLimits) noexcept;
};

// Fresh KININI decode tables per call; supplied context is the state before the
// selected caller. Source-order duplicates and ITF table mutations are retained.
// RegisterRigidMembers is one CHECKRBY scratch scope, after supplied hierarchy
// resolution. Other constraint importers and later KINCHK remain out of scope.
// Inputs may borrow an old result: publication happens only after all reads.
// Counts and budgets are checked before payload reads. Any failure leaves out
// unchanged. Byte limits cover payloads including the old result, not allocator
// bookkeeping. No allocation occurs in an already running dynamics interval.
ClassificationReport Classify(const ClassificationInput&,ClassificationResult*,ClassificationLimits={}) noexcept;
ClassificationReport RegisterRigidMembers(const RigidRegistrationInput&,ClassificationResult*,ClassificationLimits={}) noexcept;
} // namespace tl::constraints::tied_shell
