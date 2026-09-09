// SPDX-License-Identifier: AGPL-3.0-or-later
// Native mass/reference values are produced by the qualified QEPH and T3
// startup operations. This module adds only immutable identity and reduction.
#pragma once

#include "qeph/QephData.h"
#include "t3/T3Data.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace tl::fea {
constexpr std::size_t MaxShellBindingNodes=7; // One native Q4 plus one native T3.
constexpr std::size_t NoShellBindingNode=std::numeric_limits<std::size_t>::max();
enum class ShellBindingFamily { None,Qeph,T3 };
enum class ShellBindingStatus {
  Success,AlreadyInitialized,InvalidInput,InvalidConnectivity,
  InvalidQephReference,InvalidT3Reference,IdentityMismatch,PositionMismatch,
  NonfiniteMass,
};
struct ShellBindingReport {
  ShellBindingStatus status=ShellBindingStatus::Success;
  ShellBindingFamily family=ShellBindingFamily::None;
  std::size_t local_node=NoShellBindingNode,global_node=NoShellBindingNode;
  qeph::Status qeph_status=qeph::Status::kSuccess;
  t3::Status t3_status=t3::Status::kSuccess;
  const char* message="";
};

// Dense global indices [0,node_count), every index covered, no repeated index
// within a native cell. Source IDs belong to the native typed inputs; QEPH's
// uint32 IDs are widened, never used to truncate a T3 uint64 ID. Geometry may
// overlap without sharing an index: this startup object infers no attachment.
struct ShellBatchBindingInput {
  qeph::ReferenceInput qeph;
  t3::ReferenceInput t3;
  std::array<std::size_t,4> qeph_nodes{};
  std::array<std::size_t,3> t3_nodes{};
  std::size_t node_count=0;
};
struct ShellBindingMass {
  double mass=0;                // kg
  double isotropic_inertia=0;   // Native TOTAL J, kg*m^2; never recombined.
  double physical_inertia=0;    // Native thickness partition, kg*m^2.
  double added_inertia=0;       // Native area-added partition, kg*m^2.
};
struct ShellBindingNode {
  std::uint64_t source_id=0;
  tl::math::Vec3 position;      // Exact represented reference coordinate.
  ShellBindingMass native;
};

// Complete fixed-word identity encoding, NOT a persisted schema, reduced hash
// or numerical equivalence test. Includes family, arity, ordered connectivity,
// exact source IDs and every input coordinate/material binary64 bit pattern.
// Future participants must compare the entire inventory and actual binding;
// these words confer neither nodal-owner nor publication authority.
class ShellBatchInventory {
 public:
  static constexpr std::size_t WordCount=49;
  const std::array<std::uint64_t,WordCount>& words() const noexcept { return words_; }
  bool operator==(const ShellBatchInventory& other) const noexcept { return words_==other.words_; }
  bool operator!=(const ShellBatchInventory& other) const noexcept { return !(*this==other); }
 private:
  std::array<std::uint64_t,WordCount> words_{};
  friend class ShellBatchBinding;
};

// Host startup only, one QEPH Q4 and one native T3, 4..7 covered nodes. Fresh
// qualified producers validate the references; no independently mutable
// "validated" records are accepted. Shared nodes require identical source IDs
// AND coordinate bits, including signed zero. Native contributions are reduced
// QEPH first, then T3, each in local connectivity order, with no normalization.
// All failure paths preserve this object's bytes. Initialize is one-shot and
// publishes only after both references, identities and all sums have passed.
// No allocation, inverse-mass policy, constraints, clock, force/history storage,
// contact, dynamics admission or joint batch publication is implemented here.
class ShellBatchBinding {
 public:
  ShellBatchBinding()=default;
  ShellBatchBinding(const ShellBatchBinding&)=default;
  ShellBatchBinding& operator=(const ShellBatchBinding&)=delete;
  ShellBindingReport Initialize(const ShellBatchBindingInput& input) noexcept;
  bool prepared() const noexcept { return prepared_; }
  std::size_t node_count() const noexcept { return data_.node_count; }
  const qeph::ReferenceData& qeph_reference() const noexcept { return data_.qeph; }
  const t3::ReferenceData& t3_reference() const noexcept { return data_.t3; }
  const std::array<std::size_t,4>& qeph_nodes() const noexcept { return data_.qeph_nodes; }
  const std::array<std::size_t,3>& t3_nodes() const noexcept { return data_.t3_nodes; }
  const std::array<ShellBindingNode,MaxShellBindingNodes>& nodes() const noexcept { return data_.nodes; }
  // Totals use the same seven-contribution order, not a differently ordered
  // sum over global nodes. T3's measured angle sum is retained unchanged.
  const ShellBindingMass& totals() const noexcept { return data_.totals; }
  const ShellBatchInventory& inventory() const noexcept { return data_.inventory; }
 private:
  struct Data {
    qeph::ReferenceData qeph;
    t3::ReferenceData t3;
    std::array<std::size_t,4> qeph_nodes{};
    std::array<std::size_t,3> t3_nodes{};
    std::array<ShellBindingNode,MaxShellBindingNodes> nodes{};
    ShellBindingMass totals;
    ShellBatchInventory inventory;
    std::size_t node_count=0;
  } data_;
  bool prepared_=false;
};
} // namespace tl::fea
