// SPDX-License-Identifier: AGPL-3.0-or-later
// Native mass/reference values are produced by the qualified QEPH and T3
// startup operations. This module adds only immutable identity and reduction.
#pragma once

#include "ShellCollectionLimits.h"
#include "qeph/QephData.h"
#include "t3/T3Data.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace tl::fea {
// Legacy pair bound, also used by the unchanged resident publication scratch.
constexpr std::size_t MaxShellBindingNodes=7;
constexpr std::size_t NoShellBindingNode=std::numeric_limits<std::size_t>::max();
enum class ShellBindingFamily { None,Qeph,T3 };
enum class ShellBindingStatus {
  Success,AlreadyInitialized,InvalidInput,InvalidConnectivity,
  InvalidQephReference,InvalidT3Reference,IdentityMismatch,PositionMismatch,
  NonfiniteMass,InvalidParentIdentity,
};
struct ShellBindingReport {
  ShellBindingStatus status=ShellBindingStatus::Success;
  ShellBindingFamily family=ShellBindingFamily::None;
  std::size_t local_node=NoShellBindingNode,global_node=NoShellBindingNode;
  std::size_t parent_index=NoShellBindingNode; // Within the reported family.
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
// Borrowed input ranges are read only during Initialize. A zero count requires
// a null pointer. At least one family is present; combined count is bounded.
// Collection parent IDs must be nonzero and unique across BOTH families.
// Native connectivity/order and source node IDs are retained without sorting.
struct ShellQephBindingInput {
  qeph::ReferenceInput reference;
  std::array<std::size_t,4> nodes{};
  std::uint64_t source_parent_id=0;
};
struct ShellT3BindingInput {
  t3::ReferenceInput reference;
  std::array<std::size_t,3> nodes{};
  std::uint64_t source_parent_id=0;
};
struct ShellBatchCollectionInput {
  const ShellQephBindingInput* qeph=nullptr;
  const ShellT3BindingInput* t3=nullptr;
  std::size_t qeph_count=0,t3_count=0,node_count=0;
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
  static constexpr std::size_t WordCount=49; // Unchanged legacy pair encoding.
  static constexpr std::size_t Capacity=4+27*MaxShellCollectionParents;
  class WordView {
   public:
    const std::uint64_t* data() const noexcept { return data_; }
    std::size_t size() const noexcept { return size_; }
    const std::uint64_t* begin() const noexcept { return data_; }
    const std::uint64_t* end() const noexcept { return data_+size_; }
    const std::uint64_t& operator[](std::size_t i) const noexcept { return data_[i]; }
   private:
    WordView(const std::uint64_t* data,std::size_t size):data_(data),size_(size) {}
    const std::uint64_t* data_;
    std::size_t size_;
    friend class ShellBatchInventory;
  };
  WordView words() const noexcept { return {words_.data(),word_count_}; }
  bool operator==(const ShellBatchInventory& other) const noexcept {
    return word_count_==other.word_count_&&words_==other.words_;
  }
  bool operator!=(const ShellBatchInventory& other) const noexcept { return !(*this==other); }
 private:
  std::array<std::uint64_t,Capacity> words_{}; // Unused words always zero.
  std::size_t word_count_=0;
  friend class ShellBatchBinding;
};

// Immutable host startup collection. Fresh qualified producers validate every
// reference. Shared nodes require identical source IDs AND coordinate bits,
// including signed zero. Contributions are reduced QEPH then T3, in input
// parent/local order, with no normalization or total-inertia recombination.
// All failures preserve this object's bytes; successful preparation copies all
// inputs. Fixed storage, no allocation, constraints, clock or dynamics policy.
class ShellBatchBinding {
 public:
  ShellBatchBinding()=default;
  ShellBatchBinding(const ShellBatchBinding&)=default;
  ShellBatchBinding& operator=(const ShellBatchBinding&)=delete;
  // Preserves the exact original pair inventory and 4..7-node input contract.
  ShellBindingReport Initialize(const ShellBatchBindingInput& input) noexcept;
  ShellBindingReport Initialize(const ShellBatchCollectionInput& input) noexcept;
  bool prepared() const noexcept { return prepared_; }
  std::size_t node_count() const noexcept { return data_.node_count; }
  std::size_t qeph_count() const noexcept { return data_.qeph_count; }
  std::size_t t3_count() const noexcept { return data_.t3_count; }
  // Callers check counts before indexing. Invalid indices return immutable
  // empty/unprepared values; they never silently select the first element.
  const qeph::ReferenceData& qeph_reference(std::size_t i) const noexcept;
  const t3::ReferenceData& t3_reference(std::size_t i) const noexcept;
  const std::array<std::size_t,4>& qeph_nodes(std::size_t i) const noexcept;
  const std::array<std::size_t,3>& t3_nodes(std::size_t i) const noexcept;
  std::uint64_t qeph_source_id(std::size_t i) const noexcept;
  std::uint64_t t3_source_id(std::size_t i) const noexcept;
  // Compatibility accessors require exactly one member of each family;
  // otherwise return the same empty values. Source IDs are zero for old input.
  const qeph::ReferenceData& qeph_reference() const noexcept;
  const t3::ReferenceData& t3_reference() const noexcept;
  const std::array<std::size_t,4>& qeph_nodes() const noexcept;
  const std::array<std::size_t,3>& t3_nodes() const noexcept;
  const std::array<ShellBindingNode,MaxShellCollectionNodes>& nodes() const noexcept { return data_.nodes; }
  const ShellBindingMass& totals() const noexcept { return data_.totals; }
  const ShellBatchInventory& inventory() const noexcept { return data_.inventory; }
 private:
  template<class Reference,std::size_t N> struct Parent {
    Reference reference;
    std::array<std::size_t,N> nodes{};
    std::uint64_t source_id=0;
  };
  struct Data {
    std::array<Parent<qeph::ReferenceData,4>,MaxShellCollectionParents> qeph{};
    std::array<Parent<t3::ReferenceData,3>,MaxShellCollectionParents> t3{};
    std::array<ShellBindingNode,MaxShellCollectionNodes> nodes{};
    ShellBindingMass totals;
    ShellBatchInventory inventory;
    std::size_t qeph_count=0,t3_count=0,node_count=0;
  } data_;
  ShellBindingReport InitializeImpl(const ShellBatchCollectionInput&,bool legacy) noexcept;
  bool prepared_=false;
};
} // namespace tl::fea
