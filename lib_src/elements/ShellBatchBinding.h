// SPDX-License-Identifier: AGPL-3.0-or-later
// Native mass/reference values are produced by qualified QEPH, T3 and QBAT
// startup operations. This module adds only immutable identity and reduction.
#pragma once

#include "ShellCollectionLimits.h"
#include "ShellHostBindingLimits.h"
#include "../../lib_utils/BoundedStartupArray.h"
#include "qeph/QephData.h"
#include "t3/T3Data.h"
#include "qbat/QbatTypes.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace tl::fea {
// Legacy pair bound, also used by the unchanged resident publication scratch.
constexpr std::size_t MaxShellBindingNodes=7;
constexpr std::size_t NoShellBindingNode=std::numeric_limits<std::size_t>::max();
enum class ShellBindingFamily { None,Qeph,T3,Qbat };
enum class ShellBindingStatus {
  Success,AlreadyInitialized,InvalidInput,InvalidConnectivity,
  InvalidQephReference,InvalidT3Reference,IdentityMismatch,PositionMismatch,
  NonfiniteMass,InvalidParentIdentity,ResourceLimit,InvalidQbatReference,
};
struct ShellBindingReport {
  ShellBindingStatus status=ShellBindingStatus::Success;
  ShellBindingFamily family=ShellBindingFamily::None;
  std::size_t local_node=NoShellBindingNode,global_node=NoShellBindingNode;
  std::size_t parent_index=NoShellBindingNode; // Within the reported family.
  qeph::Status qeph_status=qeph::Status::kSuccess;
  t3::Status t3_status=t3::Status::kSuccess;
  const char* message="";
  qbat::Status qbat_status=qbat::Status::kSuccess;
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
// The explicit formulation entry retains every distinct source layer, even
// when its physical node set equals another QEPH or QBAT layer's node set.
struct ShellQbatBindingInput {
  qbat::ReferenceInput reference;
  std::array<std::size_t,4> nodes{};
  std::uint64_t source_parent_id=0;
};
struct ShellFormulationCollectionInput {
  ShellBatchCollectionInput shells;
  const ShellQbatBindingInput* qbat=nullptr;
  std::size_t qbat_count=0;
};
struct ShellBindingMass {
  double mass=0;                // kg
  double isotropic_inertia=0;   // Native TOTAL J, kg*m^2; never recombined.
  // Native thickness/offset partition (Q4), thickness partition (T3), kg*m^2.
  double physical_inertia=0;
  double added_inertia=0;       // Native area-added partition, kg*m^2.
};
struct ShellBindingNode {
  std::uint64_t source_id=0;
  tl::math::Vec3 position;      // Exact represented reference coordinate.
  ShellBindingMass native;
};

// Complete fixed-word identity encoding, NOT a persisted schema, reduced hash
// or numerical equivalence test. Includes family, arity, ordered connectivity,
// exact source IDs, every input coordinate/material binary64 bit pattern,
// and an explicit typed placement word after each parent's material words.
// The named formulation entry also retains every QBAT option and initial A11.
// Future participants must compare the entire inventory and actual binding;
// these words confer neither nodal-owner nor publication authority.
class ShellBatchInventory {
 public:
  ShellBatchInventory()=default;
  ShellBatchInventory(const ShellBatchInventory&) noexcept=default;
  ShellBatchInventory(ShellBatchInventory&& other) noexcept
      :ShellBatchInventory(static_cast<const ShellBatchInventory&>(other)) {}
  ShellBatchInventory& operator=(const ShellBatchInventory&) noexcept=default;
  static constexpr std::size_t WordCount=51; // Version 3 pair encoding, including placement.
  static constexpr std::size_t Capacity=4+28*MaxShellCollectionParents; // Inline capacity only.
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
    if(word_count_!=other.word_count_) return false;
    for(std::size_t i=0;i<word_count_;++i) if(words_[i]!=other.words_[i]) return false;
    return true;
  }
  std::size_t backing_bytes() const noexcept { return words_.backing_bytes(); }
  bool operator!=(const ShellBatchInventory& other) const noexcept { return !(*this==other); }
 private:
  tl::util::BoundedStartupArray<std::uint64_t,Capacity> words_; // Unused words always zero.
  std::size_t word_count_=0;
  friend class ShellBatchBinding;
};

// Immutable host startup collection. Fresh qualified producers validate every
// reference. Shared nodes require identical source IDs AND coordinate bits,
// including signed zero. Contributions are reduced QEPH, T3, then optional
// QBAT, in input parent/local order, with no normalization or total-inertia
// recombination. Distinct source layers with identical node sets are retained.
// All failures preserve this object's bytes; successful preparation copies all
// inputs. Legacy default admission uses allocation-free inline storage. Explicit
// host limits and the named formulation entry permit startup-only owned
// allocation; publication/copies allocate nothing.
// Vehicle() opts into binding-only 524288-parent/node capacity. The existing
// catalog and device participants require independent admission. Startup scratch
// is bounded separately and released before Initialize returns.
// No constraints, clock or dynamics policy are added.
class ShellBatchBinding {
 public:
  ShellBatchBinding()=default;
  ShellBatchBinding(const ShellBatchBinding&) noexcept=default;
  // Moving an immutable published handle also leaves the source usable.
  ShellBatchBinding(ShellBatchBinding&& other) noexcept
      :ShellBatchBinding(static_cast<const ShellBatchBinding&>(other)) {}
  ShellBatchBinding& operator=(const ShellBatchBinding&)=delete;
  // Retains the original pair input and 4..7-node contract; inventory version 3.
  ShellBindingReport Initialize(const ShellBatchBindingInput& input) noexcept;
  ShellBindingReport Initialize(const ShellBatchCollectionInput& input) noexcept;
  ShellBindingReport Initialize(const ShellBatchCollectionInput&,const ShellHostBindingLimits&) noexcept;
  // Requires QBAT and complete QEPH/T3/QBAT global coverage. This host value
  // confers no QBAT catalog/resident/publication admission. Legacy initializers
  // retain their original input shape and inventory encoding.
  ShellBindingReport InitializeFormulations(const ShellFormulationCollectionInput&,
      const ShellHostBindingLimits& limits={}) noexcept;
  // Inline object + complete owned backing + reserved shared-control bytes.
  // Shared backing is charged in full per handle; this is not process RSS.
  std::size_t host_bytes() const noexcept;
  bool prepared() const noexcept { return prepared_; }
  std::size_t node_count() const noexcept { return data_.node_count; }
  std::size_t qeph_count() const noexcept { return data_.qeph_count; }
  std::size_t t3_count() const noexcept { return data_.t3_count; }
  std::size_t qbat_count() const noexcept { return data_.qbat_count; }
  std::size_t startup_scratch_bytes() const noexcept;
  // Callers check counts before indexing. Invalid indices return immutable
  // empty/unprepared values; they never silently select the first element.
  const qeph::ReferenceData& qeph_reference(std::size_t i) const noexcept;
  const t3::ReferenceData& t3_reference(std::size_t i) const noexcept;
  const qbat::Reference& qbat_reference(std::size_t i) const noexcept;
  const std::array<std::size_t,4>& qeph_nodes(std::size_t i) const noexcept;
  const std::array<std::size_t,3>& t3_nodes(std::size_t i) const noexcept;
  const std::array<std::size_t,4>& qbat_nodes(std::size_t i) const noexcept;
  std::uint64_t qeph_source_id(std::size_t i) const noexcept;
  std::uint64_t t3_source_id(std::size_t i) const noexcept;
  std::uint64_t qbat_source_id(std::size_t i) const noexcept;
  // Compatibility accessors require exactly one QEPH and T3, with no QBAT;
  // otherwise return the same empty values. Source IDs are zero for old input.
  const qeph::ReferenceData& qeph_reference() const noexcept;
  const t3::ReferenceData& t3_reference() const noexcept;
  const std::array<std::size_t,4>& qeph_nodes() const noexcept;
  const std::array<std::size_t,3>& t3_nodes() const noexcept;
  using NodeView=tl::util::ConstView<ShellBindingNode>;
  // Inline storage retains 128 zero-padded entries for compatibility. Expanded
  // storage exposes its active extent; iterate active_nodes() for either form.
  NodeView nodes() const noexcept { return {data_.nodes.data(),data_.nodes.size()}; }
  NodeView active_nodes() const noexcept { return {data_.nodes.data(),node_count()}; }
  const ShellBindingMass& totals() const noexcept { return data_.totals; }
  // Attribution only: already included in nodes().native and totals().
  const ShellBindingMass& qbat_totals() const noexcept { return data_.qbat_totals; }
  const ShellBatchInventory& inventory() const noexcept { return data_.inventory; }
 private:
  template<class Reference,std::size_t N> struct Parent {
    Reference reference;
    std::array<std::size_t,N> nodes{};
    std::uint64_t source_id=0;
  };
  struct Data {
    tl::util::BoundedStartupArray<Parent<qeph::ReferenceData,4>,MaxShellCollectionParents> qeph;
    tl::util::BoundedStartupArray<Parent<t3::ReferenceData,3>,MaxShellCollectionParents> t3;
    tl::util::BoundedStartupArray<ShellBindingNode,MaxShellCollectionNodes> nodes;
    ShellBindingMass totals;
    ShellBatchInventory inventory;
    std::size_t qeph_count=0,t3_count=0,node_count=0;
    tl::util::BoundedStartupArray<Parent<qbat::Reference,4>,0> qbat;
    std::size_t qbat_count=0;
    ShellBindingMass qbat_totals;
  } data_;
  ShellBindingReport InitializeImpl(const ShellBatchCollectionInput&,bool legacy,
      const ShellHostBindingLimits&,bool expanded,
      const ShellQbatBindingInput* qbat=nullptr,std::size_t qbat_count=0) noexcept;
  ShellBindingReport Build(const ShellBatchCollectionInput&,bool legacy,
      const ShellQbatBindingInput*,std::size_t qbat_count);
  bool prepared_=false;
};
} // namespace tl::fea
