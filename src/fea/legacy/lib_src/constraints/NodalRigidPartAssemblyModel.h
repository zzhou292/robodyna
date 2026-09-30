// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidPartTopology.h"
#include "NodalRigidAssemblyValues.h"
#include "../assembly/NodalCoefficientLedger.h"

namespace tl::fea::rigid {
enum class PartAssemblyOrder { OriginalNidPartThenExtrasSI };
enum class PartAssemblyStatus {
  Success,AlreadyInitialized,InvalidInput,ResourceLimit,IdentityMismatch,
  MissingCoefficient,NonfiniteResult,InertiaFailure
};
struct PartAssemblyReport {
  PartAssemblyStatus status=PartAssemblyStatus::Success;
  const char* message="OK";
  std::size_t part=SIZE_MAX,member=SIZE_MAX;
  explicit operator bool() const noexcept {return status==PartAssemblyStatus::Success;}
};
struct PartAssemblyLimits {
  std::size_t max_parts=1024,max_members=16384,max_host_bytes=1024*1024*1024;
};
struct PartAssemblyMember {
  std::size_t domain_node=SIZE_MAX,part_index=SIZE_MAX;
};
struct PartAssemblyOriginal {
  AssemblyPrimary primary{};
  AssemblyRawBody raw{};
};
struct PartAssemblyRoot { AssemblyFinalBody value{}; };

// Immutable startup aggregate, not an owner or proof of complete source M/J.
// Keeps the complete supplied ledger and literal topology. Within each PART
// and extra set, ascending original NID defines SI reductions and the PART-only
// primary mean. Native PART/extra/reset/merge/finalization phases stay distinct.
// This does not promise native member traversal, COM bits or eigenvector bits.
class NodalRigidPartAssemblyModel {
 public:
  NodalRigidPartAssemblyModel()=default;
  NodalRigidPartAssemblyModel(const NodalRigidPartAssemblyModel&) noexcept=default;
  NodalRigidPartAssemblyModel(NodalRigidPartAssemblyModel&& other) noexcept
      :NodalRigidPartAssemblyModel(static_cast<const NodalRigidPartAssemblyModel&>(other)) {}
  NodalRigidPartAssemblyModel& operator=(const NodalRigidPartAssemblyModel&)=delete;
  PartAssemblyReport Initialize(const NodalRigidPartTopology&,
      const NodalCoefficientLedger&,NodalRigidSourceUnits,PartAssemblyLimits={}) noexcept;
  bool prepared() const noexcept {return bool(impl_);}
  const NodalRigidPartTopology* topology() const noexcept;
  const NodalCoefficientLedger* coefficients() const noexcept;
  NodalRigidSourceUnits source_units() const noexcept;
  static constexpr PartAssemblyOrder order() noexcept {return PartAssemblyOrder::OriginalNidPartThenExtrasSI;}
  // Ranges use topology PART/extra offsets, but members inside each are sorted.
  tl::util::ConstView<PartAssemblyMember> members() const noexcept;
  tl::util::ConstView<PartAssemblyOriginal> original_bodies() const noexcept;
  tl::util::ConstView<PartAssemblyRoot> roots() const noexcept;
  std::size_t RootForDomainNode(std::size_t) const noexcept;
  bool Matches(const NodalRigidPartTopology&,const NodalCoefficientLedger&,
               NodalRigidSourceUnits) const noexcept;
  bool Matches(const NodalRigidPartAssemblyModel&) const noexcept;
  std::size_t owned_payload_bytes() const noexcept;
  std::size_t startup_payload_bytes() const noexcept;
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
} // namespace tl::fea::rigid
