// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidPartAssemblyModel.h"
#include "NodalRigidAssemblyTypes.h"

namespace tl::fea {
struct RigidBindingLimits {
  std::size_t max_groups=1024,max_members=16384,max_members_per_group=1024;
  std::size_t max_nodes=524288,max_host_bytes=1024*1024*1024;
};
enum class RigidBindingStatus {
  Success,AlreadyInitialized,InvalidInput,ResourceLimit,IdentityMismatch,
  MissingCoefficient,DuplicateMembership
};
struct RigidBindingReport {
  RigidBindingStatus status=RigidBindingStatus::Success;
  const char* message="OK";
  std::size_t group=SIZE_MAX,member=SIZE_MAX;
  explicit operator bool() const noexcept {return status==RigidBindingStatus::Success;}
};

// Immutable association of already-finalized PARTs and optional plain groups
// with the same physical ledger. No primary generation, tensor accumulation,
// principal correction, dynamics, allocation on stepping, or state owner.
// PART roots precede plain groups. Native plain-member order is unchanged;
// PARTs use their retained root-member order. Source kinds remain distinct even
// when numeric IDs coincide. Every declared other-rigid member must be supplied.
class NodalRigidAssemblyBinding {
 public:
  NodalRigidAssemblyBinding()=default;
  NodalRigidAssemblyBinding(const NodalRigidAssemblyBinding&) noexcept=default;
  NodalRigidAssemblyBinding(NodalRigidAssemblyBinding&& other) noexcept
      :NodalRigidAssemblyBinding(static_cast<const NodalRigidAssemblyBinding&>(other)) {}
  NodalRigidAssemblyBinding& operator=(const NodalRigidAssemblyBinding&)=delete;
  RigidBindingReport Initialize(const rigid::NodalRigidPartAssemblyModel&,
      const NodalRigidGroupModel* plain=nullptr,RigidBindingLimits={}) noexcept;
  bool prepared() const noexcept {return bool(impl_);}
  const rigid::NodalRigidPartAssemblyModel* parts() const noexcept;
  const NodalCoefficientLedger* coefficients() const noexcept;
  const NodalNodeDomain* domain() const noexcept;
  std::uint64_t plain_source_instance_id() const noexcept;
  tl::util::ConstView<RigidBindingGroup> groups() const noexcept;
  tl::util::ConstView<RigidBindingMember> members() const noexcept;
  const RigidBindingMember* FindMember(std::size_t domain_node) const noexcept;
  std::size_t owned_payload_bytes() const noexcept;
  std::size_t startup_payload_bytes() const noexcept;
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
} // namespace tl::fea
