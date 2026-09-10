// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../elements/ShellBatchBinding.h"
#include <memory>

namespace tl::fea {
namespace type25 { class Model; }
struct NodalMassPartitions {
  ShellBindingMass shell;
  double connector_mass=0,connector_inertia=0;
  // Authoritative totals accumulate contributions in their declared order.
  // Shell physical/added partitions never absorb connector regularizers.
  double mass=0,isotropic_inertia=0;
};
struct NodalMassNode {
  std::uint64_t source_id=0;
  tl::math::Vec3 position;
  NodalMassPartitions coefficients;
};
struct NodalMassLimits {
  std::size_t max_nodes=2048,max_host_bytes=8*1024*1024;
};
enum class NodalMassStatus {
  Success,AlreadyInitialized,InvalidInput,ResourceLimit,IdentityMismatch,
  PositionMismatch,NonfiniteResult
};
struct NodalMassReport {
  NodalMassStatus status=NodalMassStatus::InvalidInput;
  const char* message="Invalid combined nodal mass input";
  std::size_t node=SIZE_MAX,connection=SIZE_MAX;
  explicit operator bool() const noexcept { return status==NodalMassStatus::Success; }
};

// Immutable startup composition of qualified producer outputs. This is not a
// mass formula, owner, contributor registration or publication authority.
// Shell references retain complete node coverage. Each TYPE25 endpoint must
// match that same source node and exact reference coordinate bits. Contributions
// follow shell native assembly, then input connection/endpoint order. Copies
// share owned immutable inputs; failed initialization leaves this handle empty.
class NodalMassBinding {
 public:
  NodalMassBinding()=default;
  NodalMassBinding(const NodalMassBinding&) noexcept=default;
  NodalMassBinding(NodalMassBinding&& other) noexcept
      :NodalMassBinding(static_cast<const NodalMassBinding&>(other)) {}
  NodalMassBinding& operator=(const NodalMassBinding&)=delete;
  NodalMassReport Initialize(const ShellBatchBinding&,const type25::Model&,
                            const NodalMassLimits& limits={}) noexcept;
  bool prepared() const noexcept { return bool(impl_); }
  std::size_t node_count() const noexcept;
  std::uint64_t source_instance_id() const noexcept;
  std::size_t host_bytes() const noexcept;
  tl::util::ConstView<NodalMassNode> nodes() const noexcept;
  const NodalMassPartitions& totals() const noexcept;
  bool Matches(const ShellBatchBinding&) const noexcept;
  bool Matches(const type25::Model&) const noexcept;
  bool Matches(const NodalMassBinding&) const noexcept;
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
} // namespace tl::fea
