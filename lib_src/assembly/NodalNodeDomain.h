// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../math/Fixed3.h"
#include "../../lib_utils/BoundedStartupArray.h"
#include <cstdint>
#include <memory>

namespace tl::fea {
struct NodalDomainNode {
  std::uint64_t source_id=0;
  tl::math::Vec3 position{}; // Exact represented SI coordinates, not a working-unit round trip.
};
struct NodalDomainInput {
  std::uint64_t source_instance_id=0;
  const NodalDomainNode* nodes=nullptr;
  std::size_t node_count=0;
};
struct NodalDomainLimits {
  std::size_t max_nodes=2048,max_host_bytes=1024*1024;
  static constexpr NodalDomainLimits Vehicle() noexcept {return {524288,128*1024*1024};}
};
enum class NodalDomainStatus {
  Success,AlreadyInitialized,InvalidInput,ResourceLimit,DuplicateIdentity,
  MissingSource,PositionMismatch
};
struct NodalDomainReport {
  NodalDomainStatus status=NodalDomainStatus::Success;
  const char* message="OK";
  std::size_t node=SIZE_MAX;
  explicit operator bool() const noexcept {return status==NodalDomainStatus::Success;}
};

// Declared physical-node identity only. Unique positive NIDs and finite positions
// are owned in caller order; equal positions do not merge distinct NIDs. This
// does not prove mechanical coverage, coefficients, DOFs or owner admission.
// Counts and complete bytes are checked before borrowed values. Failed startup
// preserves an empty handle; copies share immutable backing without allocation.
class NodalNodeDomain {
 public:
  NodalNodeDomain()=default;
  NodalNodeDomain(const NodalNodeDomain&) noexcept=default;
  NodalNodeDomain(NodalNodeDomain&& other) noexcept
      :NodalNodeDomain(static_cast<const NodalNodeDomain&>(other)) {}
  NodalNodeDomain& operator=(const NodalNodeDomain&)=delete;
  NodalDomainReport Initialize(const NodalDomainInput&,NodalDomainLimits={}) noexcept;
  bool prepared() const noexcept {return bool(impl_);}
  std::uint64_t source_instance_id() const noexcept;
  std::size_t node_count() const noexcept;
  tl::util::ConstView<NodalDomainNode> nodes() const noexcept;
  std::size_t Find(std::uint64_t source_id) const noexcept; // SIZE_MAX when absent.
  bool Matches(const NodalNodeDomain&) const noexcept;
  bool SharesStorage(const NodalNodeDomain&) const noexcept;
  std::size_t owned_payload_bytes() const noexcept;
  std::size_t startup_payload_bytes() const noexcept;
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
} // namespace tl::fea
