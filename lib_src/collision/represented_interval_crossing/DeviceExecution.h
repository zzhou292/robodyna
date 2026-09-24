// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../RepresentedIntervalCrossing.h"

namespace tlfea::contact {
class RepresentedIntervalCrossingGpu;
namespace represented_interval_crossing {
namespace native_device { class Workspace; }

// Same retained CPU row layouts; no pointer-sized executor field is added to
// the native owner and its forecast does not change.
struct CanonicalPair {
  std::uint32_t first = 0, second = 0;
  std::size_t input_pair = SIZE_MAX;
  RepresentedIntervalPairKey key;
};
struct PairStatus { bool complete = false; };

// A synchronous lexical borrow, created only by the native owner after its
// complete path/identity/canonical-pair validation. Never retained by a device
// owner or exposed as caller-provided eligibility authority.
class AuthenticatedWork {
 public:
  AuthenticatedWork(const AuthenticatedWork&) = delete;
  AuthenticatedWork& operator=(const AuthenticatedWork&) = delete;
  const RepresentedTrianglePath* paths() const noexcept { return paths_; }
  std::size_t path_count() const noexcept { return path_count_; }
  const CanonicalPair* pairs() const noexcept { return pairs_; }
  std::size_t pair_count() const noexcept { return pair_count_; }
  RepresentedIntervalLimits limits() const noexcept { return limits_; }
  RepresentedIntervalResult* staging() const noexcept { return staging_; }
  PairStatus* status() const noexcept { return status_; }
 private:
  friend class ::tlfea::contact::RepresentedIntervalCrossing;
  AuthenticatedWork(const RepresentedTrianglePath* paths, std::size_t path_count,
      const CanonicalPair* pairs, std::size_t pair_count, RepresentedIntervalLimits limits,
      RepresentedIntervalResult* staging, PairStatus* status) noexcept
      : paths_(paths), path_count_(path_count), pairs_(pairs), pair_count_(pair_count),
        limits_(limits), staging_(staging), status_(status) {}
  const RepresentedTrianglePath* paths_;
  std::size_t path_count_;
  const CanonicalPair* pairs_;
  std::size_t pair_count_;
  RepresentedIntervalLimits limits_;
  RepresentedIntervalResult* staging_;
  PairStatus* status_;
};

// Private linkage seam keeps the ordinary native library CUDA-free. Only the
// concrete retained Workspace may construct this interface. No public callback
// or alternate caller predicate can be injected into native certification.
class DeviceExecution {
 private:
  friend class ::tlfea::contact::RepresentedIntervalCrossing;
  friend class native_device::Workspace;
  DeviceExecution() = default;
  virtual ~DeviceExecution() = default;
  virtual bool Disjoint(const void*, std::size_t) const noexcept = 0;
  virtual RepresentedIntervalReport Execute(const AuthenticatedWork&) noexcept = 0;
};
class DeviceAccess {
 private:
  friend class ::tlfea::contact::RepresentedIntervalCrossingGpu;
  static RepresentedIntervalReport Certify(RepresentedIntervalCrossing&,
      const RepresentedTrianglePath*, std::size_t, const RepresentedTrianglePair*,
      std::size_t, DeviceExecution&) noexcept;
};
}  // namespace represented_interval_crossing
}  // namespace tlfea::contact
