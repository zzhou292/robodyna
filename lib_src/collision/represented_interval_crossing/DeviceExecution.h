// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../RepresentedIntervalCrossing.h"
#include "Batch.h"
#include "CanonicalPair.h"
#include <optional>
#include <type_traits>

namespace tlfea::contact {
class RepresentedIntervalCrossingGpu;
namespace represented_interval_crossing {
namespace native_device { class Workspace; }
class DeviceExecution;
struct DeviceBatchAccess;

// Same retained CPU row layouts; no pointer-sized executor field is added to
// the native owner and its forecast does not change.
struct PairStatus { bool complete = false; };

// A native-created window of the already validated canonical outer roster.
// Completion permits consuming numerical rows, never publishing a later slice
// ahead of the existing native work/failure fold.
class AuthenticatedNumericCohort {
 private:
  friend class ::tlfea::contact::RepresentedIntervalCrossing;
  friend class native_device::Workspace;
  struct ConstructionKey {
   private:
    friend class ::tlfea::contact::RepresentedIntervalCrossing;
    ConstructionKey() noexcept {}
  };
  static_assert(!std::is_aggregate_v<ConstructionKey>);
 public:
  AuthenticatedNumericCohort(ConstructionKey, std::size_t begin, std::size_t count) noexcept
      : begin_(begin), count_(count) {}
  AuthenticatedNumericCohort(const AuthenticatedNumericCohort&) = delete;
  AuthenticatedNumericCohort& operator=(const AuthenticatedNumericCohort&) = delete;
 private:
  const std::size_t begin_, count_;
  bool computed_ = false;
};

// Materialized once, only after native roster validation and slice admission.
// This noncopyable object lives inside the native lexical PathRoster. Device
// bytes may remain allocated after it dies, but no upload authority survives.
class AuthenticatedScene {
 private:
  friend class ::tlfea::contact::RepresentedIntervalCrossing;
  friend class native_device::Workspace;
  struct ConstructionKey {
   private:
    friend class ::tlfea::contact::RepresentedIntervalCrossing;
    // A defaulted private constructor can still leave a C++17 aggregate and
    // admit caller braced initialization. A user-provided constructor cannot.
    ConstructionKey() noexcept {}
  };
  static_assert(!std::is_aggregate_v<ConstructionKey>);
 public:
  AuthenticatedScene(ConstructionKey, DeviceExecution* executor,
      const RepresentedTrianglePath* paths, std::size_t count,
      const RepresentedTrianglePair* ordered_pairs = nullptr,
      std::size_t ordered_pair_count = 0, std::size_t slice_capacity = 0) noexcept
      : executor_(executor), paths_(paths), count_(count), ordered_pairs_(ordered_pairs),
        ordered_pair_count_(ordered_pair_count), slice_capacity_(slice_capacity) {}
  AuthenticatedScene(const AuthenticatedScene&) = delete;
  AuthenticatedScene& operator=(const AuthenticatedScene&) = delete;
  const RepresentedTrianglePath* paths() const noexcept { return paths_; }
  std::size_t count() const noexcept { return count_; }
 private:
  DeviceExecution* const executor_;
  const RepresentedTrianglePath* const paths_;
  const std::size_t count_;
  const RepresentedTrianglePair* const ordered_pairs_;
  const std::size_t ordered_pair_count_, slice_capacity_;
  std::optional<AuthenticatedNumericCohort> cohort_;
  bool uploaded_ = false;
};

// A synchronous lexical borrow, created only by the native owner after its
// complete path/identity/canonical-pair validation. Never retained by a device
// owner or exposed as caller-provided eligibility authority.
class AuthenticatedWork {
 public:
  AuthenticatedWork(const AuthenticatedWork&) = delete;
  AuthenticatedWork& operator=(const AuthenticatedWork&) = delete;
  const RepresentedTrianglePath* paths() const noexcept { return scene_.paths(); }
  std::size_t path_count() const noexcept { return scene_.count(); }
  const CanonicalPair* pairs() const noexcept { return pairs_; }
  std::size_t pair_count() const noexcept { return pair_count_; }
  RepresentedIntervalLimits limits() const noexcept { return limits_; }
  RepresentedIntervalResult* staging() const noexcept { return staging_; }
  PairStatus* status() const noexcept { return status_; }
 private:
  friend class ::tlfea::contact::RepresentedIntervalCrossing;
  friend class native_device::Workspace;
  AuthenticatedWork(AuthenticatedScene& scene,
      const CanonicalPair* pairs, std::size_t pair_count, RepresentedIntervalLimits limits,
      RepresentedIntervalResult* staging, PairStatus* status, std::size_t first_ordinal = 0) noexcept
      : scene_(scene), pairs_(pairs), pair_count_(pair_count),
        limits_(limits), staging_(staging), status_(status), first_ordinal_(first_ordinal) {}
  AuthenticatedScene& scene_;
  const CanonicalPair* pairs_;
  std::size_t pair_count_;
  RepresentedIntervalLimits limits_;
  RepresentedIntervalResult* staging_;
  PairStatus* status_;
  std::size_t first_ordinal_;
};

// Private linkage seam keeps the ordinary native library CUDA-free. Only the
// concrete retained Workspace may construct this interface. No public callback
// or alternate caller predicate can be injected into native certification.
class DeviceExecution {
 private:
  friend class ::tlfea::contact::RepresentedIntervalCrossing;
  friend struct BatchAccess;
  friend class native_device::Workspace;
  DeviceExecution() = default;
  virtual ~DeviceExecution() = default;
  virtual bool Disjoint(const void*, std::size_t) const noexcept = 0;
  virtual std::size_t NumericCohortCapacity() const noexcept = 0;
  virtual RepresentedIntervalReport Execute(const AuthenticatedWork&) noexcept = 0;
};
class DeviceAccess {
 private:
  friend class ::tlfea::contact::RepresentedIntervalCrossingGpu;
  friend struct DeviceBatchAccess;
  static RepresentedIntervalReport Certify(RepresentedIntervalCrossing&,
      const RepresentedTrianglePath*, std::size_t, const RepresentedTrianglePair*,
      std::size_t, DeviceExecution&) noexcept;
  static BatchReport CertifyBatch(RepresentedIntervalCrossing&,
      const RepresentedTrianglePath*, std::size_t, const RepresentedTrianglePair*,
      std::size_t, std::size_t, RepresentedIntervalResult*, std::size_t,
      DeviceExecution&) noexcept;
};
}  // namespace represented_interval_crossing
}  // namespace tlfea::contact
