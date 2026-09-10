// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "qeph/QephBatch.h"
#include "t3/T3Batch.h"
#include <memory>

namespace tl::fea {
struct ShellBatchKinetic {
  double translation=0,rotation=0;
  // Native isotropic partitions (including drilling). Total rotation uses
  // native TOTAL J and is never reconstructed by summing these partitions.
  double physical_isotropic=0,added_isotropic=0;
};
struct ShellBatchDiagnostics {
  qeph::BatchDiagnostics qeph;
  t3::BatchDiagnostics t3;
  ShellBatchKinetic base_kinetic,kinetic;
  bool valid=false;
};
enum class ShellPublicationStatus {
  Success,InvalidInput,NotInitialized,NotJoined,StaleTrial,ResourceLimit,
  DeviceFailure,NodalFailure,NonfiniteResult,
};
struct ShellPublicationReport {
  ShellPublicationStatus status=ShellPublicationStatus::InvalidInput;
  const char* message="Invalid mixed publication request";
  NodalStatus nodal_status=NodalStatus::Ok;
};

// Closed two-family publication scope: matching PrescribedFields or
// CoupledForces usage. Coupled candidates require BOTH accepted caches to have
// contributed to this same owner's attempt. Both
// batches must be initialized with the SAME complete immutable binding and
// owner/configuration/qualification/usage, then perform initial rest/mass
// binding. The caller may evaluate QEPH and T3 in either order from the same
// authentic prepared token. Neither joined batch can publish by itself.
// Startup remains reference-at-rest. This transaction API does not select a
// stable timestep or qualify a general mixed-shell/contact trajectory.
//
// This object borrows the batches; both batches must outlive the
// coordinator (including destruction), and the owner must outlive its calls. It owns one bounded reusable kinetic scratch allocation and diagnostic
// caches, not nodal state, an independent clock, material histories or a solver.
// Calls are serialized on the sole owner's stream. No per-step allocation.
// Every Prepare/Commit failure discards that nodal trial
// and BOTH material trials; all accepted results and caller outputs survive
// numerical failure. A CUDA failure poisons the participants; readable-device
// recovery is not promised. Raw fabricated device writes are not authenticated.
class ShellBatchPublication {
 public:
  ShellBatchPublication();
  ~ShellBatchPublication();
  ShellBatchPublication(const ShellBatchPublication&)=delete;
  ShellBatchPublication& operator=(const ShellBatchPublication&)=delete;
  // Requires both initial rest caches already bound from this owner's actual
  // source buffers; claims one coordinator per participant. Duplicate or
  // foreign-source attachment fails before device allocation/publication.
  ShellPublicationReport Initialize(FENodalState&,qeph::QephBatch&,t3::T3Batch&);
  // Preflight BOTH completed contributors against the actual owner token
  // before GPU measurement. Kinetic energy is reduced over the complete native
  // union exactly once at each base/endpoint, at the declared velocity times.
  // Typed family kinetic_available=false and all their kinetic fields are zero.
  ShellPublicationReport Prepare(FENodalState&,const NodalTrialToken&,
      const qeph::BatchDiagnostics&,const t3::BatchDiagnostics&,ShellBatchDiagnostics*);
  // Repeat preflight of BOTH results, measured diagnostics and receipt; perform
  // one owner commit followed only by two infallible slab publications. No CUDA
  // call, readback, allocation or other fallible operation follows owner success.
  ShellPublicationReport Commit(FENodalState&,const NodalTrialToken&,
      const ShellBatchDiagnostics&,const NodalValidationReceipt&) noexcept;
  ShellPublicationReport CopyAcceptedDiagnostics(const NodalStamp&,ShellBatchDiagnostics*) const noexcept;
  // Discards coordinator and BOTH material scratch; caller still owns nodal
  // Discard when abandoning a trial outside Prepare/Commit.
  void DiscardTrial() noexcept;
  NodalAllocationInfo allocations() const noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace tl::fea
