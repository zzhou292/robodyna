// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "qeph/QephBatch.h"
#include "t3/T3Batch.h"
#include "type25/Type25Batch.h"
#include "qbat/QbatBatch.h"
#include "ShellPhysicalPublication.h"
#include <memory>

namespace tl::fea {
struct ShellPublicationLimits {
  std::size_t max_nodes=MaxShellCollectionNodes;
  std::size_t max_device_bytes=128*1024,max_host_bytes=1024*1024;
  ShellResidentProfile profile=ShellResidentProfile::Legacy;
  static constexpr ShellPublicationLimits Vehicle() noexcept {
    return {MaxVehicleShellResidentNodes,32ULL*1024*1024,128ULL*1024*1024,ShellResidentProfile::Vehicle};
  }
};
struct ShellBatchKinetic {
  double translation=0,rotation=0;
  // Native isotropic partitions (including drilling). Total rotation uses
  // native TOTAL J and is never reconstructed by summing these partitions.
  double physical_isotropic=0,added_isotropic=0;
  // Explicit TYPE25 property partitions. Totals above use the authoritative
  // combined node coefficients, never a reconstruction from these subtotals.
  double connector_translation=0,connector_rotation=0;
};
struct ShellBatchDiagnostics {
  qeph::BatchDiagnostics qeph;
  t3::BatchDiagnostics t3;
  type25::BatchDiagnostics connector;
  ShellBatchKinetic base_kinetic,kinetic;
  bool valid=false,has_connector=false;
  // Trailing opt-in formulation channels preserve older positional members.
  // Absent families have canonical empty diagnostics, never phantom results.
  qbat::BatchDiagnostics qbat;
  bool has_qeph=true,has_t3=true,has_qbat=false;
};
struct ShellFormulationParticipants {
  qeph::QephBatch* qeph=nullptr;
  t3::T3Batch* t3=nullptr;
  qbat::Batch* qbat=nullptr;
  type25::Batch* connector=nullptr;
};
struct ShellFormulationCandidates {
  const qeph::BatchDiagnostics* qeph=nullptr;
  const t3::BatchDiagnostics* t3=nullptr;
  const qbat::BatchDiagnostics* qbat=nullptr;
  const type25::BatchDiagnostics* connector=nullptr;
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

// Closed shell publication scope: matching PrescribedFields or CoupledForces
// usage, with an optional TYPE25 participant for joined CoupledForces only.
// Coupled candidates require every accepted cache in this scope to have
// contributed to this same owner's attempt. Both
// batches must be initialized with the SAME complete immutable binding and
// its exact nonzero family counts within their explicit resident limits,
// owner/configuration/qualification/usage, then perform initial rest/mass
// binding. The caller may evaluate QEPH and T3 in either order from the same
// authentic prepared token. Neither joined batch can publish by itself.
// Startup is matching reference-rest or explicit uniform translation. This API does not select a
// stable timestep or qualify a general mixed-shell/contact trajectory.
//
// This object borrows the batches; all attached batches must outlive the
// coordinator (including destruction), and the owner must outlive its calls. It owns one bounded reusable kinetic scratch allocation and diagnostic
// caches, not nodal state, an independent clock, material histories or a solver.
// Calls are serialized on the sole owner's stream. No per-step allocation.
// Every Prepare/Commit failure discards that nodal trial
// and all attached material trials; accepted results and caller outputs survive
// numerical failure. A CUDA failure poisons the participants; readable-device
// recovery is not promised. Raw fabricated device writes are not authenticated.
class ShellBatchPublication {
 public:
  ShellBatchPublication();
  ~ShellBatchPublication();
  ShellBatchPublication(const ShellBatchPublication&)=delete;
  ShellBatchPublication& operator=(const ShellBatchPublication&)=delete;
  // Requires both initial zero-stress caches already bound from this owner's actual
  // source buffers; claims one coordinator per participant. Duplicate or
  // foreign-source attachment fails before device allocation/publication.
  // Moving startup measures common K0 once from fresh CopyAccepted fields in
  // native node order. Family kinetic stays unavailable/zero, and epoch-zero
  // base_kinetic stays zero because there is no completed interval.
  ShellPublicationReport Initialize(FENodalState&,qeph::QephBatch&,t3::T3Batch&,
      const ShellPublicationLimits& limits={});
  // Combined M/J requires this complete third participant in the transaction.
  // Its initial cache is already authenticated from the same owner's sources.
  ShellPublicationReport Initialize(FENodalState&,qeph::QephBatch&,t3::T3Batch&,
      type25::Batch&,const ShellPublicationLimits& limits={});
  // Closed explicit QEPH/T3/QBAT composition. QBAT is required; QEPH/T3
  // pointers are present exactly when the common immutable inventory contains
  // that family. Every present shell uses InitializeFormulations. The optional
  // connector is required exactly when the common combined M/J is retained.
  ShellPublicationReport InitializeFormulations(FENodalState&,const ShellFormulationParticipants&,
      const ShellPublicationLimits& limits={});
  // Physical CIN profile. The actual rigid binding is required even when the
  // shell catalog contains no rigid skin. All source proofs finish before any
  // participant is claimed. There is no second clock or kinetic allocation.
  static ShellPublicationReport ForecastPhysical(const ShellPhysicalBinding&,
      std::size_t cin_attachments,const ShellPublicationLimits&,
      ShellPhysicalPublicationForecast&) noexcept;
  ShellPublicationReport InitializePhysical(FENodalState&,const ShellPhysicalBinding&,
      const NodalRigidAssemblyBinding&,const NodalCinWitnessSource&,
      const ShellPhysicalParticipants&,const ShellPhysicalPublicationIdentity&,
      const ShellPublicationLimits& limits={});
  ShellPublicationReport PreparePhysical(FENodalState&,const NodalTrialToken&,
      const ShellPhysicalCandidates&,ShellPhysicalDiagnostics*);
  ShellPublicationReport CommitPhysical(FENodalState&,const NodalTrialToken&,
      const ShellPhysicalDiagnostics&,const NodalValidationReceipt&) noexcept;
  ShellPublicationReport CopyAcceptedPhysicalDiagnostics(const NodalStamp&,
      ShellPhysicalDiagnostics*) const noexcept;
  ShellPublicationReport PrepareFormulations(FENodalState&,const NodalTrialToken&,
      const ShellFormulationCandidates&,ShellBatchDiagnostics*);
  // Preflight all completed contributors against the actual owner token
  // before GPU measurement. Kinetic energy is reduced over the complete native
  // union exactly once at each base/endpoint, at the declared velocity times.
  // Typed family kinetic_available=false and all their kinetic fields are zero.
  ShellPublicationReport Prepare(FENodalState&,const NodalTrialToken&,
      const qeph::BatchDiagnostics&,const t3::BatchDiagnostics&,ShellBatchDiagnostics*);
  ShellPublicationReport Prepare(FENodalState&,const NodalTrialToken&,
      const qeph::BatchDiagnostics&,const t3::BatchDiagnostics&,
      const type25::BatchDiagnostics&,ShellBatchDiagnostics*);
  // Repeat preflight of every result, measured diagnostics and receipt; perform
  // one owner commit followed only by infallible slab publications. No CUDA
  // call, readback, allocation or other fallible operation follows owner success.
  ShellPublicationReport Commit(FENodalState&,const NodalTrialToken&,
      const ShellBatchDiagnostics&,const NodalValidationReceipt&) noexcept;
  ShellPublicationReport CopyAcceptedDiagnostics(const NodalStamp&,ShellBatchDiagnostics*) const noexcept;
  // Read-only authentication for consumers of accepted family fields. Checks
  // the actual attached objects, complete native inventory and live owner
  // endpoint. Matching declared IDs alone never authenticate another batch.
  ShellPublicationReport ValidateAcceptedActivitySources(const FENodalState&,
      const ShellFormulationParticipants&,const ShellBatchInventory&) const noexcept;
  // Read-only source/output authority for additional precommit validators.
  // No participant attach, claim, preparation or publication occurs here.
  ShellPublicationReport ValidatePhysicalSources(const FENodalState&,
      const ShellPhysicalBinding&,const ShellPhysicalParticipants&,
      const ShellPhysicalPublicationIdentity&) const noexcept;
  bool PhysicalOutputDisjoint(const void*,std::size_t) const noexcept;
  // Discards coordinator and every material scratch; caller still owns nodal
  // Discard when abandoning a trial outside Prepare/Commit.
  void DiscardTrial() noexcept;
  NodalAllocationInfo allocations() const noexcept;
 private:
  ShellPublicationReport CopyAcceptedFormulations(const NodalStamp&,ShellBatchDiagnostics*) const noexcept;
  ShellPublicationReport InitializeImpl(FENodalState&,qeph::QephBatch&,t3::T3Batch&,
      type25::Batch*,const ShellPublicationLimits&);
  ShellPublicationReport PrepareImpl(FENodalState&,const NodalTrialToken&,
      const qeph::BatchDiagnostics&,const t3::BatchDiagnostics&,
      const type25::BatchDiagnostics*,ShellBatchDiagnostics*);
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace tl::fea
