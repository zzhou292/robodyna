#pragma once
#include "QephForceData.h"
#include "../ShellCollectionLimits.h"
#include "../ShellResidentLimits.h"
#include "../ShellBatchStartup.h"
#include "../../solvers/ExplicitNodalStep.h"
#include <memory>

namespace tl::fea { class ShellBatchBinding; class ShellBatchPublication; class NodalMassBinding;
  class ShellBatchPlasticityBinding; struct ShellBatchPlasticityConfig; struct ShellBatchSectionState; class ShellBatchLayeredSection; }
namespace tl::fea::qeph {
constexpr std::size_t MaxBatchElements=MaxShellCollectionParents,MaxBatchNodes=MaxShellCollectionNodes;
constexpr std::size_t MaxBatchDeviceBytes=1024*1024;
struct QephBatchElement { ReferenceData reference; std::size_t nodes[4]{}; };
enum class BatchUsage { Unspecified,PrescribedFields,CoupledForces };
using BatchStartupKind=ShellBatchStartupKind;
using BatchStartup=ShellBatchStartup;
struct QephBatchConfig {
  NodalStamp owner;
  std::uint64_t configuration_id=0,qualification_id=0;
  std::size_t element_count=0,max_device_bytes=MaxBatchDeviceBytes;
  BatchUsage usage=BatchUsage::Unspecified;
  BatchStartup startup;
  ShellResidentLimits storage_limits;
};
enum class BatchStatus {
  Success,InvalidInput,NotInitialized,NotBound,ResourceLimit,WrongOwner,StaleTrial,
  InvalidMass,ElementFailure,AssemblyFailure,NonfiniteResult,DeviceFailure,NodalFailure
};
struct BatchReport {
  BatchStatus status=BatchStatus::InvalidInput;
  const char* message="Invalid batch request";
  std::uint32_t element=UINT32_MAX,node=UINT32_MAX;
  Status element_status=Status::kSuccess;
  NodalStatus nodal_status=NodalStatus::Ok;
};
enum class BatchPhase { Unspecified,Accepted,Prepared };
// Endpoint identity and the COMPLETED interval that produced it. At epoch zero
// interval fields are absent (has_completed_interval=false); no dt=0 force ran.
struct BatchDiagnostics {
  std::uint64_t owner_id=0,configuration_id=0,qualification_id=0;
  std::uint64_t epoch=0,base_epoch=0,attempt=0;
  double time=0,base_time=0,velocity_time=0,base_velocity_time=0,kick_dt=0;
  BatchPhase phase=BatchPhase::Unspecified;
  bool valid=false,has_completed_interval=false,accepted_force_assembled=false;
  // Joined participants expose no whole-owner or partial-family kinetic sum.
  // When false, all four kinetic fields are zero; use ShellBatchPublication.
  bool kinetic_available=true;
  BatchUsage usage=BatchUsage::Unspecified;
  double kinetic_translation=0,kinetic_rotation=0;
  // Scalar isotropic partitions include drilling; do not label as pure physical
  // tangential energy. Their sum need not round to the native total-J energy.
  double kinetic_physical_isotropic=0,kinetic_added_isotropic=0;
  double internal_work[2]{},internal_work_increment[2]{};
  double hourglass_viscous_work=0,hourglass_viscous_work_increment=0;
  double minimum_area_ratio=1,minimum_thickness_ratio=1,maximum_displacement=0;
  double maximum_absolute_strain=0,maximum_thickness_curvature=0;
  double minimum_native_dt=0;
  // THIS participant's signed RHS, independent of external loads. Kick duration
  // differs from drift h initially. Neither ledger is a conservative potential.
  double internal_kick_work=0,internal_drift_work=0;
};

class QephBatch;
BatchReport CommitQephTrial(FENodalState&,const NodalTrialToken&,QephBatch&,
                           const BatchDiagnostics&,const NodalValidationReceipt&) noexcept;

// One resident immutable model and two element history/cache slabs; no nodal
// state, clock, mechanics equations or timestep policy. Calls are serialized and
// use the owner's stream. LAW1 has one device allocation; the opt-in plastic
// section has one additional bounded allocation. No per-step allocation.
// Initialize admits only epoch-zero staggered owner metadata. First assembly
// verifies actual reference-at-rest x/v/omega, free m/J and unit q before binding.
// Explicit ReferenceUniformTranslation is CoupledForces only:
// reference x, bit-identical declared common v, omega=0 and identity q. It uses
// known zero initial stress/history/cache, never a dt=0 force operation. This
// startup data contract supplies no recurrence or arbitrary-pose qualification.
// Actual initial kinetic energy is published only after live-owner source
// authentication and successful first assembly/readback. Joined participants
// retain unavailable/zero family kinetic; the coordinator owns common K0.
// Failed contributions are sticky when valid failure channels exist; otherwise
// caller must discard after ANY failure. Initial readback requires this binding.
// EvaluateCandidate always starts from accepted history. It can follow a
// prescribed owner trajectory without consuming cached internal loads; the
// diagnostic records accepted_force_assembled=false. That path is a value/
// transaction test, not a coupled dynamics admission. A future coupled case
// MUST require true and supply its own whole-recurrence numerical qualification.
// Immutable CoupledForces usage requires a matching accepted assembly attempt;
// PrescribedFields deliberately forbids consuming this batch in the attempt
// that is published. CommitQephTrial establishes only joint publication.
class QephBatch {
 public:
  QephBatch(); ~QephBatch();
  QephBatch(const QephBatch&)=delete;
  QephBatch& operator=(const QephBatch&)=delete;
  BatchReport Initialize(const QephBatchConfig&,const QephBatchElement*);
  // Immutable joined scope; every QEPH cell from the complete collection.
  // Both families must be present; config.element_count must match exactly.
  // Matching prescribed or coupled usage is enforced by the sole joined
  // coordinator. Startup may be rest or common translation; standalone Commit rejects it.
  BatchReport InitializeJoined(const QephBatchConfig&,const ShellBatchBinding&);
  // Opt-in layered plasticity with explicit optional source rate settings.
  // Deep-copies the declaration;
  // caller config/curve storage may expire after Initialize returns.
  BatchReport Initialize(const QephBatchConfig&,const QephBatchElement*,const ShellBatchPlasticityConfig&);
  BatchReport InitializeJoined(const QephBatchConfig&,const ShellBatchBinding&,const ShellBatchPlasticityConfig&);
  // Complete multi-material catalog: both families copy the same full binding,
  // while each native parent selects its own prepared material/section.
  BatchReport InitializeJoined(const QephBatchConfig&,const ShellBatchBinding&,const ShellBatchPlasticityBinding&);
  // Augmented startup uses one complete typed nodal M/J composition.
  // Its connector must also join the sole publication coordinator.
  BatchReport InitializeJoined(const QephBatchConfig&,const ShellBatchBinding&,const NodalMassBinding&);
  BatchReport InitializeJoined(const QephBatchConfig&,const ShellBatchBinding&,
                               const ShellBatchPlasticityBinding&,const NodalMassBinding&);
  // Raw supplied-view validation/assembly for rest or an already-bound batch.
  // First uniform-translation binding requires the live-owner overload below.
  // Initial numerical rest binding retains
  // its source identity; actual owner association is checked separately before
  // the first standalone or joined publication. A forged raw view alone is
  // therefore not an owner/history publication authority.
  // Raw grouped views reject. Grouped rest and moving startup, and every
  // later grouped assembly, require the actual live-owner overload.
  BatchReport AssembleAccepted(const NodalAssemblyView&);
  // On first uniform binding, authenticate SOURCE pointer identity against the
  // live owner before measuring/publishing K0. The predicate does not consume
  // CUDA errors, dereference expired views or authorize force destinations.
  // No owner reference is retained. Other binding/assembly behavior is shared.
  BatchReport AssembleAccepted(FENodalState&,const NodalAssemblyView&);
  BatchReport EvaluateCandidate(const NodalPreparedView&,BatchDiagnostics*);
  // Required for rigid-group coupling: authenticate the common owner token.
  BatchReport EvaluateCandidate(FENodalState&,const NodalTrialToken&,const NodalPreparedView&,BatchDiagnostics*);
  // Output-cadence staged readback, never an evaluation/history advance. The
  // accepted slab remains readable after numerical rejection/discard. All output
  // ranges must be host writable, disjoint and not overlap this batch's storage.
  BatchReport CopyAcceptedResults(const NodalStamp&,ForceTrial*,std::size_t capacity,
                                  BatchDiagnostics*);
  BatchReport CopyPreparedResults(const BatchDiagnostics&,ForceTrial*,std::size_t capacity);
  // Explicit catalog mode: law-tagged history; elastic values have no PLA/rate fields.
  BatchReport CopyAcceptedLayeredSectionHistory(const NodalStamp&,ShellBatchLayeredSection*,
      std::size_t capacity,BatchDiagnostics*);
  BatchReport CopyPreparedLayeredSectionHistory(const BatchDiagnostics&,ShellBatchLayeredSection*,std::size_t capacity);
  BatchReport CopyAcceptedSectionHistory(const NodalStamp&,ShellBatchSectionState*,std::size_t capacity,
                                        BatchDiagnostics*);
  BatchReport CopyPreparedSectionHistory(const BatchDiagnostics&,ShellBatchSectionState*,std::size_t capacity);
  void DiscardTrial() noexcept;
  NodalAllocationInfo allocations() const noexcept;
 private:
  friend class ::tl::fea::ShellBatchPublication;
  BatchReport InitializeImpl(const QephBatchConfig&,const QephBatchElement*,const ShellBatchBinding*,
                             const ShellBatchPlasticityConfig* plasticity=nullptr,
                             const ShellBatchPlasticityBinding* collection_plasticity=nullptr,
                             const NodalMassBinding* nodal_mass=nullptr);
  BatchReport AssembleAcceptedImpl(FENodalState*,const NodalAssemblyView&);
  BatchReport EvaluateCandidateImpl(FENodalState*,const NodalTrialToken*,const NodalPreparedView&,BatchDiagnostics*);
  friend BatchReport CommitQephTrial(FENodalState&,const NodalTrialToken&,QephBatch&,
                                    const BatchDiagnostics&,const NodalValidationReceipt&) noexcept;
  struct Impl; std::unique_ptr<Impl> impl_;
};
} // namespace tl::fea::qeph
