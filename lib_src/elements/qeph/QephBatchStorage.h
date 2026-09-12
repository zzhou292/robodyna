#pragma once
#include "QephBatch.h"
#include "mapped/AssemblyLayout.h"
#include "../../solvers/NodalTrialIdentity.h"
#include "../ShellBatchBinding.h"
#include "../../assembly/ShellPhysicalBinding.h"
#include "../../assembly/NodalMassBinding.h"
#include "../ShellBatchPlasticityStorage.h"
#include "../ShellBatchArenaLayout.h"
#include "lib_utils/BoundedStartupArray.h"
#include <optional>
#include <utility>
#include <type_traits>

namespace tl::fea::qeph::batch_detail {
struct Model {
  QephBatchConfig config;
  QephBatchElement* element=nullptr;
  Vec3* initial_position=nullptr;
  double *mass=nullptr,*inertia=nullptr;
  double *physical=nullptr,*added=nullptr;
  bool joined=false;
  bool mapped=false; // Explicit complete physical-domain profile only.
};
struct Slab { ForceTrial* element=nullptr; };
struct Control {
  BatchStatus status=BatchStatus::Success;
  Status element_status=Status::kSuccess;
  std::uint32_t element=UINT32_MAX,node=UINT32_MAX;
  BatchDiagnostics diagnostics;
};
struct Storage { Model model; Slab slab[2]; Control control; Status* candidate_status=nullptr; mapped::AssemblyMemory assembly; };
static_assert(std::is_trivially_copyable<Storage>::value,"Resident records require value-copy storage");
using BaseLayout=shell_batch_detail::BatchArenaLayout<Storage,QephBatchElement,ForceTrial,Vec3,Status>;
struct Layout : BaseLayout {
  mapped::AssemblyLayout assembly;
  bool Initialize(std::size_t parents,std::size_t nodes,std::size_t cap) noexcept {
    Layout next;
    if (!next.BaseLayout::Initialize(parents,nodes,cap)) return false;
    *this=next; return true;
  }
  bool InitializeMapped(std::size_t parents,std::size_t nodes,std::size_t cap) noexcept {
    Layout next;
    if (!next.Initialize(parents,nodes,cap) ||
        !next.assembly.Initialize(next.bytes,parents,nodes,cap)) return false;
    next.bytes=next.assembly.bytes; *this=next; return true;
  }
  Storage* Construct(util::HostArena& arena) const noexcept {
    auto* result=BaseLayout::Construct(arena);
    return result&&assembly.Construct(arena,result->assembly)?result:nullptr;
  }
  Storage Rebase(const Storage& host,void* base) const noexcept {
    auto result=BaseLayout::Rebase(host,base);
    result.assembly=assembly.Rebase(base); return result;
  }
};
static_assert(sizeof(Storage)<2048,"Resident header contains no capacity-sized arrays");

BatchReport BuildModel(const QephBatchConfig&,const QephBatchElement*,Model&,Slab&,const ShellBatchBinding* joined=nullptr,
    const ShellBatchFailureBinding* failure=nullptr,bool formulations=false);
bool SameDiagnostics(const BatchDiagnostics&,const BatchDiagnostics&) noexcept;
using trial_identity::SameStamp;
using trial_identity::SamePrepared;
using trial_identity::ValidKinematics;
BatchDiagnostics InitialDiagnostics(const QephBatchConfig&,bool joined=false);
void LaunchMappedAssembly(Storage*,const Slab*,NodalAssemblyView,NodalCinAssemblyView,
    const shell_batch_plasticity_detail::MixedDeviceStorage*,bool initial);
void LaunchMappedActivity(Storage*,std::size_t parents,unsigned slab,double time,std::uint64_t epoch,
    const shell_batch_plasticity_detail::MixedDeviceStorage*,cudaStream_t);
void LaunchAssembly(Storage*,const Slab*,NodalAssemblyView,bool initial);
void LaunchMappedCandidateDiagnostics(Storage*,const Slab*,const Slab*,NodalPreparedView,BatchDiagnostics,
    const shell_batch_plasticity_detail::MixedDeviceStorage*);
void LaunchMappedObserverReduction(Storage*,const Slab*,const Slab*,NodalPreparedView,BatchDiagnostics,
    const shell_batch_plasticity_detail::MixedDeviceStorage*);
void LaunchMappedObserverDiagnostics(Storage*,const Slab*,const Slab*,NodalPreparedView,BatchDiagnostics,
    const shell_batch_plasticity_detail::MixedDeviceStorage*);
void LaunchCandidate(Storage*,const Slab*,Slab*,NodalPreparedView,BatchDiagnostics,
                     shell_batch_plasticity_detail::DeviceStorage*,unsigned accepted_slab,std::size_t element_count,
                     shell_batch_plasticity_detail::MixedDeviceStorage*,
                     shell_batch_plasticity_detail::FailureDeviceStorage*,bool mapped);
void LaunchFailure(NodalAssemblyView);
} // namespace tl::fea::qeph::batch_detail

namespace tl::fea::qeph {
struct QephBatch::Impl {
  QephBatchConfig config;
  NodalStamp accepted_stamp;
  std::optional<ShellPhysicalBinding> physical;
  std::size_t cin_witness_count=0;
  bool formulations=false; // Only the explicit complete-formulation initializer.
  std::optional<ShellBatchBinding> joined_binding; // Host-only immutable inventory.
  std::optional<NodalMassBinding> joined_mass; // Complete augmented startup identity.
  const ShellBatchPublication* publication_scope=nullptr; // One borrowed coordinator claim.
  std::unique_ptr<shell_batch_plasticity_detail::HostStorage> plasticity;
  batch_detail::Storage* storage=nullptr;
  batch_detail::Storage device_header; // Host value containing only rebased device addresses.
  batch_detail::Layout layout;
  batch_detail::Slab* accepted=nullptr;
  batch_detail::Slab* trial=nullptr;
  batch_detail::Control control;
  util::BoundedStartupArray<ForceTrial,0> staging;
  util::BoundedStartupArray<std::uint8_t,0> activity_staging;
  BatchDiagnostics accepted_diagnostics,candidate_diagnostics;
  NodalPreparedView candidate_view;
  // Saved only on first numerical rest/mass binding. Pointer identity is
  // checked against the actual owner before its first history publication;
  // this record never extends device-view lifetime or permits dereferencing.
  NodalAssemblyView initial_sources;
  cudaStream_t stream=nullptr;
  std::uint64_t assembled_epoch=UINT64_MAX,assembled_attempt=0,last_candidate_attempt=0;
  bool usable=true,bound=false,pending=false;
  ~Impl();
  BatchReport Runtime(cudaError_t,const char*) noexcept;
  BatchReport PendingError() noexcept;
  BatchReport ReadControl();
  BatchReport ReadResults(const batch_detail::Slab*);
  BatchReport InitializePlasticity(const ShellBatchPlasticityConfig&,const batch_detail::Model&);
  BatchReport InitializePlasticity(const ShellBatchPlasticityBinding&);
  BatchReport InitializeFailure(const ShellBatchFailureBinding&,const ShellBatchFailureLimits&);
  BatchReport ValidateMappedResults(unsigned slab) const noexcept;
  BatchReport ValidateMappedSections(unsigned slab);
  BatchReport ValidateMappedActivity(unsigned slab);
  bool OutputDisjoint(const void*,std::size_t) const noexcept;
  unsigned AcceptedSlabIndex() const noexcept { return accepted==&storage->slab[0]?0u:1u; }
  void Discard() noexcept { pending=false; candidate_view={}; candidate_diagnostics={}; }
  // Infallible sole publication boundary, shared by standalone and joined paths.
  void Publish(const NodalStamp& stamp) noexcept {
    std::swap(accepted,trial); accepted_stamp=stamp; accepted_diagnostics=candidate_diagnostics;
    accepted_diagnostics.phase=BatchPhase::Accepted; Discard();
  }
};
} // namespace tl::fea::qeph
