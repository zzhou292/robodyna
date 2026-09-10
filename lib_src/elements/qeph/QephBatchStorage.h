#pragma once
#include "QephBatch.h"
#include "../../solvers/NodalTrialIdentity.h"
#include "../ShellBatchBinding.h"
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
};
struct Slab { ForceTrial* element=nullptr; };
struct Control {
  BatchStatus status=BatchStatus::Success;
  Status element_status=Status::kSuccess;
  std::uint32_t element=UINT32_MAX,node=UINT32_MAX;
  BatchDiagnostics diagnostics;
};
struct Storage { Model model; Slab slab[2]; Control control; Status* candidate_status=nullptr; };
static_assert(std::is_trivially_copyable<Storage>::value,"Resident records require value-copy storage");
using Layout=shell_batch_detail::BatchArenaLayout<Storage,QephBatchElement,ForceTrial,Vec3,Status>;
static_assert(sizeof(Storage)<2048,"Resident header contains no capacity-sized arrays");

BatchReport BuildModel(const QephBatchConfig&,const QephBatchElement*,Model&,Slab&,const ShellBatchBinding* joined=nullptr);
bool SameDiagnostics(const BatchDiagnostics&,const BatchDiagnostics&) noexcept;
using trial_identity::SameStamp;
using trial_identity::SamePrepared;
using trial_identity::ValidKinematics;
BatchDiagnostics InitialDiagnostics(const QephBatchConfig&,bool joined=false);
void LaunchAssembly(Storage*,const Slab*,NodalAssemblyView,bool initial);
void LaunchCandidate(Storage*,const Slab*,Slab*,NodalPreparedView,BatchDiagnostics,
                     shell_batch_plasticity_detail::DeviceStorage*,unsigned accepted_slab,std::size_t element_count);
void LaunchFailure(NodalAssemblyView);
} // namespace tl::fea::qeph::batch_detail

namespace tl::fea::qeph {
struct QephBatch::Impl {
  QephBatchConfig config;
  NodalStamp accepted_stamp;
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
  unsigned AcceptedSlabIndex() const noexcept { return accepted==&storage->slab[0]?0u:1u; }
  void Discard() noexcept { pending=false; candidate_view={}; candidate_diagnostics={}; }
  // Infallible sole publication boundary, shared by standalone and joined paths.
  void Publish(const NodalStamp& stamp) noexcept {
    std::swap(accepted,trial); accepted_stamp=stamp; accepted_diagnostics=candidate_diagnostics;
    accepted_diagnostics.phase=BatchPhase::Accepted; Discard();
  }
};
} // namespace tl::fea::qeph
