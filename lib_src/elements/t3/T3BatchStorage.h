#pragma once
#include "T3Batch.h"
#include "../../solvers/NodalTrialIdentity.h"
#include "../ShellBatchBinding.h"
#include "../ShellBatchPlasticityStorage.h"
#include <array>
#include <optional>
#include <utility>
#include <type_traits>

namespace tl::fea::t3::batch_detail {
struct Model {
  T3BatchConfig config;
  T3BatchElement element[MaxBatchElements];
  Vec3 initial_position[MaxBatchNodes]{};
  double mass[MaxBatchNodes]{},inertia[MaxBatchNodes]{};
  double physical[MaxBatchNodes]{},added[MaxBatchNodes]{};
  bool joined=false;
};
struct Slab { ForceTrial element[MaxBatchElements]; };
struct Control {
  BatchStatus status=BatchStatus::Success;
  Status element_status=Status::kSuccess;
  std::uint32_t element=UINT32_MAX,node=UINT32_MAX;
  BatchDiagnostics diagnostics;
};
struct Storage { Model model; Slab slab[2]; Control control; };
static_assert(std::is_trivially_copyable<Storage>::value,"Resident records require value-copy storage");
static_assert(sizeof(Storage)<=MaxBatchDeviceBytes,"Bounded T3 batch allocation");
// Collection capacity changes the private storage extent, not native results.
// Allocation tests retain actual host/CUDA sizes before promotion; no layout is
// a persisted ABI. Native result size and alignment remain independently fixed.
static_assert(sizeof(ForceTrial)==976&&alignof(Storage)==8,"Qualified native T3 record layout");

BatchReport BuildModel(const T3BatchConfig&,const T3BatchElement*,Model&,Slab&,const ShellBatchBinding* joined=nullptr);
bool SameDiagnostics(const BatchDiagnostics&,const BatchDiagnostics&) noexcept;
using trial_identity::SameStamp;
using trial_identity::SamePrepared;
using trial_identity::ValidKinematics;
BatchDiagnostics InitialDiagnostics(const T3BatchConfig&,bool joined=false);
void LaunchAssembly(Storage*,const Slab*,NodalAssemblyView,bool initial);
void LaunchCandidate(Storage*,const Slab*,Slab*,NodalPreparedView,BatchDiagnostics,
                     shell_batch_plasticity_detail::DeviceStorage*,unsigned accepted_slab);
void LaunchFailure(NodalAssemblyView);
} // namespace tl::fea::t3::batch_detail

namespace tl::fea::t3 {
struct T3Batch::Impl {
  T3BatchConfig config;
  NodalStamp accepted_stamp;
  std::optional<ShellBatchBinding> joined_binding; // Host-only immutable inventory.
  const ShellBatchPublication* publication_scope=nullptr; // One borrowed coordinator claim.
  std::unique_ptr<shell_batch_plasticity_detail::HostStorage> plasticity;
  batch_detail::Storage* storage=nullptr;
  batch_detail::Slab* accepted=nullptr;
  batch_detail::Slab* trial=nullptr;
  batch_detail::Control control;
  std::array<ForceTrial,MaxBatchElements> staging;
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
} // namespace tl::fea::t3
