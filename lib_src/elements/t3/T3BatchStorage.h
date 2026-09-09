#pragma once
#include "T3Batch.h"
#include "../../solvers/NodalTrialIdentity.h"
#include <array>
#include <type_traits>

namespace tl::fea::t3::batch_detail {
struct Model {
  T3BatchConfig config;
  T3BatchElement element[MaxBatchElements];
  Vec3 initial_position[MaxBatchNodes]{};
  double mass[MaxBatchNodes]{},inertia[MaxBatchNodes]{};
  double physical[MaxBatchNodes]{},added[MaxBatchNodes]{};
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
// Root's host declaration probe t3-batch-abi-probe-1: CUDA compilation must
// independently agree before claiming the corresponding device allocation.
static_assert(sizeof(Model)==2064&&sizeof(Slab)==1952&&sizeof(Control)==248&&
    sizeof(Storage)==6216&&sizeof(ForceTrial)==976&&sizeof(BatchDiagnostics)==232&&alignof(Storage)==8,
    "Reviewed 64-bit T3 batch ABI; diagnose layout drift before allocation");

BatchReport BuildModel(const T3BatchConfig&,const T3BatchElement*,Model&,Slab&);
bool SameDiagnostics(const BatchDiagnostics&,const BatchDiagnostics&) noexcept;
using trial_identity::SameStamp;
using trial_identity::SamePrepared;
using trial_identity::ValidKinematics;
BatchDiagnostics InitialDiagnostics(const T3BatchConfig&);
void LaunchAssembly(Storage*,const Slab*,NodalAssemblyView,bool initial);
void LaunchCandidate(Storage*,const Slab*,Slab*,NodalPreparedView,BatchDiagnostics);
void LaunchFailure(NodalAssemblyView);
} // namespace tl::fea::t3::batch_detail

namespace tl::fea::t3 {
struct T3Batch::Impl {
  T3BatchConfig config;
  NodalStamp accepted_stamp;
  batch_detail::Storage* storage=nullptr;
  batch_detail::Slab* accepted=nullptr;
  batch_detail::Slab* trial=nullptr;
  batch_detail::Control control;
  std::array<ForceTrial,MaxBatchElements> staging;
  BatchDiagnostics accepted_diagnostics,candidate_diagnostics;
  NodalPreparedView candidate_view;
  cudaStream_t stream=nullptr;
  std::uint64_t assembled_epoch=UINT64_MAX,assembled_attempt=0,last_candidate_attempt=0;
  bool usable=true,bound=false,pending=false;
  ~Impl();
  BatchReport Runtime(cudaError_t,const char*) noexcept;
  BatchReport PendingError() noexcept;
  BatchReport ReadControl();
  BatchReport ReadResults(const batch_detail::Slab*);
  void Discard() noexcept { pending=false; candidate_view={}; candidate_diagnostics={}; }
};
} // namespace tl::fea::t3
