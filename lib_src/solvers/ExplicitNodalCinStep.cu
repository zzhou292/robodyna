// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCinRuntime.h"
#include "cin_advance/Node.h"
#include "cin_advance/ForceInputs.h"
#include "cin_advance/ForceTransfers.h"
#include "cin_advance/Recovery.h"
#include "cin_advance/RecoveryDrift.h"
#include "cin_advance/Screen.h"
#include "cin_advance/Capture.h"
#include "cin_advance/Groups.h"
#include "cin_timestep/Screen.h"
#include "cin_limiter/Capture.h"
#include "NodalCinStorage.h"
#include "NodalRigidGroupStorage.h"
#include "NodalNodeStep.h"
#include "NodalForceStageCaptureLayout.h"
#include "ExplicitNodalStep.h"
#include "../constraints/NodalRigidGroupCandidate.h"
#include "../constraints/tied_shell/runtime/CinMotionStage.h"

namespace tl::fea {
namespace {
namespace cin = constraints::tied_shell::cin;
__device__ void Fail(nodal_detail::Control* control, NodalStatus status, std::uint32_t node) {
  control->status = status;
  control->node = node;
  if (status == NodalStatus::StepTooLarge) control->limit.dt = 0;
}
__global__ void PrepareCin(const cin_advance::Input input, bool inputs_prepared, bool parallel_screen,
    bool transfers_prepared = false) {
  auto* control = input.control;
  const auto* accepted = input.accepted;
  auto* loads = input.loads;
  const auto* fixed = input.fixed;
  const auto model = input.model;
  auto* tail = input.tail;
  auto* work = input.work;
  auto* patches = input.patches;
  const auto* activity = input.activity;
  const auto groups = input.groups;
  const auto durations = input.durations;
  const auto epoch = input.epoch;
  const auto attempt = input.attempt;
  const auto* rotation_present = input.rotation_present;
  const auto structural = input.structural;
  if (control->status != NodalStatus::Ok) return;
  if (!inputs_prepared && !cin_advance::force_inputs::CheckPrefix(input)) return;
  const auto n = model.node_count;
  const auto r = model.row_count;
  const cin::ForceTrial force{accepted, loads, tail, tail+n, work, work+n,
    r ? tail+4*n : nullptr, r ? tail+4*n+r : nullptr, tail+4*n+2*r, work+2*n, patches, activity};
  auto stage = transfers_prepared ? cin_advance::force_transfers::Apply(input)
      : inputs_prepared ? cin::detail::TransferForceTrial(model, force)
      : cin::PrepareForceTrial(model, force);
  if (!stage) {
    Fail(control, stage.status == cin::StageStatus::PendingReleaseEligibility
        ? NodalStatus::MissingStepAdmission : NodalStatus::InvalidOutput, stage.node);
    return;
  }
  // Screen owns the ordinary failure-key initialization only after successful
  // structural admission. A rejected screen leaves the prior key untouched.
  if (parallel_screen) return;
  if (structural.profile != NodalCinStructuralProfile::Disabled) {
    const cin_timestep::Sources sources{accepted, tail, tail+n, work, work+n,
      fixed+n, fixed+2*n, rotation_present, model.dependent_nodes, groups, n,
      durations.previous_drift_dt};
    cin_timestep::Result result;
    std::uint32_t invalid_node = UINT32_MAX;
    if (!cin_timestep::Screen(sources, structural.factor, result, invalid_node)) {
      Fail(control, NodalStatus::InvalidOutput, invalid_node);
      return;
    }
    // Preserve the complete native/analytical bound for both success and a
    // rejected step. This existing owner control is not a second row proof.
    control->limit.dt = result.minimum_dt;
    if (structural.capture_limiter)
      control->structural_limiter = cin_limiter::Capture(sources, structural.factor,
          result, epoch, attempt);
    if (durations.drift_dt > result.minimum_dt) {
      control->status = NodalStatus::StepTooLarge;
      control->node = result.limiting_node;
      return;
    }
  }
  *input.failure = cin_advance::NoFailure;
}
__global__ void AdvanceOrdinaryCin(const cin_advance::Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  for (std::uint32_t node = blockIdx.x*blockDim.x+threadIdx.x;
       node < input.model.node_count; node += blockDim.x*gridDim.x) {
    const auto status = cin_advance::AdvanceNode(input, node);
    if (status != NodalStatus::Ok) {
      atomicMin(input.failure, cin_advance::EncodeFailure(node, status));
    }
  }
}
__global__ void CompleteCin(const cin_advance::Input input, bool groups_prepared = false,
    bool defer_recovery = false) {
  auto* control = input.control;
  const auto* accepted = input.accepted;
  auto* trial = input.trial;
  auto* loads = input.loads;
  const auto model = input.model;
  auto* work = input.work;
  auto* patches = input.patches;
  const auto groups = input.groups;
  const auto durations = input.durations;
  const auto maximum_angle = input.maximum_angle;
  const auto capture = input.capture;
  if (control->status != NodalStatus::Ok) return;
  const auto failure = *input.failure;
  if (failure != cin_advance::NoFailure) {
    Fail(control, cin_advance::FailureStatus(failure), cin_advance::FailureNode(failure));
    return;
  }
  const auto n = model.node_count;
  const auto r = model.row_count;
  auto* acceleration = work+3*n;
  auto* angular_acceleration = work+6*n;
  cin::StageReport stage;
  // Source/owner admission proves CIN has no rigid member intersection. The
  // existing native aggregate and member arithmetic therefore stays unchanged.
  for (std::uint32_t g = 0; !groups_prepared && g < groups.group_count; ++g) {
    const auto result = capture.node
      ? rigid::PrepareGroupCandidate<true>(groups, g, accepted, trial, loads, n, durations, capture)
      : rigid::PrepareGroupCandidate<false>(groups, g, accepted, trial, loads, n, durations);
    if (result.status != rigid::StepStatus::Success) {
      Fail(control, result.status == rigid::StepStatus::RotationLimit
          ? NodalStatus::StepTooLarge : NodalStatus::InvalidOutput, result.node);
      return;
    }
    const auto range = groups.groups[g];
    for (std::uint32_t i = 0; i < range.count; ++i) {
      const auto node = groups.members[range.offset+i].node;
      const auto status = nodal_detail::PrepareNodeOrientation(accepted, trial, node, n,
          durations.drift_dt, maximum_angle, false);
      if (status != NodalStatus::Ok) {
        Fail(control, status, node);
        return;
      }
    }
  }
  if (defer_recovery) return;
  stage = cin::RecoverMotionTrial(model, {patches, trial+3*n, trial+6*n,
      acceleration, angular_acceleration});
  if (!stage) {
    Fail(control, NodalStatus::InvalidOutput, stage.node);
    return;
  }
  cin_advance::recovery::Drift(input);

}
} // namespace

cudaError_t cin_advance::Launch(const Input& input, cudaStream_t stream) {
  const bool parallel_inputs = input.input_failure != nullptr;
  auto error = cudaSuccess;
  if (parallel_inputs) {
    error = force_inputs::Launch(input, stream);
    if (error != cudaSuccess) return error;
  }
  const bool parallel_screen = input.screen && screen::Blocks(input.model.node_count) &&
      input.structural.profile != NodalCinStructuralProfile::Disabled;
  const bool parallel_transfers = parallel_inputs && input.prepared_transfers;
  if (parallel_transfers) {
    error = force_transfers::Launch(input, stream);
    if (error != cudaSuccess) return error;
  }
  PrepareCin<<<1,1,0,stream>>>(input, parallel_inputs, parallel_screen, parallel_transfers);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  if (parallel_screen) {
    error = screen::Launch(input, stream);
    if (error != cudaSuccess) return error;
  }
  constexpr unsigned threads = 128;
  const auto blocks = (input.model.node_count+threads-1)/threads;
  AdvanceOrdinaryCin<<<blocks,threads,0,stream>>>(input);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  const bool parallel_groups = input.group_reports && input.groups.group_count;
  if (parallel_groups) {
    error = groups::LaunchMotion(input, stream);
    if (error != cudaSuccess) return error;
  }
  const bool parallel_recovery = input.prepared_recovery && input.recovery_failure && input.model.row_count;
  CompleteCin<<<1,1,0,stream>>>(input, parallel_groups, parallel_recovery);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  if (parallel_recovery) {
    error = recovery::Launch(input, stream);
    if (error != cudaSuccess) return error;
  }
  return capture::Launch(input, stream);
}

cudaError_t FENodalState::Impl::LaunchCinAdvance(double maximum_angle,
    const NodalCinStructuralStep* structural) {
  const rigid::StepDurations durations{stamp.epoch == 0 ? 0 : config.fixed_dt,
      candidate_kick_dt, config.fixed_dt};
  const auto groups = rigid_groups ? rigid_groups->device : rigid::GroupDeviceView{};
  rigid::AccelerationSink capture;
  if (config.capture_force_stage_accelerations) {
    const nodal_detail::ForceStageCaptureLayout layout{config.node_count, groups.group_count};
    capture = layout.Sink(scratch);
  }
  return cin_advance::Launch({control, accepted, trial, scratch, fixed, cin->device,
      trial+cin->state_offset, cin->work, cin->patches, cin->activity, groups, durations,
      maximum_angle, stamp.epoch, attempt, capture,
      stamp.has_rotation_presence?fixed+3*config.node_count:nullptr,
      structural ? *structural : NodalCinStructuralStep{}, cin->failure, cin->input_failure, cin->screen, cin->group_reports,
      cin->prepared_transfers, cin->prepared_recovery, cin->recovery_failure, cin->prepared_drift}, stream);
}

NodalReport AdvanceStaggeredCin(FENodalState& owner, const NodalTrialToken& token,
    const NodalCinAdmission& admission) {
  if (!owner.impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  auto& state = *owner.impl_;
  if (!state.cin || !admission.no_explicit_interface_release_event ||
      admission.qualification_id != state.cin->qualification_id) {
    return state.Reject(NodalStatus::MissingStepAdmission, "Missing CIN no-release qualification");
  }
  if (!ValidCinStructuralStep(admission.structural)) {
    return state.Reject(NodalStatus::MissingStepAdmission, "Invalid physical CIN structural profile");
  }
  const auto* structural = admission.structural.profile == NodalCinStructuralProfile::Disabled
      ? nullptr : &admission.structural;
  const NodalStepAdmission declared{admission.owner_id, admission.base_epoch, admission.attempt,
    admission.maximum_dt, admission.maximum_rotation_increment,
    NodalStepAdmissionKind::RestrictedHistoryTrajectory, admission.qualification_id, 0};
  return state.AdvanceSealedNodal(token.owner_id_, token.base_epoch_, token.attempt_, declared,
      NodalTemporalScheme::StaggeredHalfKickStart, bool(state.rigid_groups), true, structural);
}
} // namespace tl::fea
