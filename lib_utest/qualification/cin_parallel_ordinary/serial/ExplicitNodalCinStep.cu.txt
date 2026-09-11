// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCinRuntime.h"
#include "cin_timestep/Screen.h"
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
__global__ void AdvanceCin(nodal_detail::Control* control, const double* accepted, double* trial,
    double* loads, const std::uint8_t* fixed, cin::StageView model, double* tail,
    double* work, constraints::tied_shell::Patch* patches, const std::uint8_t* activity,
    rigid::GroupDeviceView groups, rigid::StepDurations durations, double maximum_angle,
    std::uint64_t epoch, std::uint64_t attempt, rigid::AccelerationSink capture,
    const std::uint8_t* rotation_present, NodalCinStructuralStep structural) {
  if (control->status != NodalStatus::Ok) return;
  if (control->rows.base_epoch != epoch || control->rows.attempt != attempt ||
      !stability::IsCurrentLimit(control->rows, control->limit) ||
      control->limit.dt < durations.drift_dt) {
    Fail(control, NodalStatus::StaleTrial, UINT32_MAX);
    return;
  }
  if (control->limit.has_stiffness_or_damping) {
    Fail(control, NodalStatus::MissingStepAdmission, UINT32_MAX);
    return;
  }
  const auto n = model.node_count;
  const auto r = model.row_count;
  const cin::ForceTrial force{accepted, loads, tail, tail+n, work, work+n,
    tail+4*n, tail+4*n+r, tail+4*n+2*r, work+2*n, patches, activity};
  auto stage = cin::PrepareForceTrial(model, force);
  if (!stage) {
    Fail(control, stage.status == cin::StageStatus::PendingReleaseEligibility
        ? NodalStatus::MissingStepAdmission : NodalStatus::InvalidOutput, stage.node);
    return;
  }
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
    if (durations.drift_dt > result.minimum_dt) {
      control->status = NodalStatus::StepTooLarge;
      control->node = result.limiting_node;
      return;
    }
  }
  auto* current_inverse = tail+2*n;
  auto* acceleration = work+3*n;
  auto* angular_acceleration = work+6*n;
  for (std::uint32_t i = 0; i < n; ++i) {
    // There is no conventional inverse/kick for a dependent CIN DOF.
    if (model.dependent_nodes[i]) {
      current_inverse[i] = 0;
      current_inverse[n+i] = 0;
      continue;
    }
    const bool rigid_dependent = groups.member_nodes && rigid::UsesDependentCoefficients(groups.member_nodes[i]);
    current_inverse[i] = fixed[n+i] == 7 || (rigid_dependent && tail[i] == 0) ? 0 : 1/tail[i];
    current_inverse[n+i] = fixed[2*n+i] || (rotation_present && !rotation_present[i]) ||
        (rigid_dependent && tail[n+i] == 0) ? 0 : 1/tail[n+i];
    if (!std::isfinite(current_inverse[i]) || !std::isfinite(current_inverse[n+i])) {
      Fail(control, NodalStatus::InvalidOutput, i);
      return;
    }
    if (groups.member_nodes && groups.member_nodes[i]) continue;
    const auto status = nodal_detail::AdvanceOrdinaryNode<true>(accepted, trial, loads,
      current_inverse, fixed, i, n, durations.drift_dt, durations.kick_dt, maximum_angle,
      acceleration, angular_acceleration, rotation_present);
    if (status != NodalStatus::Ok) {
      Fail(control, status, i);
      return;
    }
  }
  // Source/owner admission proves CIN has no rigid member intersection. The
  // existing native aggregate and member arithmetic therefore stays unchanged.
  for (std::uint32_t g = 0; g < groups.group_count; ++g) {
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
  stage = cin::RecoverMotionTrial(model, {patches, trial+3*n, trial+6*n,
      acceleration, angular_acceleration});
  if (!stage) {
    Fail(control, NodalStatus::InvalidOutput, stage.node);
    return;
  }
  for (std::uint32_t row = 0; row < r; ++row) {
    const auto node = model.rows[row].secondary;
    for (unsigned a = 0; a < 3; ++a) {
      const auto j = 3*node+a;
      trial[j] = accepted[j]+durations.drift_dt*trial[3*n+j];
      // A dependent is not a prescribed fixed DOF. Its transfer is represented
      // by the retained native loads, not a fabricated fixed reaction.
      trial[13*n+j] = 0;
      trial[16*n+j] = 0;
      if (!std::isfinite(trial[j])) {
        Fail(control, NodalStatus::InvalidOutput, node);
        return;
      }
    }
    const auto status = nodal_detail::PrepareNodeOrientation(accepted, trial, node, n,
        durations.drift_dt, maximum_angle, false);
    if (status != NodalStatus::Ok) {
      Fail(control, status, node);
      return;
    }
  }
  if (capture.node) {
    for (std::uint32_t node = 0; node < n; ++node) {
      if (groups.member_nodes && groups.member_nodes[node]) continue;
      for (unsigned a = 0; a < 3; ++a) {
        capture.node[3*node+a] = acceleration[3*node+a];
        capture.node_rotation[3*node+a] = angular_acceleration[3*node+a];
      }
    }
  }
}
} // namespace

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
  AdvanceCin<<<1,1,0,stream>>>(control, accepted, trial, scratch, fixed, cin->device,
      trial+cin->state_offset, cin->work, cin->patches, cin->activity, groups, durations,
      maximum_angle, stamp.epoch, attempt, capture,
      stamp.has_rotation_presence?fixed+3*config.node_count:nullptr,
      structural ? *structural : NodalCinStructuralStep{});
  return cudaGetLastError();
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
