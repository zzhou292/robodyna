#pragma once
namespace tl::fea::cin_recovery_test::frozen_tail {
using namespace cin_advance;
namespace cin = constraints::tied_shell::cin;
__device__ inline void Fail(nodal_detail::Control* control, NodalStatus status, std::uint32_t node) {
  control->status = status;
  control->node = node;
  if (status == NodalStatus::StepTooLarge) control->limit.dt = 0;
}
__global__ void CompleteCin(const cin_advance::Input input, bool groups_prepared = false) {
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
  constraints::tied_shell::cin::StageReport stage;
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
  stage = constraints::tied_shell::cin_recovery_frozen::RecoverMotionTrial(model, {patches, trial+3*n, trial+6*n,
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

}
} // namespace frozen_tail
