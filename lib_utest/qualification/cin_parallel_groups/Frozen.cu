// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FrozenSummary.h"

namespace tl::fea::cin_group_test::frozen_screen {
using cin_advance::NoFailure;
namespace {
__global__ void Begin(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  if (!frozen_values::CheckSources(Sources(input), input.structural.factor)) {
    input.control->status = NodalStatus::InvalidOutput;
    input.control->node = UINT32_MAX;
  }
}
__device__ void Reduce(Summary* values) {
  for (unsigned offset = Threads/2; offset; offset /= 2) {
    __syncthreads();
    if (threadIdx.x < offset) frozen_screen::Merge(values[threadIdx.x], values[threadIdx.x+offset]);
  }
  __syncthreads();
}
__global__ void Nodes(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  __shared__ Summary values[Threads];
  auto local = Empty();
  const auto source = Sources(input);
  for (std::uint32_t node = blockIdx.x*blockDim.x+threadIdx.x;
       node < source.nodes; node += gridDim.x*blockDim.x) {
    frozen_screen::Observe(source, input.structural.factor, node, local);
  }
  values[threadIdx.x] = local;
  Reduce(values);
  if (!threadIdx.x) input.screen[blockIdx.x] = values[0];
}
__global__ void Finish(Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  __shared__ Summary values[Threads];
  auto local = Empty();
  const auto blocks = Blocks(input.model.node_count);
  for (unsigned block = threadIdx.x; block < blocks; block += blockDim.x)
    frozen_screen::Merge(local, input.screen[block]);
  values[threadIdx.x] = local;
  Reduce(values);
  if (threadIdx.x) return;
  // Retained private first record makes the executed reduction observable in
  // owning CUDA packets; it is not a public clock or mechanics authority.
  input.screen[0] = values[0];
  cin_timestep::Result result;
  std::uint32_t invalid_node = UINT32_MAX;
  if (!frozen_screen::Complete(Sources(input), input.structural.factor, values[0], result, invalid_node)) {
    input.control->status = NodalStatus::InvalidOutput;
    input.control->node = invalid_node;
    return;
  }
  input.control->limit.dt = result.minimum_dt;
  if (input.durations.drift_dt > result.minimum_dt) {
    input.control->status = NodalStatus::StepTooLarge;
    input.control->node = result.limiting_node;
    return;
  }
  *input.failure = NoFailure;
}
} // namespace
cudaError_t Launch(const Input& input, cudaStream_t stream) {
  Begin<<<1, 1, 0, stream>>>(input);
  auto error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  Nodes<<<Blocks(input.model.node_count), Threads, 0, stream>>>(input);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  Finish<<<1, Threads, 0, stream>>>(input);
  return cudaGetLastError();
}
} // namespace tl::fea::cin_advance::screen

// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCinRuntime.h"
#include "cin_advance/Node.h"
#include "cin_advance/ForceInputs.h"
#include "cin_advance/Screen.h"
#include "cin_advance/Capture.h"
#include "cin_timestep/Screen.h"
#include "NodalCinStorage.h"
#include "NodalRigidGroupStorage.h"
#include "NodalNodeStep.h"
#include "NodalForceStageCaptureLayout.h"
#include "ExplicitNodalStep.h"
#include "../constraints/NodalRigidGroupCandidate.h"
#include "../constraints/tied_shell/runtime/CinMotionStage.h"

namespace tl::fea::cin_group_test {
using namespace cin_advance;
namespace {
namespace cin = constraints::tied_shell::cin;
__device__ void Fail(nodal_detail::Control* control, NodalStatus status, std::uint32_t node) {
  control->status = status;
  control->node = node;
  if (status == NodalStatus::StepTooLarge) control->limit.dt = 0;
}
__global__ void PrepareCin(const cin_advance::Input input, bool inputs_prepared, bool parallel_screen) {
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
    tail+4*n, tail+4*n+r, tail+4*n+2*r, work+2*n, patches, activity};
  auto stage = inputs_prepared ? cin::detail::TransferForceTrial(model, force)
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
    if (!frozen_values::Screen(sources, structural.factor, result, invalid_node)) {
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
__global__ void CompleteCin(const cin_advance::Input input) {
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

}
} // namespace

cudaError_t LaunchFrozen(const Input& input, cudaStream_t stream) {
  const bool parallel_inputs = input.input_failure != nullptr;
  auto error = cudaSuccess;
  if (parallel_inputs) {
    error = force_inputs::Launch(input, stream);
    if (error != cudaSuccess) return error;
  }
  const bool parallel_screen = input.screen && frozen_screen::Blocks(input.model.node_count) &&
      input.structural.profile != NodalCinStructuralProfile::Disabled;
  PrepareCin<<<1,1,0,stream>>>(input, parallel_inputs, parallel_screen);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  if (parallel_screen) {
    error = frozen_screen::Launch(input, stream);
    if (error != cudaSuccess) return error;
  }
  constexpr unsigned threads = 128;
  const auto blocks = (input.model.node_count+threads-1)/threads;
  AdvanceOrdinaryCin<<<blocks,threads,0,stream>>>(input);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  CompleteCin<<<1,1,0,stream>>>(input);
  error = cudaGetLastError();
  if (error != cudaSuccess) return error;
  return capture::Launch(input, stream);
}

} // namespace tl::fea::cin_group_test
