// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Input.h"
#include "../../constraints/tied_shell/runtime/CinForceStage.h"

#if defined(__CUDACC__)
#define TL_CIN_INPUT_HD __host__ __device__
#else
#define TL_CIN_INPUT_HD
#endif

namespace tl::fea::cin_advance::force_inputs {
namespace cin = constraints::tied_shell::cin;

TL_CIN_INPUT_HD inline void RejectStage(const Input& input, cin::StageReport stage) noexcept {
  input.control->status = stage.status == cin::StageStatus::PendingReleaseEligibility
      ? NodalStatus::MissingStepAdmission : NodalStatus::InvalidOutput;
  input.control->node = stage.node;
}

// The same owner/control checks precede every force pointer or numerical read.
TL_CIN_INPUT_HD inline bool CheckPrefix(const Input& input) noexcept {
  auto* control = input.control;
  if (control->status != NodalStatus::Ok) return false;
  if (control->rows.base_epoch != input.epoch || control->rows.attempt != input.attempt ||
      !stability::IsCurrentLimit(control->rows, control->limit) ||
      control->limit.dt < input.durations.drift_dt) {
    control->status = NodalStatus::StaleTrial;
    control->node = UINT32_MAX;
    return false;
  }
  if (control->limit.has_stiffness_or_damping) {
    control->status = NodalStatus::MissingStepAdmission;
    control->node = UINT32_MAX;
    return false;
  }
  return true;
}

TL_CIN_INPUT_HD inline cin::ForceTrial ForceView(const Input& input) noexcept {
  const auto n = input.model.node_count;
  const auto r = input.model.row_count;
  return {input.accepted, input.loads, input.tail, input.tail+n, input.work, input.work+n,
      r ? input.tail+4*n : nullptr, r ? input.tail+4*n+r : nullptr, input.tail+4*n+2*r, input.work+2*n,
      input.patches, input.activity};
}

TL_CIN_INPUT_HD inline bool Begin(const Input& input) noexcept {
  if (!CheckPrefix(input)) return false;
  const auto report = cin::detail::CheckForcePointers(input.model, ForceView(input));
  if (!report) {
    RejectStage(input, report);
    return false;
  }
  // This key is distinct from the ordinary-motion key. Rejected force inputs
  // must leave that later phase's incoming sentinel and entry IN untouched.
  *input.input_failure = NoFailure;
  return true;
}

TL_CIN_INPUT_HD inline bool Complete(const Input& input) noexcept {
  if (input.control->status != NodalStatus::Ok) return false;
  if (*input.input_failure != NoFailure) {
    RejectStage(input, {cin::StageStatus::InvalidInput, UINT32_MAX,
        static_cast<std::uint32_t>(*input.input_failure)});
    return false;
  }
  const auto report = cin::detail::CheckForceAfterNodes(input.model, ForceView(input));
  if (!report) {
    RejectStage(input, report);
    return false;
  }
  return true;
}

// Resolve one complete phase before beginning independent checks for the next.
// The existing key is reused only after its prior value has been consumed.
TL_CIN_INPUT_HD inline bool CompleteNodes(const Input& input) noexcept {
  if (input.control->status != NodalStatus::Ok) return false;
  if (*input.input_failure != NoFailure) {
    RejectStage(input, {cin::StageStatus::InvalidInput, UINT32_MAX,
        static_cast<std::uint32_t>(*input.input_failure)});
    return false;
  }
  if (!constraints::tied_shell::detail::math::Finite(*ForceView(input).numerical_mass)) {
    RejectStage(input, {cin::StageStatus::InvalidInput});
    return false;
  }
  return true;
}
TL_CIN_INPUT_HD inline bool CompleteWitnesses(const Input& input) noexcept {
  if (input.control->status != NodalStatus::Ok) return false;
  if (*input.input_failure != NoFailure) {
    RejectStage(input, {cin::StageStatus::SourceMismatch});
    return false;
  }
  return true;
}
TL_CIN_INPUT_HD inline bool CompleteRows(const Input& input) noexcept {
  if (input.control->status != NodalStatus::Ok) return false;
  if (*input.input_failure != NoFailure) {
    RejectStage(input, cin::detail::CheckForceRow(input.model, ForceView(input),
        static_cast<std::uint32_t>(*input.input_failure)));
    return false;
  }
  return true;
}

cudaError_t Launch(const Input&, cudaStream_t);
} // namespace tl::fea::cin_advance::force_inputs
#undef TL_CIN_INPUT_HD
