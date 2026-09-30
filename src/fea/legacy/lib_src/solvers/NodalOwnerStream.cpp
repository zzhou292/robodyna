// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FENodalStateStorage.h"

#include "NodalTrialIdentity.h"

namespace tl::fea {
namespace {

NodalReport Ok() noexcept {
  return {NodalStatus::Ok, "OK"};
}

}  // namespace

NodalReport FENodalState::ValidateOwnerStream(
    cudaStream_t stream) const noexcept {
  if (!impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  const auto& state = *impl_;
  if (!state.usable)
    return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  if (!stream || stream == cudaStreamLegacy ||
      stream == cudaStreamPerThread || stream != state.stream)
    return {NodalStatus::InvalidInput,
            "Stream is not the actual nondefault owner stream"};
  return Ok();
}

NodalReport FENodalState::BorrowOwnerStream(
    cudaStream_t* output) const noexcept {
  if (!impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  const auto& state = *impl_;
  if (!state.usable)
    return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  if (!output ||
      !trial_identity::Disjoint(
          output, sizeof(*output), this, sizeof(*this)) ||
      !trial_identity::Disjoint(
          output, sizeof(*output), &state, sizeof(state)))
    return {NodalStatus::InvalidOutput,
            "Owner stream output is absent or aliases owner control"};
  *output = state.stream;
  return Ok();
}

}  // namespace tl::fea
