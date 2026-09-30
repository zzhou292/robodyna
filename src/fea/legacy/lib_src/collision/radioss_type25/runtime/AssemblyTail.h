// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include <cuda_runtime_api.h>
namespace tlfea::contact::radioss_type25::runtime_detail {
// Private host sequencing for the one failure-gated assembly tail. Gather only
// writes private scratch; Apply observes its stable failure key before any write.
// Callables keep the unchanged launch/Fence operations visible to qualification.
template<class GatherStage,class ApplyStage,class Fence,class ClearLaunchStatus>
TransactionReport AssembleTail(GatherStage&& gather,ApplyStage&& apply,Fence&& fence,
    ClearLaunchStatus&& clear_launch_status) noexcept {
  const auto gathered=gather();
  if(gathered!=cudaSuccess)return fence(gathered); // Original first-launch handling.
  const auto applied=apply();
  if(applied==cudaSuccess)return fence(cudaSuccess);
  // Apply's cudaPeekAtLastError leaves a failed launch in this host thread.
  // Clear it only on this exceptional path so the prior Gather can be observed
  // as the old intermediate Fence would have done. Stream/copy faults still
  // poison through Fence. A prior semantic rejection retains first-error priority.
  clear_launch_status();
  const auto prior=fence(cudaSuccess);
  if(prior.status!=TransactionStatus::Ok)return prior;
  return fence(applied); // Gather passed: preserve original Apply error/poisoning.
}
} // namespace tlfea::contact::radioss_type25::runtime_detail
