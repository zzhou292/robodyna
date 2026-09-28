// Literal two-fence control flow from7695f855 Transaction::AssembleAccepted.
#pragma once
#include "lib_src/collision/radioss_type25/runtime/Types.h"
#include <cuda_runtime_api.h>
namespace native_tail_test {
namespace n=tlfea::contact::radioss_type25;
template<class GatherStage,class ApplyStage,class Fence>
n::TransactionReport FrozenTail(GatherStage&& gather,ApplyStage&& apply,Fence&& fence) noexcept {
  auto status=fence(gather());if(status.status!=n::TransactionStatus::Ok)return status;
  status=fence(apply());if(status.status!=n::TransactionStatus::Ok)return status;
  return {n::TransactionStatus::Ok,"OK"};
}
}
