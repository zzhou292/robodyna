#include "Frozen.h"
#include "FrozenStability.h"
namespace tl::fea::reset_frozen {
using nodal_detail::Control;
namespace sc = tlfea::contact;
namespace stability = reset_frozen_stability;
#include "frozen/ResetTrial.cuh"
}
namespace tl::fea::reset_test {
void FrozenLaunch(nodal_detail::Control* control, std::uint64_t epoch,
                  std::uint64_t attempt, cudaStream_t stream) {
  reset_frozen::ResetTrial<<<1,1,0,stream>>>(control, epoch, attempt);
}
}
