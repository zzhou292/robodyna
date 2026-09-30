#pragma once
#include "QephBatch.h"
#include "../ShellBatchFields.h"

namespace tl::fea::qeph::batch_detail {
inline bool ValidStartup(const QephBatchConfig& config,bool joined) {
  (void)joined;
  return shell_startup_detail::ValidStartup(config.startup,config.usage==BatchUsage::CoupledForces);
}
// Same finite binary64 reduction on host metadata and actual device inputs.
// The host preflight admits its operation domain; it does not publish a
// measured diagnostic. The device reads each actual accepted node only after
// complete initial-motion/mass validation. Failure preserves the partial sum.
using shell_startup_detail::AddInitialTranslationKinetic;
} // namespace tl::fea::qeph::batch_detail
