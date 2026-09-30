// SPDX-License-Identifier: AGPL-3.0-or-later
// Deliberately compiled at O0 and linked only to the CXX forecast library.
// No CUDA runtime, owner construction, GPU query or section GC is required.
#include "lib_src/collision/self_contact_transaction/CrossingExecutor.h"
#include "lib_src/collision/SelfContactTransactionTypes.h"
int main() {
  using namespace tlfea::contact;
  SelfContactTransactionConfig config;
  SelfContactTransactionLimits limits;
  const auto cpu = self_contact_transaction::CrossingExecutor::Preflight(config, limits);
  config.enable_cuda_native_crossing = true;
  const auto gpu = self_contact_transaction::CrossingExecutor::Preflight(config, limits);
  return cpu.report.status != RepresentedIntervalStatus::Ok || cpu.device_bytes != 0 ||
      gpu.report.status != RepresentedIntervalStatus::Ok || !gpu.device_bytes ||
      gpu.device_allocations != 1;
}
