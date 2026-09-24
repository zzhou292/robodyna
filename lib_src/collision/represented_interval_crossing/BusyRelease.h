// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <atomic>
namespace tlfea::contact::represented_interval_crossing {
// Shared synchronous owner lease. Declaration follows a successful exchange;
// every later exit releases the same owner before a new caller may mutate it.
struct BusyRelease {
  std::atomic<bool>* value;
  ~BusyRelease() { value->store(false, std::memory_order_release); }
};
}  // namespace tlfea::contact::represented_interval_crossing
