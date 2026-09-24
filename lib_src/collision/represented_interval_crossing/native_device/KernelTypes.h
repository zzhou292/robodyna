// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../native/CellKernel.h"
#include "../native/FixedIntegerPolicy.h"
namespace tlfea::contact::represented_interval_crossing::native_device {
using Kernel = native::CellKernel<512, native::FixedIntegerPolicy<512>>;
using ExactScratch = Kernel::ExactScratch;
inline constexpr unsigned MaximumDeviceWorkers = 128;
inline constexpr unsigned ThreadsPerBlock = 32;
static_assert(sizeof(ExactScratch) <= 8192);
}  // namespace tlfea::contact::represented_interval_crossing::native_device
