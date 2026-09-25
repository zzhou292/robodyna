// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
// GPU execution boundary only. Shared values/layout do not include CUDA headers.
#include "Device.h"
#include <cuda_runtime_api.h>
namespace tlfea::contact::radioss_type25::search::detail {
cudaError_t Run(Device, const Current&, unsigned slab, bool capture,
    double margin, double previous_dt, bool force_sort, bool has_reference,
    cudaStream_t) noexcept;
} // namespace tlfea::contact::radioss_type25::search::detail
