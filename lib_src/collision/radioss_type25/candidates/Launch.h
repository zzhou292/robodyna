// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Device.h"
#include <cuda_runtime.h>
namespace tlfea::contact::radioss_type25::candidates::detail {
cudaError_t QueryScratch(const Source&,Limits,std::size_t&) noexcept;
cudaError_t BuildRanges(Device,const Current&,cudaStream_t) noexcept;
cudaError_t CountPairs(Device,const Current&,std::size_t,cudaStream_t) noexcept;
cudaError_t FillPairs(Device,const Current&,std::size_t,std::size_t,cudaStream_t) noexcept;
} // namespace tlfea::contact::radioss_type25::candidates::detail
