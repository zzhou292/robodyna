// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ActivityRuntime.h"
#include <algorithm>
namespace tlfea::contact::radioss_type25::runtime_detail {
namespace {
__global__ void Transfer(const double* coefficients,lifecycle::Secondary* rows,std::size_t count) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<count;i+=blockDim.x*gridDim.x)
    rows[i].coefficient=coefficients[i];
}
}
cudaError_t ApplyActivitySecondary(const double* coefficients,lifecycle::Secondary* rows,
    std::size_t count,cudaStream_t stream) noexcept {
  if(!count)return cudaSuccess;
  if(!coefficients||!rows)return cudaErrorInvalidValue;
  const auto blocks=unsigned(std::min<std::size_t>(256,(count+127)/128));
  Transfer<<<blocks,128,0,stream>>>(coefficients,rows,count);
  return cudaGetLastError();
}
}
