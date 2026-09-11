// SPDX-License-Identifier: MIT
#include "ResidentFixture.h"

namespace qbat_resident_test {
namespace {
ReadFault armed=ReadFault::None;
std::size_t extent=0;
}
void Arm(ReadFault fault,std::size_t parents) {
  armed=fault;
  extent=parents*sizeof(qb::BatchResult);
}
extern "C" cudaError_t __real_cudaMemcpyAsync(void*,const void*,std::size_t,cudaMemcpyKind,cudaStream_t);
extern "C" cudaError_t __wrap_cudaMemcpyAsync(void* destination,const void* source,std::size_t bytes,
    cudaMemcpyKind kind,cudaStream_t stream) {
  if(kind!=cudaMemcpyDeviceToHost||bytes!=extent||armed==ReadFault::None) {
    return __real_cudaMemcpyAsync(destination,source,bytes,kind,stream);
  }
  const auto fault=armed;
  armed=ReadFault::None;
  if(fault==ReadFault::CopyError) return cudaErrorInvalidValue;
  const auto copied=__real_cudaMemcpyAsync(destination,source,bytes,kind,stream);
  if(copied!=cudaSuccess) return copied;
  const auto completed=cudaStreamSynchronize(stream);
  if(completed!=cudaSuccess) return completed;
  auto& last=static_cast<qb::BatchResult*>(destination)[bytes/sizeof(qb::BatchResult)-1];
  if(fault==ReadFault::LateNonfinite) last.point[3].material.equivalent_stress_pa=std::numeric_limits<double>::quiet_NaN();
  if(fault==ReadFault::InvalidFlag) {
    const unsigned char invalid=2;
    std::memcpy(&last.history.point[3].surface_active,&invalid,1);
  }
  return cudaSuccess;
}
} // namespace qbat_resident_test
