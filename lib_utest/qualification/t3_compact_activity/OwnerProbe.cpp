// SPDX-License-Identifier: MIT
#include "OwnerProbe.h"
#include "lib_src/elements/t3/mapped/ActivityValues.h"
namespace t3_readback_test { Transfers transfers; }
namespace t3_compact_owner { Probe probe; }
extern "C" cudaError_t __real_cudaMemcpyAsync(void*,const void*,std::size_t,cudaMemcpyKind,cudaStream_t);
extern "C" cudaError_t __real_cudaStreamSynchronize(cudaStream_t);
extern "C" cudaError_t __real_cudaGetLastError();
extern "C" cudaError_t __wrap_cudaMemcpyAsync(void* output,const void* input,std::size_t bytes,
    cudaMemcpyKind kind,cudaStream_t stream) {
  auto& t=t3_readback_test::transfers;auto& p=t3_compact_owner::probe;
  const bool watched=t.enabled&&kind==cudaMemcpyDeviceToHost;
  if(p.capture_sources&&kind==cudaMemcpyDeviceToHost) {
    if(bytes==sizeof(tl::fea::t3::ForceTrial))p.force_device=input;
    if(bytes==sizeof(tl::fea::ShellBatchOnePointSectionState)&&++p.point_sized_copies==1)p.point_device=input;
  }
  if(watched){++t.copies;t.bytes+=bytes;if(bytes==sizeof(tl::fea::t3::ForceTrial)){++t.force_copies;t.force_host=output;t.force_device=input;}if(bytes==5)p.packet_host=output;
    if(t.copies==p.fail_copy)return cudaErrorInvalidValue;}
  const auto result=__real_cudaMemcpyAsync(output,input,bytes,kind,stream);
  if(result==cudaSuccess&&watched&&bytes==5&&p.packet_corruption==t.copies) {
    const auto drained=__real_cudaStreamSynchronize(stream);if(drained!=cudaSuccess)return drained;
    if(t.copies==2||t.copies==3||t.copies==4)static_cast<unsigned char*>(output)[4]=255;
    else {const std::uint32_t malformed=0;std::memcpy(output,&malformed,4);}
  }
  return result;
}
extern "C" cudaError_t __wrap_cudaStreamSynchronize(cudaStream_t stream) {
  const auto result=__real_cudaStreamSynchronize(stream);
  auto& t=t3_readback_test::transfers;
  if(t.enabled&&++t.syncs==t3_compact_owner::probe.fail_sync)return cudaErrorInvalidValue;
  return result;
}
extern "C" cudaError_t __wrap_cudaGetLastError() {
  const auto result=__real_cudaGetLastError();
  auto& t=t3_readback_test::transfers;auto& p=t3_compact_owner::probe;
  if(t.enabled&&++p.error_checks==p.fail_error)return cudaErrorInvalidValue;
  return result;
}
