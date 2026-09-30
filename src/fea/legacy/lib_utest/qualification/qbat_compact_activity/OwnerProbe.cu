// SPDX-License-Identifier: MIT
#include "OwnerProbe.h"
#include "lib_src/elements/qbat/QbatBatchTypes.h"
#include <cuda_runtime.h>
namespace qbat_activity_test {
namespace {
std::size_t full_bytes=0,packet_bytes=0;
const void* full_source=nullptr;
const void* packet_device=nullptr;
void* packet_host=nullptr;
ErrorPhase error=ErrorPhase::None;
bool memset_seen=false,copy_seen=false;
unsigned hits=0;
}
void CaptureFullSource(std::size_t parents) { full_bytes=parents*sizeof(tl::fea::qbat::BatchResult);full_source=nullptr; }
const void* FullSource(){return full_source;}
void CapturePacket(std::size_t parents) {packet_bytes=sizeof(unsigned)+parents;packet_host=nullptr;packet_device=nullptr;}
void* PacketHost(){return packet_host;}
std::size_t PacketBytes(){return packet_bytes;}
bool ArmError(ErrorPhase phase) {
  if(!packet_device) return false;
  error=phase;memset_seen=copy_seen=false;hits=0;return true;
}
unsigned ErrorHits(){return hits;}
extern "C" cudaError_t __real_cudaMemcpyAsync(void*,const void*,std::size_t,cudaMemcpyKind,cudaStream_t);
extern "C" cudaError_t __real_cudaMemsetAsync(void*,int,std::size_t,cudaStream_t);
extern "C" cudaError_t __real_cudaGetLastError();
extern "C" cudaError_t __real_cudaStreamSynchronize(cudaStream_t);
extern "C" cudaError_t __wrap_cudaMemcpyAsync(void* destination,const void* source,std::size_t bytes,
    cudaMemcpyKind kind,cudaStream_t stream) {
  if(kind==cudaMemcpyDeviceToHost && full_bytes && bytes==full_bytes) {full_source=source;full_bytes=0;}
  if(kind==cudaMemcpyDeviceToHost && packet_bytes && bytes==packet_bytes) {
    packet_host=destination;packet_device=source;
    if(memset_seen) {
      copy_seen=true;
      if(error==ErrorPhase::Copy) {error=ErrorPhase::None;++hits;return cudaErrorInvalidValue;}
    }
  }
  return __real_cudaMemcpyAsync(destination,source,bytes,kind,stream);
}
extern "C" cudaError_t __wrap_cudaMemsetAsync(void* destination,int value,std::size_t bytes,cudaStream_t stream) {
  if(destination==packet_device && bytes==sizeof(unsigned) && value==0xff && error!=ErrorPhase::None) {
    memset_seen=true;
    if(error==ErrorPhase::Memset) {error=ErrorPhase::None;++hits;return cudaErrorInvalidValue;}
  }
  return __real_cudaMemsetAsync(destination,value,bytes,stream);
}
extern "C" cudaError_t __wrap_cudaGetLastError() {
  const auto result=__real_cudaGetLastError();
  if(memset_seen && error==ErrorPhase::Launch) {error=ErrorPhase::None;++hits;return cudaErrorInvalidConfiguration;}
  return result;
}
extern "C" cudaError_t __wrap_cudaStreamSynchronize(cudaStream_t stream) {
  const auto result=__real_cudaStreamSynchronize(stream);
  if(copy_seen && error==ErrorPhase::Drain) {error=ErrorPhase::None;++hits;return cudaErrorLaunchFailure;}
  return result;
}
} // namespace qbat_activity_test
