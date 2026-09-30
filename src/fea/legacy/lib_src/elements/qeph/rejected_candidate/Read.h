// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "../../ShellDiagnosticDeviceSources.h"
#include <cstdint>
namespace tl::fea::qeph::rejected_detail {
// Diagnostic path only. Synchronize every small borrowed transfer so no stack
// destination outlives a pending DMA, including failure exits.
struct Reader {
  cudaStream_t stream=nullptr;
  cudaError_t error=cudaSuccess;
  bool Bytes(void* output,const void* input,std::size_t bytes) noexcept {
    if(error!=cudaSuccess)return false;
    const auto copy=cudaMemcpyAsync(output,input,bytes,cudaMemcpyDeviceToHost,stream);
    const auto completed=cudaStreamSynchronize(stream);
    error=copy!=cudaSuccess?copy:completed;return error==cudaSuccess;
  }
  template<class T> bool Value(T& output,const T* input) noexcept {return input&&Bytes(&output,input,sizeof(T));}
};
inline bool CurveRange(const double* pointer,const double* base,std::size_t capacity,
                       std::size_t count) noexcept {
  if(!pointer||!base||count>MaxShellPlasticityCurvePoints||count>capacity||capacity>SIZE_MAX/sizeof(double))return false;
  const auto p=reinterpret_cast<std::uintptr_t>(pointer),b=reinterpret_cast<std::uintptr_t>(base);
  if(p<b||p-b>capacity*sizeof(double)||(p-b)%sizeof(double))return false;
  return count<=capacity-(p-b)/sizeof(double);
}
bool ReadMaterial(const shell_batch_plasticity_detail::HostStorage*,std::size_t,unsigned,
                  RejectedCandidateInput&,Reader&) noexcept;
} // namespace tl::fea::qeph::rejected_detail
