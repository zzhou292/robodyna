#include "FailureResidentFixture.h"
#include <limits>
namespace resident_failure_test {
namespace {
Fault fault=Fault::None;
unsigned copies=0;
}
void Arm(Fault value) {fault=value;copies=0;}
unsigned Copies() {return copies;}
extern "C" cudaError_t __real_cudaMemcpyAsync(void*,const void*,std::size_t,cudaMemcpyKind,cudaStream_t);
extern "C" cudaError_t __wrap_cudaMemcpyAsync(void* output,const void* input,std::size_t bytes,cudaMemcpyKind kind,cudaStream_t stream) {
  const auto status=__real_cudaMemcpyAsync(output,input,bytes,kind,stream);
  if(fault==Fault::None||kind!=cudaMemcpyDeviceToHost||++copies!=3||status!=cudaSuccess)return status;
  const auto mode=fault;
  fault=Fault::None;
  const auto completed=cudaStreamSynchronize(stream);
  if(completed!=cudaSuccess)return completed;
  if(mode==Fault::DeviceError)return cudaErrorInvalidValue;
  if(bytes!=Parents*sizeof(fe::ShellBatchFailureState))return cudaErrorInvalidValue;
  auto& last=static_cast<fe::ShellBatchFailureState*>(output)[Parents-1];
  if(mode==Fault::InvalidFlag)*reinterpret_cast<unsigned char*>(&last.active)=2;
  else last.current_force_point[2].stress[4]=std::numeric_limits<double>::quiet_NaN();
  return cudaSuccess;
}
} // namespace resident_failure_test
