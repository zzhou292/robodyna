#include "MixedResidentFixture.h"
#include <limits>
namespace mixed_layered_test {
namespace { ReadFault fault=ReadFault::None;unsigned copies=0; }
void ArmReadFault(ReadFault next) {fault=next;copies=0;}
unsigned ReadFaultCopies() {return copies;}
extern "C" cudaError_t __real_cudaMemcpyAsync(void*,const void*,std::size_t,cudaMemcpyKind,cudaStream_t);
extern "C" cudaError_t __wrap_cudaMemcpyAsync(void* output,const void* input,std::size_t bytes,
    cudaMemcpyKind kind,cudaStream_t stream) {
  const auto status=__real_cudaMemcpyAsync(output,input,bytes,kind,stream);
  if(fault==ReadFault::None||kind!=cudaMemcpyDeviceToHost||++copies!=2||status!=cudaSuccess)return status;
  const auto armed=fault;fault=ReadFault::None;
  const auto completed=cudaStreamSynchronize(stream);if(completed!=cudaSuccess)return completed;
  if(armed==ReadFault::DeviceError)return cudaErrorInvalidValue;
  if(bytes!=Parents*sizeof(fe::sections::ShellLayeredLaw1History))return cudaErrorInvalidValue;
  static_cast<fe::sections::ShellLayeredLaw1History*>(output)[Parents-1].point[2].stress[4]=
    std::numeric_limits<double>::quiet_NaN();
  return cudaSuccess;
}
} // namespace mixed_layered_test
