#include "Probe.h"
#include "lib_src/elements/t3/T3Batch.h"
namespace native_group_activity_test {Probe probe;}
extern "C" cudaError_t __real_cudaMemcpyAsync(void*,const void*,std::size_t,cudaMemcpyKind,cudaStream_t);
extern "C" cudaError_t __real_cudaStreamSynchronize(cudaStream_t);
extern "C" cudaError_t __wrap_cudaMemcpyAsync(void* out,const void* in,std::size_t bytes,cudaMemcpyKind kind,cudaStream_t stream){
 auto&p=native_group_activity_test::probe;
 if(p.capture_force&&kind==cudaMemcpyDeviceToHost&&bytes==sizeof(tl::fea::t3::ForceTrial))p.force=in;
 if(p.enabled&&kind==cudaMemcpyDeviceToHost){++p.copies;p.bytes+=bytes;if(p.copies==p.fail_copy)return cudaErrorInvalidValue;}
 return __real_cudaMemcpyAsync(out,in,bytes,kind,stream);
}
extern "C" cudaError_t __wrap_cudaStreamSynchronize(cudaStream_t stream){
 if(native_group_activity_test::probe.enabled)++native_group_activity_test::probe.syncs;
 return __real_cudaStreamSynchronize(stream);
}
