#include <cuda_runtime_api.h>
#include <atomic>
#include <cerrno>
#include <cstdlib>
#include <cstring>

namespace {
std::atomic<void*> last_dst{nullptr};std::atomic<const void*> last_src{nullptr};
std::atomic<std::size_t> last_count{0};std::atomic<int> last_kind{0},incoming_errno{0};
std::atomic<cudaStream_t> last_stream{nullptr};std::atomic<unsigned> async_count{0};
std::atomic<int> early_ok{0};
}
extern "C" int fake_last_matches(void* dst,const void* src,std::size_t count,int kind,cudaStream_t stream) {
    return last_dst==dst&&last_src==src&&last_count==count&&last_kind==kind&&last_stream==stream;
}
extern "C" int fake_incoming_errno() {return incoming_errno;}
extern "C" unsigned fake_async_calls() {return async_count;}
extern "C" int fake_early_ok() {return early_ok;}
extern "C" cudaError_t CUDARTAPI cudaMemcpyAsync(void* dst,const void* src,std::size_t n,cudaMemcpyKind kind,cudaStream_t stream) {
    incoming_errno=errno;last_dst=dst;last_src=src;last_count=n;last_kind=kind;last_stream=stream;++async_count;
    if(stream==reinterpret_cast<cudaStream_t>(0x99)) {errno=EAGAIN;return cudaErrorInvalidValue;}
    std::memcpy(dst,src,n);errno=E2BIG;return cudaSuccess;
}
extern "C" cudaError_t CUDARTAPI cudaStreamSynchronize(cudaStream_t stream) {
    if(stream==reinterpret_cast<cudaStream_t>(0x22)) {errno=ERANGE;return cudaErrorNotReady;}
    if(stream==reinterpret_cast<cudaStream_t>(0x33)) {
        char source[5]{1,2,3,4,5},destination[5]{};
        const auto r=cudaMemcpyAsync(destination,source,5,cudaMemcpyHostToHost,reinterpret_cast<cudaStream_t>(0x44));
        if(r!=cudaSuccess||std::memcmp(source,destination,5))return cudaErrorUnknown;
    }
    errno=E2BIG;return cudaSuccess;
}
extern "C" cudaError_t CUDARTAPI cudaMemcpy(void* dst,const void* src,std::size_t n,cudaMemcpyKind kind) {
    if(static_cast<int>(kind)<0) {errno=EDOM;return cudaErrorInvalidMemcpyDirection;}
    std::memcpy(dst,src,n);errno=E2BIG;return cudaSuccess;
}
extern "C" cudaError_t CUDARTAPI cudaDeviceSynchronize() {errno=E2BIG;return cudaSuccess;}

// A dependency DSO initializes before the preload. Its first call must invoke
// the one-time lazy resolver and still preserve arguments, return and errno.
__attribute__((constructor)) static void EarlyDependencyCall() {
    if(std::getenv("ROBO_DYNA_FAKE_CUDA_EARLY_CALL")) {
        char source=1,destination=0;
        errno=EBUSY;
        const auto result=cudaMemcpyAsync(&destination,&source,1,cudaMemcpyHostToHost,nullptr);
        early_ok=result==cudaSuccess&&errno==E2BIG&&destination==source&&incoming_errno==EBUSY&&
            fake_last_matches(&destination,&source,1,cudaMemcpyHostToHost,nullptr);
    }
}
