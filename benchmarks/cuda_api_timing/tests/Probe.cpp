#include <cuda_runtime_api.h>
#include <atomic>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>

extern "C" int fake_last_matches(void*,const void*,std::size_t,int,cudaStream_t);
extern "C" int fake_incoming_errno();
extern "C" unsigned fake_async_calls();
extern "C" int fake_early_ok();
int main() {
    const bool early=std::getenv("ROBO_DYNA_FAKE_CUDA_EARLY_CALL");
    if(early&&!fake_early_ok())return 11;
    char source[32]="unchanged call arguments",destination[32]{};auto stream=reinterpret_cast<cudaStream_t>(0x11);
    errno=EBUSY;
    if(cudaMemcpyAsync(destination,source,7,cudaMemcpyHostToDevice,stream)!=cudaSuccess||errno!=E2BIG||
       std::memcmp(source,destination,7)||!fake_last_matches(destination,source,7,cudaMemcpyHostToDevice,stream)||fake_incoming_errno()!=EBUSY)return 1;
    stream=reinterpret_cast<cudaStream_t>(0x99);errno=EBUSY;
    if(cudaMemcpyAsync(destination,source,13,cudaMemcpyDeviceToHost,stream)!=cudaErrorInvalidValue||errno!=EAGAIN||
       !fake_last_matches(destination,source,13,cudaMemcpyDeviceToHost,stream)||fake_incoming_errno()!=EBUSY)return 2;
    if(cudaStreamSynchronize(reinterpret_cast<cudaStream_t>(0x22))!=cudaErrorNotReady||errno!=ERANGE)return 3;
    if(cudaStreamSynchronize(reinterpret_cast<cudaStream_t>(0x33))!=cudaSuccess||errno!=E2BIG)return 4;
    if(cudaMemcpy(destination,source,9,cudaMemcpyDeviceToDevice)!=cudaSuccess||errno!=E2BIG||std::memcmp(destination,source,9))return 5;
    if(cudaMemcpy(destination,source,3,static_cast<cudaMemcpyKind>(-7))!=cudaErrorInvalidMemcpyDirection||errno!=EDOM)return 6;
    if(cudaDeviceSynchronize()!=cudaSuccess||errno!=E2BIG)return 7;
    std::atomic<unsigned> failed{0};std::thread threads[4];
    for(auto& thread:threads)thread=std::thread([&] {
        char a[11]="concurrent",b[11]{};
        for(unsigned i=0;i<100;++i)if(cudaMemcpyAsync(b,a,11,cudaMemcpyHostToHost,reinterpret_cast<cudaStream_t>(0x55))!=cudaSuccess||
            std::memcmp(a,b,11)||errno!=E2BIG)++failed;
    });
    for(auto& thread:threads)thread.join();if(failed)return 8;
    if(cudaMemcpyAsync(destination,source,4,cudaMemcpyDefault,stream=nullptr)!=cudaSuccess||errno!=E2BIG)return 9;
    if(fake_async_calls()!=404+static_cast<unsigned>(early))return 10;
    std::cout<<"fake CUDA forwarding passed\n";return 0;
}
