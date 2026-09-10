#include "Timing.h"
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>

namespace robo_dyna::cuda_api_timing {
namespace {
struct Json {
    char bytes[ReportCap]{};std::size_t size=0;bool valid=true;
    template<class... A> void Append(const char* format,A... args) noexcept {
        if(!valid)return;
        int n;
        if constexpr(sizeof...(A)==0)n=std::snprintf(bytes+size,ReportCap-size,"%s",format);
        else n=std::snprintf(bytes+size,ReportCap-size,format,args...);
        if(n<0||static_cast<std::size_t>(n)>=ReportCap-size) {valid=false;return;}size+=static_cast<std::size_t>(n);
    }
};
std::uint64_t Read(const std::atomic<std::uint64_t>& n) noexcept {return n.load(std::memory_order_relaxed);}
void Warning() noexcept {
    constexpr char text[]="robo-dyna CUDA timing: create-only report could not be completed\n";
    const auto ignored=write(STDERR_FILENO,text,sizeof(text)-1);(void)ignored;
}
void Bytes(Json& j,const char* name,const std::atomic<std::uint64_t>* bytes) noexcept {
    constexpr const char* names[]{"host_to_host","host_to_device","device_to_host","device_to_device","runtime_default","unknown"};
    j.Append("\"%s\":{",name);
    for(unsigned i=0;i<DirectionCount;++i)j.Append("%s\"%s\":%" PRIu64,i?",":"",names[i],Read(bytes[i]));
    j.Append("}");
}
}
void EmitReport(const State& s,std::uint64_t end) noexcept {
    Json j;
    j.Append("{\"schema\":\"robo_dyna.cuda_api_timing.v1\",\"clock\":\"CLOCK_MONOTONIC\",\"pid\":%ld,",
             static_cast<long>(getpid()));
    j.Append("\"process_interval_ns\":%" PRIu64 ",\"active_calls_at_shutdown\":%" PRIu64 ",",end>=s.started_ns?end-s.started_ns:0,s.active_at_shutdown);
    j.Append("\"nested_calls_skipped\":%" PRIu64 ",\"clock_failures\":%" PRIu64 ",\"counter_saturated\":%s,\"missing_symbols\":%u,",
        Read(s.nested),Read(s.clock_failures),s.saturated.load(std::memory_order_relaxed)?"true":"false",s.missing_symbols);
    j.Append("\"scope\":\"Outer public CUDA runtime API wall time, including work waited for; not device kernel time. Concurrent API intervals may overlap. No implicit synchronization.\",\"functions\":[");
    constexpr const char* names[]{"cudaMemcpyAsync","cudaStreamSynchronize","cudaMemcpy","cudaDeviceSynchronize"};
    for(unsigned i=0;i<ApiCount;++i) {
        const auto& c=s.api[i];j.Append("%s{\"name\":\"%s\",\"calls\":%" PRIu64 ",\"failures\":%" PRIu64
            ",\"wall_ns\":%" PRIu64 ",\"maximum_ns\":%" PRIu64 ",",i?",":"",names[i],Read(c.calls),Read(c.failures),Read(c.wall_ns),Read(c.maximum_ns));
        Bytes(j,"requested_bytes",c.requested);j.Append(",");Bytes(j,"successful_bytes",c.successful);j.Append("}");
    }
    j.Append("]}\n");if(!j.valid) {Warning();return;}
    const int fd=open(s.output,O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600);
    if(fd<0) {Warning();return;}
    bool complete=true;std::size_t written=0;
    while(written<j.size) {
        const auto n=write(fd,j.bytes+written,j.size-written);
        if(n<0&&errno==EINTR)continue;if(n<=0) {complete=false;break;}written+=static_cast<std::size_t>(n);
    }
    if(close(fd)!=0)complete=false;if(!complete)Warning();
}
} // namespace robo_dyna::cuda_api_timing
