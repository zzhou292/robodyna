#include "StepTimingReport.h"
#include <cerrno>
#include <fcntl.h>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <unistd.h>

namespace crash::cases::source_assembly_wall {
namespace {
constexpr std::size_t ReportCap=32*1024;
void Counters(std::ostream& out,const source_assembly_dynamics::StepTimingSnapshot& s,bool last) {
    const auto& counters=last?s.last_step:s.total;
    out<<'[';
    for(std::size_t i=0;i<counters.size();++i) {
        const auto& c=counters[i];
        out<<(i?",":"")<<"{\"stage\":\""<<source_assembly_dynamics::StepStageNames[i]
            <<"\",\"calls\":"<<c.calls<<",\"failures\":"<<c.failures<<",\"valid_samples\":"<<c.valid_samples
            <<",\"wall_ns\":"<<c.wall_ns<<",\"maximum_ns\":"<<c.maximum_ns<<'}';
    }
    out<<']';
}
}
void CheckStepTimingPath(const std::filesystem::path& output,const std::filesystem::path& archive) {
    if(output.empty()||archive.empty())throw std::invalid_argument("Timing output and archive paths must be nonempty");
    const auto destination=std::filesystem::weakly_canonical(std::filesystem::absolute(output));
    const auto root=std::filesystem::weakly_canonical(std::filesystem::absolute(archive));
    auto p=destination.begin(),r=root.begin();
    for(;p!=destination.end()&&r!=root.end()&&*p==*r;++p,++r) {}
    if(r==root.end())throw std::invalid_argument("Stage timing output must be outside the accepted archive");
}
std::string FormatStepTiming(const source_assembly_dynamics::StepTimingSnapshot& s,int status) {
    std::ostringstream out;out.imbue(std::locale::classic());
    out<<"{\"schema\":\"robo_dyna.source_assembly_step_timing.v1\",\"clock\":\"CLOCK_MONOTONIC\","
        <<"\"enabled\":"<<(s.enabled?"true":"false")<<",\"execution_status\":"<<status
        <<",\"clock_failures\":"<<s.clock_failures<<",\"backward_samples\":"<<s.backward_samples
        <<",\"counter_saturated\":"<<(s.counter_saturated?"true":"false")
        <<",\"scope\":\"Initialized case Step calls only, including rejected calls. Slot zero is inclusive; other stage intervals are disjoint. "
          "Host monotonic wall time around existing calls, not kernel time. Queued CUDA work may be charged to a later readback. "
          "No added CUDA calls or synchronization. Excludes initialization, accepted output capture, archive I/O and this report. "
          "Counters are diagnostic and are not accepted physics epochs or energy observations.\",\"total\":";
    Counters(out,s,false);out<<",\"last_step\":";Counters(out,s,true);out<<"}\n";
    auto bytes=out.str();if(bytes.size()>ReportCap)throw std::length_error("Stage timing report exceeds fixed byte cap");
    return bytes;
}
void WriteStepTiming(const std::filesystem::path& path,const source_assembly_dynamics::StepTimingSnapshot& s,int status) {
    const auto bytes=FormatStepTiming(s,status);
    const int fd=open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600);
    if(fd<0)throw std::system_error(errno,std::generic_category(),"Cannot create stage timing report");
    std::size_t written=0;int error=0;
    while(written<bytes.size()) {
        const auto n=write(fd,bytes.data()+written,bytes.size()-written);
        if(n<0&&errno==EINTR)continue;
        if(n<=0) {error=n<0?errno:EIO;break;}written+=static_cast<std::size_t>(n);
    }
    if(close(fd)!=0&&!error)error=errno;
    if(error)throw std::system_error(error,std::generic_category(),"Stage timing report incomplete");
}
}
