#include "QephSpinTrace.h"
#include "case/source_assembly_dynamics/Case.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <cerrno>
#include <fcntl.h>
#include <system_error>
#include <unistd.h>

namespace crash::output::assembly {
namespace dynamics=cases::source_assembly_dynamics;
struct QephSpinTrace::Impl {
    int fd=-1;std::uint64_t requested=0,cadence=0,seen=0,saved=0,records=0,source_node=0;
    std::size_t bytes=0,forecast=0;bool finished=false,poisoned=false;
    tl::fea::NodalStamp initial;
    ~Impl(){if(fd>=0)close(fd);}
    void Write(const Document& d) {
        Require(fd>=0&&!finished&&!poisoned,"Spin trace is closed or incomplete");
        try {
            rapidjson::StringBuffer buffer;rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
            Require(d.Accept(writer),"Spin trace serialization failed");const auto size=buffer.GetSize();
            Require(size<SpinTraceRowCap&&bytes<=forecast-(size+1),"Spin trace row or total forecast exceeded");
            const std::string line=std::string(buffer.GetString(),size)+'\n';std::size_t offset=0;
            while(offset<line.size()) {
                const auto count=write(fd,line.data()+offset,line.size()-offset);
                if(count<0&&errno==EINTR)continue;
                if(count<=0)throw std::system_error(count<0?errno:EIO,std::generic_category(),"Spin trace write failed");
                offset+=static_cast<std::size_t>(count);
            }
            bytes+=line.size();
        }catch(...){poisoned=true;throw;}
    }
    const cases::source_assembly_observation::QephSpinObservation& Check(const dynamics::SourceAssemblyWallCase& run) {
        Require(!finished&&!poisoned&&run.owner()&&run.bindings(),"Spin trace has no live case");
        const auto* record=run.accepted_qeph_spin();const auto current=run.owner()->accepted();
        Require(record&&record->completed&&record->source_node==source_node&&current.owner_id==initial.owner_id&&
            current.node_count==initial.node_count&&current.fixed_dt==initial.fixed_dt&&
            tl::fea::SameRigidGroupInfo(current.rigid_groups,initial.rigid_groups)&&
            tl::fea::trial_identity::SameStamp(current,record->enclosing),"Spin trace does not identify the complete accepted record");
        return *record;
    }
    void Save(const cases::source_assembly_observation::QephSpinObservation& record) {
        Write(QephSpinDocument(record));saved=record.enclosing.epoch;++records;
    }
};
QephSpinTrace::QephSpinTrace(const std::filesystem::path& path,const dynamics::SourceAssemblyWallCase& run,
                           std::uint64_t steps,std::uint64_t cadence):impl_(std::make_unique<Impl>()) {
    auto& s=*impl_;s.forecast=PlanQephSpinTrace(steps,cadence);
    Require(run.initialized()&&run.config()&&run.config()->observe_qeph_spin_node&&run.owner()->accepted().epoch==0,
            "Spin trace must begin with enabled source probe at accepted initialization");
    s.initial=run.owner()->accepted();s.requested=steps;s.cadence=cadence;s.source_node=run.config()->observe_qeph_spin_node;
    Document d;d.SetObject();String(d,"schema","robo_dyna.source_assembly_qeph_spin_trace.v1");String(d,"record","header");
    const auto& source=run.bindings()->source().data();String(d,"source_inventory_sha256",source.identity.sha256);
    Integer(d,"source_inventory_bytes",source.identity.bytes);Integer(d,"source_instance_id",run.bindings()->source_instance_id());
    Integer(d,"configuration_id",run.setup()->settings()->configuration_id);
    Integer(d,"qualification_id",run.setup()->settings()->qualification_id);
    Integer(d,"wall_binding_id",run.setup()->settings()->wall_binding_id);
    Integer(d,"source_node_id",s.source_node);Integer(d,"owner_id",s.initial.owner_id);Number(d,"fixed_dt_s",s.initial.fixed_dt);
    Integer(d,"requested_steps",steps);Integer(d,"cadence",cadence);Integer(d,"forecast_bytes",s.forecast);
    Integer(d,"row_byte_cap",SpinTraceRowCap);Integer(d,"total_byte_cap",SpinTraceByteCap);
    String(d,"scope","Actual retained native force/history packets and assembled loads at base force time; published only after enclosing commit. "
        "Paired enclosing candidate motion/packets retain exact native one-step replay inputs and results at sparse cadence. "
        "Complete incident QEPH coverage for one ordinary node; zero T3 incidence. Native corotational point stress; HOURG/STRA/rate mixed units "
        "follow their owning native contracts. Carried-power decomposition is not collocated work, dissipation or an energy acceptance certificate. "
        "No force recomputation or sparse spin differentiation. Missing completion footer means incomplete trace.");
    s.fd=open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600);
    if(s.fd<0)throw std::system_error(errno,std::generic_category(),"Cannot create spin trace");
    s.Write(d);
}
QephSpinTrace::~QephSpinTrace()=default;
void QephSpinTrace::RecordInterval(const dynamics::SourceAssemblyWallCase& run) {
    auto& s=*impl_;
    try {
        const auto& record=s.Check(run);
        Require(s.seen<s.requested&&record.enclosing.epoch==s.seen+1&&record.base.epoch==s.seen,
                "Spin trace interval is duplicated, skipped or exceeds the request");
        if(record.enclosing.epoch==1||record.enclosing.epoch%s.cadence==0)s.Save(record);
        ++s.seen;
    }catch(...){s.poisoned=true;throw;}
}
void QephSpinTrace::Finish(const dynamics::SourceAssemblyWallCase& run,bool complete,const std::string& reason) {
    auto& s=*impl_;
    try {
        Require(!s.finished&&!s.poisoned&&run.owner()&&run.bindings(),"Spin trace cannot finish a closed/incomplete case");
        const auto current=run.owner()->accepted();
        Require(current.epoch==s.seen&&complete==(s.seen==s.requested)&&
                (complete?reason.empty():!reason.empty()),"Spin trace completion does not match its actual accepted prefix");
        double force_time=0;
        if(s.seen) {
            const auto& record=s.Check(run);force_time=record.base.time;
            if(s.saved!=s.seen)s.Save(record);
        } else Require(tl::fea::trial_identity::SameStamp(current,s.initial)&&!run.accepted_qeph_spin(),
                       "Empty prefix must retain the exact initial state");
        Document d;d.SetObject();String(d,"record","completion");Boolean(d,"horizon_complete",complete);String(d,"stop_reason",reason);
        Integer(d,"accepted_epoch",s.seen);Number(d,"accepted_time_s",current.time);Integer(d,"saved_force_stages",s.records);
        Boolean(d,"has_force_stage",s.seen!=0);Number(d,"last_observed_force_time_s",force_time);
        Integer(d,"trace_bytes_before_footer",s.bytes);s.Write(d);
        const int fd=s.fd;s.fd=-1;if(close(fd))throw std::system_error(errno,std::generic_category(),"Spin trace close failed");s.finished=true;
    }catch(...){s.poisoned=true;throw;}
}
}
