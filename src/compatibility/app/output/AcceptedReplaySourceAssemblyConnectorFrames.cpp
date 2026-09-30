#include "AcceptedReplaySourceAssemblyConnectorValues.h"
namespace crash::output::replay_detail {
namespace {
void Phase(const Bundle& b,const Entry& e,const Value& d) {
    const auto& a=*b.assembly;
    Require(d.IsObject()&&d.MemberCount()==24&&Text(d,"phase")=="accepted"&&Unsigned(d,"source_instance_id")==a.instance&&Unsigned(d,"owner_id")==e.owner&&
        Unsigned(d,"configuration_id")==b.source_configuration_id&&Unsigned(d,"qualification_id")==b.qualification_id&&
        Unsigned(d,"epoch")==e.epoch&&Unsigned(d,"base_epoch")== (e.epoch?e.epoch-1:0)&&
        Unsigned(d,"attempt")==e.interval_attempt&&Unsigned(d,"element_count")==a.connectors.size(),
        "Connector accepted source/owner/interval association changed");
    WallBool(d,"valid",true);WallBool(d,"has_completed_interval",e.epoch!=0);WallBool(d,"accepted_force_assembled",e.epoch!=0);
    AssemblyEqual(Real(d,"time_s"),e.time);AssemblyEqual(Real(d,"base_time_s"),e.interval_base_time);
    AssemblyEqual(Real(d,"velocity_time_s"),e.epoch?e.interval_base_time+.5*b.fixed_dt:0);
    AssemblyEqual(Real(d,"base_velocity_time_s"),e.interval_base_velocity_time);
    AssemblyEqual(Real(d,"kick_dt_s"),e.epoch?(e.epoch==1?.5*b.fixed_dt:b.fixed_dt):0);
    const auto active=Unsigned(d,"active_count"),failed=Unsigned(d,"newly_failed_count");
    Require(active<=a.connectors.size()&&failed<=a.connectors.size()-active,"Connector failure counts exceed scope");
    WallNumbers(d,"internal_work_J",4);WallNumbers(d,"internal_work_increment_J",4);
    Real(d,"internal_kick_work_J");Real(d,"internal_drift_work_J");
    if(!e.epoch) {
        Require(active==a.connectors.size()&&!failed&&Real(d,"internal_kick_work_J")==0&&Real(d,"internal_drift_work_J")==0,
            "Initial connector diagnostics contain an executed interval");
        for(const char* key:{"internal_work_J","internal_work_increment_J"})
            for(const auto& x:Member(d,key).GetArray())Require(x.GetDouble()==0,"Initial connector work is not zero");
    }
}
void Previous(const Bundle& b,const Entry& e,const Value& record) {
    const auto it=std::find_if(b.entries.begin(),b.entries.end(),[&](const auto& entry){return entry.epoch==e.epoch;});
    Require(it!=b.entries.end(),"Connector frame is not indexed");if(it==b.entries.begin())return;
    const auto& previous=*(it-1);const auto old=Json(VerifiedBytes(b,previous.mesh.substr(0,previous.mesh.size()-10)+".fields.json"));
    const auto& prior=Member(old,"connectors");const auto count=b.assembly->connectors.size();
    const auto& before=WallArray(prior,"elements",count);const auto& after=WallArray(record,"elements",count);
    std::size_t newly=0;std::array<long double,4> increments{},magnitude{};
    for(std::size_t i=0;i<count;++i) {
        const auto& p=Member(before[i],"history");const auto& n=Member(after[i],"history");
        const auto& prior_active=Member(p,"active");Require(prior_active.IsBool(),"Prior connector active state has invalid type");
        const bool active=Member(n,"active").GetBool();
        if(!prior_active.GetBool()) {
            Require(!active,"Accepted connector was resurrected after failure");
            for(const char* key:{"local_force_N","local_couple_N_m"})for(const auto& x:Member(n,key).GetArray())
                Require(x.GetDouble()==0,"A subsequent inactive connector evaluation retained a force cache");
        }
        newly+=prior_active.GetBool()&&!active;
        const auto& pw=WallNumbers(p,"internal_work_J",4);const auto& nw=Member(n,"internal_work_J");
        for(unsigned j=0;j<4;++j){increments[j]+=static_cast<long double>(nw[j].GetDouble())-pw[j].GetDouble();
            magnitude[j]+=std::abs(nw[j].GetDouble())+std::abs(pw[j].GetDouble());}
    }
    Require(Unsigned(Member(record,"diagnostics"),"newly_failed_count")<=newly,
        "A connector already inactive at the previous sample cannot fail again");
    if(previous.epoch+1==e.epoch) {
        Require(Unsigned(Member(record,"diagnostics"),"newly_failed_count")==newly,"Connector interval failure count changed");
        const auto& increment=Member(Member(record,"diagnostics"),"internal_work_increment_J");
        for(unsigned j=0;j<4;++j)AssemblyReduction(increment[j].GetDouble(),increments[j],magnitude[j],count);
    }
    // Sparse increments are the final completed interval only. Do not equate
    // them with the cumulative work difference across an unsaved interval span.
}
}
void CheckAssemblyConnectors(const Bundle& b,const Entry& e,const Value& frame) {
    if(b.assembly->connectors.empty()) {Require(!frame.HasMember("connectors"),"Six-part frame has undeclared connectors");return;}
    const auto& record=Member(frame,"connectors");Require(record.IsObject()&&record.MemberCount()==3&&Text(record,"kind")==AssemblyConnectorKind,"Unknown accepted connector kind");
    Phase(b,e,Member(record,"diagnostics"));CheckAssemblyConnectorElements(b,e,record,Member(frame,"nodal_fields"));
    Previous(b,e,record);
}
}
