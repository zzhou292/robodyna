#include "AcceptedReplaySourceAssembly.h"

namespace crash::output::replay_detail {
void CheckAssemblyMotion(const Bundle&,const Entry&,const Value&,const Value*);
namespace {
void CheckFamily(const Bundle& b,const Entry& e,const Value& v) {
    Require(Text(v,"phase")=="accepted"&&Text(v,"usage")=="coupled_forces"&&Unsigned(v,"owner_id")==e.owner&&
        Unsigned(v,"configuration_id")==b.source_configuration_id&&Unsigned(v,"qualification_id")==b.qualification_id&&
        Unsigned(v,"epoch")==e.epoch&&Unsigned(v,"base_epoch")== (e.epoch?e.epoch-1:0)&&
        Unsigned(v,"attempt")==e.interval_attempt,"Assembly shell publication association changed");
    WallBool(v,"valid",true);WallBool(v,"has_completed_interval",e.epoch!=0);WallBool(v,"kinetic_available",false);
    WallBool(v,"accepted_force_assembled",e.epoch!=0);
    AssemblyEqual(Real(v,"time_s"),e.time);AssemblyEqual(Real(v,"base_time_s"),e.interval_base_time);
    AssemblyEqual(Real(v,"velocity_time_s"),e.epoch?e.interval_base_time+.5*b.fixed_dt:0);
    AssemblyEqual(Real(v,"base_velocity_time_s"),e.interval_base_velocity_time);
    AssemblyEqual(Real(v,"kick_dt_s"),e.epoch?(e.epoch==1?.5*b.fixed_dt:b.fixed_dt):0);
    WallNumbers(v,"native_internal_work_J",2);WallNumbers(v,"native_internal_work_increment_J",2);
    Real(v,"internal_kick_work_J");Real(v,"internal_drift_work_J");
    for(const char* key:{"minimum_area_ratio","minimum_thickness_ratio"})Require(Real(v,key)>0,"Invalid native shell minimum");
    Require(e.epoch?Real(v,"minimum_native_dt_s")>0:Real(v,"minimum_native_dt_s")==0,"Native timestep diagnostic does not match interval phase");
    for(const char* key:{"maximum_displacement_m","maximum_absolute_strain","maximum_thickness_curvature"})Require(Real(v,key)>=0,"Invalid native shell maximum");
    Require(Real(v,"maximum_displacement_m")<=b.assembly->maximum_displacement&&
        Real(v,"minimum_thickness_ratio")>=b.assembly->minimum_thickness_ratio,"Native shell geometry exceeded recorded envelope");
}
}
void CheckAssemblyDiagnostics(const Bundle& b,const Entry& e,const Value& d,const Value* nodal) {
    const auto& a=*b.assembly;WallBool(d,"has_interval",e.epoch!=0);
    for(const char* key:{"maximum_rotation_rad","maximum_area_ratio","maximum_thickness_ratio","maximum_plastic_strain","cumulative_plastic_work_J"})
        Require(Real(d,key)>=0,"Invalid assembly diagnostic magnitude");
    CheckAssemblyRotation(b,e,d,nodal);
    Require(Real(d,"maximum_area_ratio")<=a.maximum_area_ratio&&
        Real(d,"maximum_thickness_ratio")<=a.maximum_thickness_ratio&&Unsigned(d,"yielded_points")<=3*a.source.data().parents.size()&&
        Unsigned(d,"yielded_parents")<=a.source.data().parents.size()&&Unsigned(d,"active_contact_nodes")<=b.info.node_count,
        "Assembly diagnostics exceed source/envelope counts");
    const auto first=Unsigned(d,"first_contact_epoch"),last=Unsigned(d,"last_contact_epoch"),intervals=Unsigned(d,"contact_intervals");
    Require(intervals<=e.epoch&&(intervals?first>0&&first<=last&&last<=e.epoch:!first&&!last),"Assembly accepted contact history changed");
    Require(e.interval_contact_history==std::array<std::uint64_t,3>{first,last,intervals},"Assembly contact counters differ from complete accepted ledger");
    Require(!Unsigned(d,"active_contact_nodes")||(last==e.epoch&&intervals),"Active assembly contact lacks its accepted interval");
    const auto& shells=Member(d,"shells");
    Require(Text(shells,"native_kinetic_columns")=="translation_J,rotation_J,physical_isotropic_J,added_isotropic_J","Native shell kinetic columns changed");
    for(const char* key:{"base_native_kinetic_J","native_kinetic_J"})
        for(const auto& x:WallNumbers(shells,key,4).GetArray())Require(x.GetDouble()>=0,"Negative native shell kinetic magnitude");
    if(a.connectors.empty()) {
        Require(!shells.HasMember("connector_kinetic_columns")&&!shells.HasMember("base_connector_kinetic_J")&&!shells.HasMember("connector_kinetic_J"),
            "Undeclared connector kinetic diagnostics");
    } else {
        Require(Text(shells,"connector_kinetic_columns")=="translation_J,rotation_J","Connector kinetic subtotal semantics changed");
        const auto& motion=Member(d,"motion");
        const auto after=AssemblyConnectorKinetic(b,Member(motion,"after"));
        const auto& current=WallNumbers(shells,"connector_kinetic_J",2);
        // Publication uses binary64 endpoint arithmetic; the independent host
        // observation reduces in long double. These derived subtotals are not
        // copied identities. Bound agreement by their own positive magnitude
        // and complete endpoint count, with no absolute floor for tiny values.
        const auto agreement=[&](double raw,double observed) {
            Require(raw>=0,"Negative connector publication kinetic subtotal");
            AssemblyReduction(raw,observed,std::max(raw,observed),2*a.connectors.size());
        };
        for(unsigned j=0;j<2;++j)agreement(current[j].GetDouble(),after[j]);
        const auto base=e.epoch?AssemblyConnectorKinetic(b,Member(motion,"before")):std::array<double,2>{};
        const auto& raw_base=WallNumbers(shells,"base_connector_kinetic_J",2);
        for(unsigned j=0;j<2;++j) {
            if(e.epoch)agreement(raw_base[j].GetDouble(),base[j]);
            else AssemblyEqual(raw_base[j].GetDouble(),0); // No completed initial interval.
        }
    }
    const auto& q=Member(shells,"qeph");const auto& t=Member(shells,"t3");CheckFamily(b,e,q);CheckFamily(b,e,t);
    Real(q,"hourglass_viscous_work_J");Real(q,"hourglass_viscous_work_increment_J"); // Signed native work, not a dissipation ledger.
    long double work=Real(q,"hourglass_viscous_work_J"),magnitude=std::abs(work);
    for(const auto* family:{&q,&t})for(const auto& x:Member(*family,"native_internal_work_J").GetArray()){work+=x.GetDouble();magnitude+=std::abs(x.GetDouble());}
    AssemblyReduction(Real(d,"native_internal_work_J"),work,magnitude,5);
    const auto& motion=Member(d,"motion");CheckAssemblyMotion(b,e,motion,nodal);
    const auto& k=Member(shells,"native_kinetic_J");
    AssemblyNear(Real(Member(motion,"after"),"native_total_J"),static_cast<long double>(k[0].GetDouble())+k[1].GetDouble(),b.info.node_count);
    if(!e.epoch) {
        Require(!intervals&&!Unsigned(d,"active_contact_nodes")&&!Unsigned(d,"yielded_points")&&!Unsigned(d,"yielded_parents")&&
            Real(d,"maximum_plastic_strain")==0&&Real(d,"cumulative_plastic_work_J")==0,"Initial assembly diagnostics contain interval history");return;
    }
    const auto it=std::find_if(b.entries.begin(),b.entries.end(),[&](const auto& x){return x.epoch==e.epoch;});
    Require(it!=b.entries.end(),"Assembly diagnostic epoch is not indexed");const auto& values=a.intervals[std::size_t(it-b.entries.begin())];
    const auto& after=Member(motion,"after");
    const double current[]={Real(after,"native_total_J"),Real(after,"effective_total_J"),Real(d,"native_internal_work_J"),
        Real(d,"cumulative_plastic_work_J"),Real(motion,"native_delta_J"),Real(motion,"effective_delta_J"),Real(motion,"replacement_delta_J"),
        Member(motion,"applied_kick_work_J")[2].GetDouble(),Member(motion,"reaction_kick_work_J")[2].GetDouble(),
        Real(motion,"native_residual_J"),Real(motion,"effective_residual_J"),Real(motion,"roundoff_budget_J")};
    for(unsigned j=0;j<12;++j)AssemblyEqual(current[j],values[j+2]);
    const double final[]={double(Unsigned(d,"active_contact_nodes")),Real(d,"maximum_plastic_strain"),double(Unsigned(d,"yielded_points")),
        double(Unsigned(d,"yielded_parents")),Real(d,"maximum_rotation_rad"),Real(d,"maximum_area_ratio"),Real(d,"maximum_thickness_ratio")};
    for(unsigned j=0;j<7;++j)AssemblyEqual(final[j],values[j+26]);
}
} // namespace crash::output::replay_detail
