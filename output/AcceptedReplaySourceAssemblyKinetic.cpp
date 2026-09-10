#include "AcceptedReplaySourceAssembly.h"

namespace crash::output::replay_detail {
namespace {
void CheckKineticPhase(const Bundle& b,const Entry& e,const Value& v,bool before) {
    const bool initial=before?e.epoch==1:e.epoch==0;const auto& p=Member(v,"phase");
    Require(Text(p,"kind")== (initial?"physical_initialization":"stored_midpoint_with_lagged_frame"),"Assembly kinetic observation phase changed");
    AssemblyEqual(Real(p,"position_time_s"),before?e.interval_base_time:e.time);
    AssemblyEqual(Real(p,"velocity_time_s"),initial?0:before?e.interval_base_velocity_time:e.interval_base_time+.5*b.fixed_dt);
    AssemblyEqual(Real(p,"frame_time_s"),initial?0:before?e.interval_previous_base_time:e.interval_base_time);
}
void CheckKinetic(const Bundle& b,const Entry& e,const Value& v,bool before,const Value* nodal) {
    CheckKineticPhase(b,e,v,before);
    const auto channels=CheckAssemblyKineticChannels(b,v);
    const auto& ordinary=channels.ordinary;const auto& members=channels.members;
    const double budget=Real(v,"publication_roundoff_budget_J");
    Require(budget>=0&&std::abs(Real(v,"publication_residual_J"))<=budget,"Native publication kinetic budget exceeded");
    if(!nodal)return;
    const auto& a=*b.assembly;const auto& velocity=Member(*nodal,"velocity_xyz_m_per_s");const auto& omega=Member(*nodal,"angular_velocity_xyz_rad_per_s");
    long double native[2][4]{};
    for(std::size_t n=0;n<a.native_nodes.size();++n) {
        long double vv=0,ww=0;
        for(unsigned j=0;j<3;++j) {const long double x=velocity[3*n+j].GetDouble(),w=omega[3*n+j].GetDouble();vv+=x*x;ww+=w*w;}
        auto& sum=native[a.grouped_node[n]?1:0];sum[0]+=.5L*a.native_nodes[n][0]*vv;
        for(unsigned j=1;j<4;++j)sum[j]+=.5L*a.native_nodes[n][j]*ww;
    }
    for(unsigned j=0;j<4;++j) {
        AssemblyNear(ordinary[j],native[0][j],a.native_nodes.size());
        AssemblyNear(members[j],native[1][j],a.member_count);
    }
    if(!e.epoch) {
        const double actual_native=Real(v,"native_total_J"),actual_aggregate=Real(v,"effective_total_J");
        Require(actual_native>=a.initial_native[1]&&actual_native<=a.initial_native[2]&&
            actual_aggregate>=a.initial_aggregate[1]&&actual_aggregate<=a.initial_aggregate[2],"Measured assembly initial kinetic lies outside its startup certificate");
    }
}
}
void CheckAssemblyMotion(const Bundle& b,const Entry& e,const Value& m,const Value* nodal) {
    CheckKinetic(b,e,Member(m,"after"),false,nodal);
    if(e.epoch)CheckKinetic(b,e,Member(m,"before"),true,nullptr);
    else Require(Member(m,"before").IsNull(),"Initial kinetic observation has an invented prior interval");
    Require(Text(m,"kick_work_columns")=="translation_J,rotation_J,total_J","Assembly kick-work columns changed");
    for(const char* key:{"applied_kick_work_J","reaction_kick_work_J"}) {
        const auto& row=WallNumbers(m,key,3);AssemblyReduction(row[2].GetDouble(),static_cast<long double>(row[0].GetDouble())+row[1].GetDouble(),
            std::abs(static_cast<long double>(row[0].GetDouble()))+std::abs(row[1].GetDouble()));
        if(!e.epoch)for(const auto& x:row.GetArray())Require(x.GetDouble()==0,"Initial kick work is not zero");
    }
    for(const char* key:{"native_delta_J","effective_delta_J","replacement_delta_J","native_residual_J","effective_residual_J","roundoff_budget_J"}) {
        const double x=Real(m,key);if(!e.epoch)Require(x==0,"Initial observation has interval bookkeeping");
    }
    const double budget=Real(m,"roundoff_budget_J");
    Require(budget>=0&&std::abs(Real(m,"native_residual_J"))<=budget&&std::abs(Real(m,"effective_residual_J"))<=budget,
        "Assembly native recurrence/effective bookkeeping budget exceeded");
    const double native=Real(m,"native_delta_J"),effective=Real(m,"effective_delta_J"),replacement=Real(m,"replacement_delta_J");
    const double applied=Member(m,"applied_kick_work_J")[2].GetDouble(),reaction=Member(m,"reaction_kick_work_J")[2].GetDouble();
    AssemblyEqual(Real(m,"native_residual_J"),(native-applied)-reaction);
    AssemblyEqual(Real(m,"effective_residual_J"),((effective-applied)-reaction)-replacement);
    // Stable deltas are serialized from actual kick arithmetic. Do not replace
    // them with subtraction of large endpoint energies or claim conservation.
    Require(Text(m,"reaction_work_semantics")=="Native constraint reaction kick work; recurrence consistency, not dissipation"&&
        Text(m,"energy_semantics")=="Stored midpoint with lagged-frame observation; effective residual is bookkeeping, not collocated or independent physical energy conservation",
        "Assembly reaction/energy interpretation changed");
}
} // namespace crash::output::replay_detail
