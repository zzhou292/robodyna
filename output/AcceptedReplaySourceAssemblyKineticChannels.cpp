#include "AcceptedReplaySourceAssembly.h"
#include "source_assembly/SourceAssemblyKineticSchema.h"

namespace crash::output::replay_detail {
namespace {
std::array<double,6> MemberChannels(const Value& v,const char* key) {
    const auto& row=WallNumbers(v,key,6);std::array<double,6> x{};
    for(unsigned j=0;j<6;++j){x[j]=row[j].GetDouble();Require(j==5||x[j]>=0,"Invalid native kinetic magnitude");}
    AssemblyNear(x[4],static_cast<long double>(x[0])+x[1]);
    AssemblyReduction(x[5],static_cast<long double>(x[1])-x[2]-x[3],static_cast<long double>(x[1])+x[2]+x[3],256);
    return x;
}
}
AssemblyKineticChannels CheckAssemblyKineticChannels(const Bundle& b,const Value& v) {
    Require(Text(v,"member_columns")==assembly::KineticMemberColumns&&Text(v,"aggregate_columns")==assembly::KineticAggregateColumns,
        "Assembly kinetic channel semantics changed");
    AssemblyKineticChannels next;next.ordinary=MemberChannels(v,"ordinary_native_nodes");
    next.members=MemberChannels(v,"grouped_native_members");auto& group=next.groups;
    const auto& values=WallNumbers(v,"aggregate_groups",14);
    for(unsigned j=0;j<14;++j) {group[j]=values[j].GetDouble();Require(j==12||group[j]>=0,"Invalid aggregate kinetic magnitude");}
    AssemblyNear(group[2],static_cast<long double>(group[0])+group[1]);
    AssemblyNear(group[0],static_cast<long double>(group[3])+group[4],b.assembly->group_count);
    long double decomposition=0;for(unsigned j:{5u,6u,9u,10u,11u})decomposition+=group[j];
    AssemblyReduction(group[12],static_cast<long double>(group[1])-decomposition,group[1]+decomposition,b.assembly->group_count);
    AssemblyNear(group[6],static_cast<long double>(group[7])+group[8],b.assembly->member_count);
    Require(std::abs(group[12])<=group[13],"Aggregate kinetic decomposition budget exceeded");
    const double native=Real(v,"native_total_J"),effective=Real(v,"effective_total_J");
    Require(native>=0&&effective>=0,"Negative assembly kinetic total");
    AssemblyNear(native,static_cast<long double>(next.ordinary[4])+next.members[4]);
    AssemblyNear(effective,static_cast<long double>(next.ordinary[4])+group[2]);return next;
}
} // namespace crash::output::replay_detail
