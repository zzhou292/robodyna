#include "AcceptedReplaySourceAssembly.h"

namespace crash::output::replay_detail {
namespace {
void CheckBoundary(const source::Data& s,const Value& b) {
    Require(Text(b,"policy")==s.boundary.policy&&Text(b,"interpretation")==s.boundary.interpretation&&
        Text(b,"unresolved_tied_scope")==s.boundary.unresolved_tied_scope,"Assembly released boundary policy changed");
    AssemblyIds(Member(b,"released_nodal_rigid_ids"),s.boundary.nodal_rigid_ids);
    AssemblyIds(Member(b,"released_spotweld_ids"),s.boundary.spotweld_ids);
    AssemblyIds(Member(b,"external_node_ids"),s.boundary.external_node_ids);
    AssemblyIds(Member(b,"external_part_ids"),s.boundary.external_part_ids);
    const auto& ties=WallArray(b,"released_tied_scopes",s.boundary.tied_scopes.size());
    for(std::size_t i=0;i<s.boundary.tied_scopes.size();++i) {
        const auto& t=s.boundary.tied_scopes[i];const auto& v=ties[i];
        Require(Text(v,"filename")==t.filename&&Unsigned(v,"source_line")==t.source_line&&Text(v,"reason")==t.reason,
            "Assembly released tied scope changed");
        WallBool(v,"pairing_qualified",false);AssemblyIds(Member(v,"selected_master_parts"),t.selected_master_parts);
        AssemblyIds(Member(v,"selected_slave_parts"),t.selected_slave_parts);
    }
}
}
void ReadAssemblyGroups(Bundle& b,const Value& input) {
    auto& a=*b.assembly;const auto& s=a.source.data();a.grouped_node.assign(s.nodes.size(),false);
    a.group_count=std::count_if(s.nodal_rigid_groups.begin(),s.nodal_rigid_groups.end(),[](const auto& g){return g.internal;});
    const auto& groups=WallArray(input,"internal_rigid_groups",a.group_count);std::size_t index=0;
    for(const auto& source:s.nodal_rigid_groups) {
        if(!source.internal)continue;
        const auto& g=groups[index++];
        Require(Unsigned(g,"source_group_id")==source.id&&Unsigned(g,"source_node_set_id")==source.node_set_id&&
            Unsigned(g,"member_count")==source.members.size(),"Assembly internal rigid group identity changed");
        Require(Text(g,"member_columns")=="source_node_id,global_node,reference_xyz_m,mass_kg,total_inertia_kg_m2,physical_inertia_kg_m2,added_inertia_kg_m2",
            "Assembly rigid member columns changed");
        const auto& members=WallArray(g,"members",source.members.size());std::array<long double,4> sums{};
        for(std::size_t m=0;m<source.members.size();++m) {
            const auto n=source.selected_global_nodes[m];const auto& row=AssemblyRow(members[m],7);
            Require(n<s.nodes.size()&&!a.grouped_node[n]&&AssemblyId(row[0])==source.members[m]&&AssemblyId(row[1])==n,
                "Assembly rigid member missing, duplicated or mapped to a foreign node");
            a.grouped_node[n]=true;++a.member_count;const auto& x=AssemblyNumbers(row[2],3);const auto p=s.nodes[n].position_m;
            const double expected[]={p.x,p.y,p.z};for(unsigned j=0;j<3;++j)AssemblyEqual(x[j].GetDouble(),expected[j]);
            for(unsigned j=0;j<4;++j) {const double value=AssemblyNumber(row[3+j]);AssemblyEqual(value,a.native_nodes[n][j]);sums[j]+=value;}
        }
        const char* keys[]={"structural_mass_kg","native_total_inertia_sum_kg_m2","physical_inertia_sum_kg_m2","added_inertia_sum_kg_m2"};
        for(unsigned j=0;j<4;++j)AssemblyNear(Real(g,keys[j]),sums[j],members.Size());
        for(const char* key:{"generated_primary_position_m","structural_center_m","center_m"})WallNumbers(g,key,3);
        const auto& raw=WallNumbers(g,"raw_tensor_kg_m2",9);const auto& effective=WallNumbers(g,"effective_tensor_kg_m2",9);
        const auto& axes=WallNumbers(g,"principal_axes_row_major",9);const auto& inertia=WallNumbers(g,"principal_inertia_kg_m2",3);
        const auto& raw_inertia=WallNumbers(g,"raw_principal_inertia_kg_m2",3);const auto& r=Member(g,"regularization");
        const double primary=Real(r,"primary_mass_kg"),primary_j=Real(r,"primary_isotropic_inertia_kg_m2");
        AssemblyEqual(primary,1e-20*s.units.mass_to_kg);
        AssemblyEqual(primary_j,1e-20*s.units.mass_to_kg*s.units.length_to_m*s.units.length_to_m);
        AssemblyNear(Real(g,"total_mass_kg"),sums[0]+primary,members.Size()+1);
        const auto& added=WallNumbers(r,"principal_inertia_added_kg_m2",3);const auto& tensor=WallNumbers(r,"tensor_added_kg_m2",9);
        Require(Member(r,"principal_threshold_reached").IsBool()&&Member(r,"principal_inertia_changed").IsBool(),"Invalid rigid regularization flags");
        bool changed=false;
        for(unsigned j=0;j<3;++j) {
            Require(inertia[j].GetDouble()>0&&raw_inertia[j].GetDouble()>0&&added[j].GetDouble()>=0,"Invalid principal inertia ledger");
            AssemblyNear(inertia[j].GetDouble(),static_cast<long double>(raw_inertia[j].GetDouble())+added[j].GetDouble());
            changed|=added[j].GetDouble()!=0;
            for(unsigned k=0;k<3;++k) {
                long double dot=0;for(unsigned l=0;l<3;++l)dot+=static_cast<long double>(axes[3*l+j].GetDouble())*axes[3*l+k].GetDouble();
                // Orthogonal components near zero need an absolute unit-frame budget.
                Require(std::abs(dot-(j==k?1.L:0.L))<=1e-12L,"Invalid rigid principal frame");
                AssemblyNear(effective[3*j+k].GetDouble(),static_cast<long double>(raw[3*j+k].GetDouble())+tensor[3*j+k].GetDouble());
            }
        }
        WallBool(r,"principal_inertia_changed",changed);
    }
    Require(a.group_count==6&&a.member_count==76,"Pinned assembly internal group coverage changed");
    CheckBoundary(s,Member(input,"boundary"));
}
} // namespace crash::output::replay_detail
