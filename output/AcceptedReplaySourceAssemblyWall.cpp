#include "AcceptedReplaySourceAssembly.h"
#include <numeric>

namespace crash::output::replay_detail {
void ReadAssemblyWallSetup(Bundle& b,const Value& v) {
    auto& a=*b.assembly;const auto& s=a.source.data();
    Require(Text(v,"contact_model")=="reference-area-lumped-nodal-wall-v1"&&Text(v,"boundary_policy")==s.boundary.policy&&
        Text(v,"certificate_columns")=="value,lower,upper,error"&&Unsigned(v,"configuration_id")==b.source_configuration_id&&
        Unsigned(v,"qualification_id")==b.qualification_id&&Unsigned(v,"wall_binding_id")==b.wall_binding_id,
        "Assembly wall setup association changed");
    const auto& velocity=WallNumbers(v,"initial_velocity_xyz_m_per_s",3);
    for(unsigned j=0;j<3;++j)b.source_initial_velocity[j]=velocity[j].GetDouble();
    Require(b.source_initial_velocity[0]>0&&b.source_initial_velocity[1]==0&&b.source_initial_velocity[2]==0,"Assembly startup velocity policy changed");
    for(const char* key:{"declared_leading_gap_m","area_floor_m2","design_penetration_m","penetration_cap_m","kinetic_budget_factor",
        "kinetic_budget_upper_J","stiffness_per_area_N_m3","design_potential_lower_J","minimum_nodal_area_lower_m2",
        "motion_margin_m","exposed_clearance_m","parent_force_error_N","parent_energy_error_J","maximum_contact_step_rate"})
        Require(Real(v,key)>0,"Invalid assembly wall setup positive declaration");
    b.wall_penetration_cap=Real(v,"penetration_cap_m");
    Require(Real(v,"design_penetration_m")<=b.wall_penetration_cap&&Real(v,"minimum_nodal_area_lower_m2")>=Real(v,"area_floor_m2")&&
        Real(v,"design_potential_lower_J")>=Real(v,"kinetic_budget_upper_J")&&Unsigned(v,"minimum_area_node")<s.nodes.size(),
        "Assembly penalty certificate bounds changed");
    Unsigned(v,"kappa_upward_steps");WallBool(v,"projected_motion_covered",true);
    for(const char* key:{"query_lower_expansion_upper_m","query_upper_expansion_upper_m"})
        for(const auto& x:WallNumbers(v,key,3).GetArray())Require(x.GetDouble()>=0,"Invalid wall query expansion");
    a.initial_native=WallCertificate(Member(v,"native_initial_kinetic_J"));
    a.initial_aggregate=WallCertificate(Member(v,"aggregate_initial_kinetic_J"));
    WallCertificate(Member(v,"generated_primary_mass_kg"));
    Require(Unsigned(v,"ordinary_nodes")==s.nodes.size()-a.member_count&&Unsigned(v,"member_nodes")==a.member_count&&
        Unsigned(v,"generated_primaries")==a.group_count,"Assembly wall initial mass partition changed");
    const auto& groups=WallArray(v,"group_initial_kinetic",a.group_count);std::size_t i=0;
    for(const auto& g:s.nodal_rigid_groups)if(g.internal) {
        const auto& row=AssemblyRow(groups[i++],6);
        Require(AssemblyId(row[0])==g.id&&AssemblyId(row[1])==g.node_set_id&&AssemblyId(row[2])==g.members.size(),
            "Assembly wall initial group identity changed");
        WallCertificate(row[3]);WallCertificate(row[4]);Require(AssemblyNumber(row[5])>0,"Missing generated primary mass channel");
    }
    const auto& nodes=WallArray(v,"nodal_area_m2",s.nodes.size());
    for(const auto& row:nodes.GetArray())Require(WallCertificate(row)[1]>=Real(v,"area_floor_m2"),"Assembly contact area below declared floor");
    Require(Text(v,"contact_parent_columns")=="weight_index,source_parent_index,source_element_id,arity,area_m2,share_m2",
        "Assembly contact area mapping columns changed");
    const auto& parents=WallArray(v,"contact_parents",s.parents.size());
    a.contact_source_parent.resize(s.parents.size());std::iota(a.contact_source_parent.begin(),a.contact_source_parent.end(),0);
    std::sort(a.contact_source_parent.begin(),a.contact_source_parent.end(),[&](auto x,auto y){return s.parents[x].source_id<s.parents[y].source_id;});
    for(std::size_t e=0;e<s.parents.size();++e) {
        const auto& row=AssemblyRow(parents[e],6);const auto& p=s.parents[a.contact_source_parent[e]];
        Require(AssemblyId(row[0])==e&&AssemblyId(row[1])==p.index&&AssemblyId(row[2])==p.source_id&&AssemblyId(row[3])==p.arity,
            "Assembly contact parent order/identity changed");
        const auto area=WallCertificate(row[4]),share=WallCertificate(row[5]);
        Require(area[1]>0&&share[1]>0,"Assembly contact parent area is not positive");
        AssemblyNear(area[0],static_cast<long double>(share[0])*p.arity);
    }
    Require(Text(v,"wall_placement_file")=="placed-wall-placement.json"&&Text(v,"wall_mesh_file")=="placed-wall.mesh.json"&&
        Text(v,"original_wall_manifest_file")=="original-canonical-wall.manifest.json","Assembly wall file names changed");
    std::array<double,3> minimum{},maximum{};const auto p=s.nodes.front().position_m;
    minimum=maximum={p.x,p.y,p.z};
    for(const auto& node:s.nodes) {
        const double x[]={node.position_m.x,node.position_m.y,node.position_m.z};
        for(unsigned j=0;j<3;++j){minimum[j]=std::min(minimum[j],x[j]);maximum[j]=std::max(maximum[j],x[j]);}
    }
    CheckPlacedWall(b,v,minimum,maximum);
}
} // namespace crash::output::replay_detail
