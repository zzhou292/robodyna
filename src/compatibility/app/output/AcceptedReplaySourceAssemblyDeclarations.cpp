#include "AcceptedReplaySourceAssembly.h"

namespace crash::output::replay_detail {
void CheckAssemblySurface(const Bundle& b,const Value& value) {
    const auto& a=*b.assembly;const auto& s=a.source.data();
    Require(Text(value,"vertex_binding_columns")=="tl_node,asset,instance,source_node"&&
        Text(value,"triangle_binding_columns")=="v0,v1,v2,asset,instance,parent_element,part,local_face,subtriangle",
        "Assembly surface column semantics changed");
    const auto& vertices=WallArray(value,"vertex_binding",s.nodes.size());
    for(std::size_t n=0;n<s.nodes.size();++n) {
        const auto& row=AssemblyRow(vertices[n],4);
        const std::uint64_t expected[]={n,a.asset,a.instance,s.nodes[n].source_id};
        for(unsigned j=0;j<4;++j)Require(AssemblyId(row[j])==expected[j],"Assembly vertex source mapping changed");
    }
    const auto& triangles=WallArray(value,"triangle_binding",2*s.qeph_count+s.t3_count);
    std::size_t triangle=0;
    for(const auto& p:s.parents)for(unsigned sub=0;sub<(p.arity==4?2u:1u);++sub) {
        const auto& row=AssemblyRow(triangles[triangle++],9);
        const std::uint64_t expected[]={p.nodes[0],p.nodes[sub?2:1],p.nodes[sub?3:2],a.asset,a.instance,p.source_id,p.part_id,0,sub};
        for(unsigned j=0;j<9;++j)Require(AssemblyId(row[j])==expected[j],"Assembly triangle source mapping changed");
    }
}
void ReadAssemblyDeclarations(Bundle& b,const Value& input) {
    auto& a=*b.assembly;const auto& s=a.source.data();
    Require(Text(input,"inventory_file")=="source-assembly-inventory.json"&&Text(input,"inventory_schema")==s.schema&&
        Text(input,"inventory_sha256")==s.identity.sha256&&Unsigned(input,"inventory_bytes")==s.identity.bytes&&
        Unsigned(input,"node_count")==s.nodes.size()&&Unsigned(input,"parent_count")==s.parents.size()&&
        Unsigned(input,"qeph_count")==s.qeph_count&&Unsigned(input,"t3_count")==s.t3_count&&
        Text(input,"material_rate_policy")=="openradioss_direct_import_default","Assembly input identity or policy changed");
    const auto& units=WallNumbers(input,"source_units_to_SI",3);
    AssemblyEqual(units[0].GetDouble(),s.units.mass_to_kg);AssemblyEqual(units[1].GetDouble(),s.units.length_to_m);
    AssemblyEqual(units[2].GetDouble(),s.units.time_to_s);
    Require(Text(input,"part_columns")=="source_part_id,source_material_id,source_section_id,first_parent,parent_count"&&
        Text(input,"material_columns")=="source_material_id,source_curve_id,density_kg_m3,young_Pa,poisson_ratio,source_rate_C_per_s,source_rate_P,source_VP,resolved_rate_enabled,resolved_cutoff_Hz"&&
        Text(input,"section_columns")=="source_section_id,source_elform,through_thickness_points,source_thickness_m"&&
        Text(input,"curve_columns")=="source_curve_id,equivalent_plastic_strain,true_stress_Pa","Assembly declaration columns changed");
    const auto& parts=WallArray(input,"parts",s.parts.size());
    for(std::size_t i=0;i<s.parts.size();++i) {
        const auto& p=s.parts[i];const auto& row=AssemblyRow(parts[i],5);
        const std::uint64_t ids[]={p.id,p.material_id,p.section_id,p.first_parent,p.parent_count};
        for(unsigned j=0;j<5;++j)Require(AssemblyId(row[j])==ids[j],"Assembly complete part mapping changed");
    }
    const auto& materials=WallArray(input,"materials",s.materials.size());
    for(std::size_t i=0;i<s.materials.size();++i) {
        const auto& m=s.materials[i];const auto& row=AssemblyRow(materials[i],10);
        Require(AssemblyId(row[0])==m.id&&AssemblyId(row[1])==m.curve_id&&AssemblyId(row[7])==m.source_rate_type&&
            row[8].IsBool()&&row[8].GetBool(),"Assembly source material or resolved rate policy changed");
        const double values[]={m.density_kg_m3,m.young_pa,m.poisson_ratio,m.rate_c_per_s,m.rate_p};
        for(unsigned j=0;j<5;++j)AssemblyEqual(AssemblyNumber(row[2+j]),values[j]);
        AssemblyEqual(AssemblyNumber(row[9]),10000.);
    }
    const auto& sections=WallArray(input,"sections",s.sections.size());
    for(std::size_t i=0;i<s.sections.size();++i) {
        const auto& x=s.sections[i];const auto& row=AssemblyRow(sections[i],4);
        Require(AssemblyId(row[0])==x.id&&AssemblyId(row[1])==x.source_elform&&AssemblyId(row[2])==x.through_thickness_points,
            "Assembly original shell section mapping changed");
        const auto& thickness=AssemblyNumbers(row[3],4);
        for(unsigned j=0;j<4;++j)AssemblyEqual(thickness[j].GetDouble(),x.thickness_m[j]);
    }
    const auto& curves=WallArray(input,"curves",s.curves.size());
    for(std::size_t i=0;i<s.curves.size();++i) {
        const auto& c=s.curves[i];const auto& row=AssemblyRow(curves[i],3);
        Require(AssemblyId(row[0])==c.id,"Assembly source curve ID changed");
        const auto& x=AssemblyNumbers(row[1],c.plastic_strain.size());const auto& y=AssemblyNumbers(row[2],c.stress_pa.size());
        for(std::size_t j=0;j<c.plastic_strain.size();++j) {
            AssemblyEqual(x[j].GetDouble(),c.plastic_strain[j]);AssemblyEqual(y[j].GetDouble(),c.stress_pa[j]);
        }
    }
    Require(Text(input,"native_node_columns")=="global_node,source_node_id,mass_kg,total_inertia_kg_m2,physical_inertia_kg_m2,added_inertia_kg_m2",
        "Assembly native coefficient columns changed");
    const auto& nodes=WallArray(input,"native_nodes",s.nodes.size());a.native_nodes.resize(s.nodes.size());
    for(std::size_t n=0;n<s.nodes.size();++n) {
        const auto& row=AssemblyRow(nodes[n],6);
        Require(AssemblyId(row[0])==n&&AssemblyId(row[1])==s.nodes[n].source_id,"Assembly native coefficient source mapping changed");
        for(unsigned j=0;j<4;++j)a.native_nodes[n][j]=AssemblyNumber(row[j+2]);
        const auto& c=a.native_nodes[n];Require(c[0]>0&&c[1]>0&&c[2]>0&&c[3]>=0,"Assembly native coefficients are invalid");
        AssemblyNear(c[1],static_cast<long double>(c[2])+c[3]); // Total J remains authoritative.
    }
    const auto& ledger=WallArray(input,"part_native_ledger",s.parts.size());
    Require(Text(input,"part_native_columns")=="source_part_id,parent_count,qeph_count,t3_count,mass_kg,total_inertia_kg_m2,physical_inertia_kg_m2,added_inertia_kg_m2",
        "Assembly part native ledger columns changed");
    for(std::size_t i=0;i<s.parts.size();++i) {
        const auto& p=s.parts[i];const auto& row=AssemblyRow(ledger[i],8);std::size_t q=0,t=0;
        for(std::size_t j=p.first_parent;j<p.first_parent+p.parent_count;++j)(s.parents[j].arity==4?q:t)++;
        Require(AssemblyId(row[0])==p.id&&AssemblyId(row[1])==p.parent_count&&AssemblyId(row[2])==q&&AssemblyId(row[3])==t,
            "Assembly part native ledger source counts changed");
        for(unsigned j=4;j<8;++j)Require(AssemblyNumber(row[j])>=(j==7?0:std::numeric_limits<double>::min()),"Invalid part native coefficient");
    }
}
} // namespace crash::output::replay_detail
