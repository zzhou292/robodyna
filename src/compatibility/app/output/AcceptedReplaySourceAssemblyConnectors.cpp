#include "AcceptedReplaySourceAssemblyConnectorValues.h"
namespace crash::output::replay_detail {
void ReadAssemblyConnectors(Bundle& b,const Value& c) {
    auto& a=*b.assembly;const auto& s=a.source.data();const auto& input=Member(c,"input");
    if(s.internal_spotwelds.empty()) {
        Require(!input.HasMember("connectors")&&!c.HasMember("connector_storage_limits")&&!c.HasMember("connector_work_scope"),
            "Six-part source has no internal connector mechanics");return;
    }
    const auto& v=Member(input,"connectors");
    Require(v.IsObject()&&v.MemberCount()==10&&s.identity.sha256==source::PinnedYarisSevenPartInventory().sha256&&s.internal_spotwelds.size()==1&&
        Text(v,"kind")==AssemblyConnectorKind&&Text(v,"policy")==AssemblyConnectorPolicy&&
        Unsigned(v,"source_instance_id")==a.instance&&Unsigned(v,"connection_count")==s.internal_spotwelds.size()&&
        Unsigned(v,"property_count")==1,"Connector source identity/policy/count differs");
    const auto& units=WallNumbers(v,"source_units_to_SI",3);const double expected_units[]{1000.,.001,1.};
    for(unsigned i=0;i<3;++i){AssemblyEqual(units[i].GetDouble(),expected_units[i]);}
    const auto& properties=WallArray(v,"properties",1);const auto id=Unsigned(properties[0],"generated_property_id");
    CheckAssemblyConnectorProperty(properties[0],id,s);
    Require(Text(c,"connector_work_scope")==AssemblyConnectorWorkScope,"Connector native work cannot be relabelled shell work or dissipation");
    const auto& limits=Member(c,"connector_storage_limits");
    Require(limits.IsObject()&&limits.MemberCount()==3&&Unsigned(limits,"max_connections")>=s.internal_spotwelds.size()&&Unsigned(limits,"max_connections")<=1024&&
        Unsigned(limits,"max_device_bytes")>0&&Unsigned(limits,"max_device_bytes")<=2*1024*1024&&
        Unsigned(limits,"max_host_bytes")>0&&Unsigned(limits,"max_host_bytes")<=8*1024*1024,"Invalid connector resident storage declaration");
    a.connector_dt_fraction=Real(Member(c,"deformation_limits"),"maximum_native_dt_fraction");
    Require(a.connector_dt_fraction>0&&a.connector_dt_fraction<=1,"Invalid connector native timestep fraction");
    const auto& rows=WallArray(v,"connections",s.internal_spotwelds.size());
    const auto& masses=WallArray(v,"endpoint_contributions",2*s.internal_spotwelds.size());
    Require(Text(v,"endpoint_columns")=="source_element_id,generated_property_id,local_endpoint,global_node,source_node_id,mass_kg,inertia_kg_m2,total_node_mass_kg,total_node_inertia_kg_m2",
        "Connector endpoint M/J ledger columns differ");
    a.connectors.resize(rows.Size());a.connector_nodes.resize(s.nodes.size());
    for(std::size_t i=0;i<s.internal_spotwelds.size();++i) {
        const auto& source=s.internal_spotwelds[i];const auto& r=rows[i];auto& declaration=a.connectors[i];
        declaration.element=source.record.id;declaration.property=id;declaration.nodes=source.nodes;
        Require(r.IsObject()&&r.MemberCount()==8&&Unsigned(r,"source_element_id")==source.record.id&&Unsigned(r,"generated_property_id")==id&&
            source.record.cards.size()==2&&source.record.cards[0].blank_mask==254&&source.record.cards[1].blank_mask==252,
            "Connector WID/property or supported blank-card scope changed");
        const auto& nids=WallArray(r,"source_node_ids",2);const auto& nodes=WallArray(r,"global_nodes",2);
        const auto& lines=WallArray(r,"source_card_lines",2);const auto& positions=WallNumbers(r,"reference_positions_m",6);
        for(unsigned e=0;e<2;++e) {
            const auto n=source.nodes[e];Require(n<s.nodes.size()&&!a.grouped_node[n]&&AssemblyId(nodes[e])==n&&
                AssemblyId(nids[e])==source.record.node_ids[e]&&s.nodes[n].source_id==source.record.node_ids[e]&&
                AssemblyId(lines[e])==source.record.cards[e].source_line,"Connector endpoint/card/source-group association differs");
            const auto x=ConnectorSourcePosition(s.nodes[n]);for(unsigned j=0;j<3;++j)AssemblyEqual(positions[3*e+j].GetDouble(),double(x[j]));
            const auto& mass=AssemblyRow(masses[2*i+e],9);const std::uint64_t ids[]{source.record.id,id,e,n,s.nodes[n].source_id};
            for(unsigned j=0;j<5;++j)Require(AssemblyId(mass[j])==ids[j],"Connector endpoint mass order/identity changed");
            const double values[]{.5*Real(properties[0],"mass_kg"),.5*Real(properties[0],"isotropic_inertia_kg_m2")};
            for(unsigned j=0;j<2;++j){AssemblyEqual(AssemblyNumber(mass[5+j]),values[j]);a.connector_nodes[n][j]+=values[j];}
        }
        const auto chord=ConnectorSourcePosition(s.nodes[source.nodes[1]])-ConnectorSourcePosition(s.nodes[source.nodes[0]]);
        declaration.length=Real(r,"reference_length_m");Require(declaration.length>0,"Connector reference length is not positive");
        AssemblyNear(declaration.length,chord.Length());const auto x=chord/chord.Length();
        auto seed=ConnectorVector{0,1,0};if(chord.Cross(seed).Length()/chord.Length()<1e-5L)seed={1,0,0};
        auto expected=chord.Cross(seed).Cross(chord);expected/=expected.Length();
        const auto transverse=ConnectorVectorValue(Member(r,"transverse_axis"));ConnectorUnit(transverse);ConnectorDirection(transverse,expected);
        for(unsigned j=0;j<3;++j)declaration.transverse[j]=double(transverse[j]);
    }
    // Read-only arithmetic over explicitly declared native coefficients, not a
    // shell mass formula. Shell physical/added J remain separate and unchanged.
    for(std::size_t n=0;n<s.nodes.size();++n)for(unsigned j=0;j<2;++j)a.native_nodes[n][j]+=a.connector_nodes[n][j];
    for(std::size_t i=0;i<s.internal_spotwelds.size();++i)for(unsigned e=0;e<2;++e) {
        const auto n=s.internal_spotwelds[i].nodes[e];const auto& row=masses[2*i+e];
        for(unsigned j=0;j<2;++j)AssemblyEqual(AssemblyNumber(row[7+j]),a.native_nodes[n][j]);
    }
}
}
