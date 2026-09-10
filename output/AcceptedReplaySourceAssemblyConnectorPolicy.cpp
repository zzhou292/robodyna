#include "AcceptedReplaySourceAssemblyConnectorValues.h"
namespace crash::output::replay_detail {
void CheckAssemblySourceScope(const Value& c,const Value& manifest,bool seven) {
    Require(Text(c,"scope")== (seven?AssemblyConnectorScope:
        "Original six-part Yaris component, internal nodal rigid groups active, external connections explicitly released")&&
        Text(manifest,"scope")== (seven?AssemblyConnectorScope:
        "Original connected six-part Yaris component; complete internal groups and explicit released external connections"),
        "Assembly configuration/manifest scope does not match its original source inventory");
}
source::ArtifactIdentity AssemblySourceIdentity(const Value& c) {
    const auto& input=Member(c,"input");Require(input.IsObject(),"Assembly source declarations are absent");
    if(!input.HasMember("connectors")) {
        Require(!c.HasMember("connector_storage_limits")&&!c.HasMember("connector_work_scope"),"Undeclared connector configuration");
        return source::PinnedYarisSixPartInventory();
    }
    const auto& weld=Member(input,"connectors");
    Require(Text(weld,"kind")==AssemblyConnectorKind&&Text(weld,"policy")==AssemblyConnectorPolicy,
        "Only the named source-unit spotweld policy admits the seven-part inventory");
    return source::PinnedYarisSevenPartInventory();
}
void CheckAssemblyConnectorProperty(const Value& p,std::uint64_t id,const source::Data& s) {
    Require(p.IsObject()&&p.MemberCount()==9&&id&&Unsigned(p,"generated_property_id")==id&&
        std::none_of(s.sections.begin(),s.sections.end(),[&](const auto& section){return section.id==id;}),
        "Generated TYPE25 property cannot replace an original source section");
    // Independent literal verification of the named converter policy. These are
    // SI source declarations, not a material response or native mass producer.
    // unitsystemdefaults.cxx mm_s_Mg; prop_p25_spr_axi Ileng0; see source adapter.
    const double mass_unit=1000.,length_unit=.001,inertia_unit=mass_unit*length_unit*length_unit;
    const double force_unit=mass_unit*length_unit,moment_unit=force_unit*length_unit;
    AssemblyEqual(Real(p,"mass_kg"),.001e-3*mass_unit);
    AssemblyEqual(Real(p,"isotropic_inertia_kg_m2"),.01e-3*inertia_unit);
    const double stiffness[]={100.e3*mass_unit,100.e3*mass_unit,1000.e3*inertia_unit,1000.e3*inertia_unit};
    for(const char* key:{"stiffness","damping","failure_negative","failure_positive","failure_weight","failure_exponent"})WallNumbers(p,key,4);
    for(unsigned c=0;c<4;++c) {
        AssemblyEqual(p["stiffness"][c].GetDouble(),stiffness[c]);AssemblyEqual(p["damping"][c].GetDouble(),0);
        const double threshold=1.e30*(c<2?force_unit:moment_unit);
        AssemblyEqual(p["failure_positive"][c].GetDouble(),threshold);AssemblyEqual(p["failure_negative"][c].GetDouble(),-threshold);
        AssemblyEqual(p["failure_weight"][c].GetDouble(),1);AssemblyEqual(p["failure_exponent"][c].GetDouble(),2);
    }
}
std::array<double,2> AssemblyConnectorKinetic(const Bundle& b,const Value& v) {
    if(b.assembly->connectors.empty()) {Require(!v.HasMember("connector_kinetic_J"),"Undeclared connector kinetic subtotal");return {};}
    const auto& row=WallNumbers(v,"connector_kinetic_J",2);
    const std::array<double,2> result{row[0].GetDouble(),row[1].GetDouble()};
    Require(result[0]>=0&&result[1]>=0,"Connector kinetic subtotal is negative");return result;
}
}
