#include "MaterialDeclarationFields.h"
#include <array>
#include <algorithm>
namespace crash::modelio::assembly::reader {
Material ReadLaw1Material(const Value& value,const Data& data) {
    Require(data.schema==SectionInventorySchema,"Layered LAW1 requires the explicit V3 inventory");
    constexpr std::array<const char*,7> allowed{{"material_id","density_kg_m3","young_pa","poisson_ratio","cards","source","material_law"}};
    Require(value.MemberCount()==allowed.size(),"Elastic material contains nonelastic or missing declarations");
    for(auto it=value.MemberBegin();it!=value.MemberEnd();++it)
        Require(std::any_of(allowed.begin(),allowed.end(),[&](const char* key){return it->name==key;}),
                "Elastic material contains a nonelastic declaration");
    Material m;m.law=MaterialLaw::LayeredLaw1;ReadMaterialTuple(value,m);
    m.source=Block(Member(value,"source"));m.cards=Cards(value,1);
    Require(m.source.keyword=="*MAT_ELASTIC"&&m.cards.size()==1&&m.cards[0].blank_mask==240,
            "Unsupported assembly elastic card options");
    Require(m.density_kg_m3>0&&m.young_pa>0&&m.poisson_ratio>=0&&m.poisson_ratio<.5,
            "Invalid assembly elastic tuple");
    const auto& card=m.cards[0];
    Require(card.names==std::vector<std::string>{"mid","ro","e","pr","reserved5","reserved6","reserved7","reserved8"},
            "Elastic source card column identity changed");
    Same(RequiredMaterialCard(card,0),double(m.id));
    Same(RequiredMaterialCard(card,1)*MaterialDensityScale(data.units),m.density_kg_m3);
    Same(RequiredMaterialCard(card,2)*MaterialStressScale(data.units),m.young_pa);
    Same(RequiredMaterialCard(card,3),m.poisson_ratio);
    return m;
}
Material ReadMaterial(const Value& value,const Data& data) {
    if(data.schema!=SectionInventorySchema) {
        Require(!value.HasMember("material_law"),"Explicit material-law tags require V3");
        return ReadLaw44Material(value,data);
    }
    const auto law=Text(value,"material_law");
    if(law=="layered_law1")return ReadLaw1Material(value,data);
    Require(law=="layered_law44","Unsupported explicit layered material law");
    return ReadLaw44Material(value,data);
}
} // namespace crash::modelio::assembly::reader
